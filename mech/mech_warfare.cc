// Copyright 2014-2019 Josh Pieper, jjp@pobox.com.  All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "mech_warfare.h"

#include <optional>

#include <boost/property_tree/json_parser.hpp>

#include <fmt/format.h>

#include "mjlib/base/fail.h"
#include "mjlib/base/program_options_archive.h"

#include "base/common.h"
#include "base/context_full.h"
#include "base/now.h"
#include "base/property_tree_archive.h"

#include "drive_command.h"
#include "mech_telemetry.h"
#include "mech_warfare_data.h"

namespace mjmech {
namespace mech {

namespace {
const double kTelemetryTimeoutS = 0.2;

typedef HerkuleXConstants HC;

double Limit(double value, double min, double max) {
  if (value > max) { return max; }
  if (value < min) { return min; }
  return value;
}

double Limit(double value, double max) {
  return Limit(value, -max, max);
}

typedef MechWarfareData Data;
}

class MechWarfare::Impl : boost::noncopyable {
 public:
  Impl(MechWarfare* parent,
       base::Context& context)
      : parent_(parent),
        context_(context),
        service_(context.service),
        server_(service_),
        timer_(service_) {
    context_.telemetry_registry->Register("mech_warfare", &data_signal_);
  }

  void AsyncStart(mjlib::io::ErrorCallback handler) {
    try {
      RippleConfig ripple_config;
      ripple_config = LoadRippleConfig();

      parent_->m_.gait_driver->SetGait(std::unique_ptr<RippleGait>(
                                           new RippleGait(ripple_config)));

      NetworkListen();
      StartTimer();
    } catch (mjlib::base::system_error& se) {
      service_.post(std::bind(handler, se.code()));
      return;
    }

    parent_->parameters_.children.Start(handler);
  }

  RippleConfig LoadRippleConfig() {
    RippleConfig ripple_config;
    try {
      std::ifstream inf(parent_->parameters_.gait_config);
      if (!inf.is_open()) {
        throw mjlib::base::system_error::syserrno("error opening config");
      }

      boost::property_tree::ptree tree;
      boost::property_tree::read_json(inf, tree);
      auto optional_child = tree.get_child_optional("gaitconfig.ripple");
      if (!optional_child) {
        throw mjlib::base::system_error::einval("could not find ripple config in file");
      }

      base::PropertyTreeReadArchive(
          *optional_child,
          base::PropertyTreeReadArchive::kErrorOnMissing).Accept(&ripple_config);

      std::string type = tree.get<std::string>("ikconfig.iktype");
      auto& leg_configs = ripple_config.mechanical.leg_config;
      for (size_t i = 0; i < leg_configs.size(); i++) {
        if (type == "Mammal") {
          MammalIK::Config config;

          std::string field = fmt::format("ikconfig.leg.{}", i);
          auto optional_child = tree.get_child_optional(field);
          if (!optional_child) {
            throw mjlib::base::system_error::einval("could not locate field: " + field);
          }

          base::PropertyTreeReadArchive(
              *optional_child,
              base::PropertyTreeReadArchive::kErrorOnMissing).Accept(&config);
          leg_configs[i].leg_ik =
              boost::shared_ptr<IKSolver>(new MammalIK(config));
        } else if (type == "Lizard") {
          LizardIK::Config config;

          std::string field = fmt::format("ikconfig.leg.{}", i);
          auto optional_child = tree.get_child_optional(field);
          if (!optional_child) {
            throw mjlib::base::system_error::einval("could not locate field: " + field);
          }

          base::PropertyTreeReadArchive(
              *optional_child,
              base::PropertyTreeReadArchive::kErrorOnMissing).Accept(&config);
          leg_configs[i].leg_ik =
              boost::shared_ptr<IKSolver>(new LizardIK(config));
        } else {
          throw mjlib::base::system_error::einval("unknown iktype: " + type);
        }
      }
    } catch (mjlib::base::system_error& se) {
      se.code().Append(
          "while opening config: '" + parent_->parameters_.gait_config + "'");
      throw;
    }

    return ripple_config;
  }

  void NetworkListen() {
    boost::asio::ip::udp::endpoint endpoint(
        boost::asio::ip::udp::v4(), parent_->parameters_.port);
    server_.open(boost::asio::ip::udp::v4());
    server_.bind(endpoint);
    StartRead();
  }

  void StartRead() {
    server_.async_receive_from(
        boost::asio::buffer(receive_buffer_),
        receive_endpoint_,
        std::bind(&Impl::HandleRead, this,
                  std::placeholders::_1,
                  std::placeholders::_2));
  }

  void StartTimer() {
    timer_.expires_from_now(base::ConvertSecondsToDuration(
                                parent_->parameters_.period_s));
    timer_.async_wait(std::bind(&Impl::HandleTimer, this,
                                std::placeholders::_1));
  }

  void WorkTowardsIdle() {
    // Try to get into an idle state.  First, we need to sit down,
    // then power off the servos.

    switch (data_.mode) {
      case Data::Mode::kIdle: {
        mjlib::base::AssertNotReached();
        break;
      }
      case Data::Mode::kTurretBias: {
        // We haven't even turned on the servos yet.
        data_.mode = Data::Mode::kIdle;
        break;
      }
      case Data::Mode::kPrepositioning: // fall through
      case Data::Mode::kStanding:
      case Data::Mode::kManual:
      case Data::Mode::kDrive: {
        // Start sitting.
        data_.sitting_start_timestamp = base::Now(service_);
        parent_->m_.gait_driver->CommandSitting();
        data_.mode = Data::Mode::kSitting;
        break;
      }
      case Data::Mode::kSitting: {
        // TODO: Wait for our timeout.
        const auto now = base::Now(service_);
        const auto elapsed_s = base::ConvertDurationToSeconds(
            now - data_.sitting_start_timestamp);
        if (elapsed_s > parent_->parameters_.sitting_timeout_s) {
          data_.mode = Data::Mode::kIdle;
          parent_->m_.gait_driver->SetFree();
          parent_->m_.servo_monitor->ExpectTorqueOff();
        }
        break;
      }
    }
  }

  void HandleTimer(mjlib::base::error_code ec) {
    if (ec == boost::asio::error::operation_aborted) { return; }
    mjlib::base::FailIf(ec);

    StartTimer();

    if (data_.mode != Data::Mode::kIdle) {
      const double elapsed_s = base::ConvertDurationToSeconds(
          base::Now(service_) - data_.last_command_timestamp);
      if (elapsed_s > parent_->parameters_.idle_timeout_s) {
        WorkTowardsIdle();
      }
    }

    switch (data_.mode) {
      case Data::Mode::kIdle:  // fall through
      case Data::Mode::kTurretBias:  // fall through
      case Data::Mode::kPrepositioning:  // fall through
      case Data::Mode::kStanding:  // fall through
      case Data::Mode::kSitting:  // fall through
      case Data::Mode::kManual:
        break;
      case Data::Mode::kDrive: {
        DoDrive();
        break;
      }
    }

    data_.timestamp = base::Now(context_.service);
    data_signal_(&data_);

    MechTelemetry telemetry;
    telemetry.timestamp = data_.timestamp;
    telemetry.turret_absolute_deg = parent_->m_.turret->data().absolute.x_deg;
    telemetry.total_fire_time_s = parent_->m_.turret->data().total_fire_time_s;
    telemetry.servo_min_voltage_V = servo_min_voltage_V_;
    telemetry.servo_max_voltage_V = servo_max_voltage_V_;
    telemetry.mech_mode = static_cast<int>(data_.mode);
    telemetry.target_data = parent_->m_.video->target_tracker()->data();

    telemetry_->SetTelemetry(
        "mech",
        mjlib::telemetry::TelemetryWriteArchive<MechTelemetry>::Serialize(&telemetry),
        base::Now(service_) +
        mjlib::base::ConvertSecondsToDuration(kTelemetryTimeoutS));
  }

  void DoDrive() {
    const auto& p = parent_->parameters_;

    TurretCommand turret;
    Command gait;

    turret.fire_control = data_.current_drive.fire_control;
    if (data_.current_drive.turret_target_relative) {
      TurretCommand::TargetRelative relative;
      relative.x = data_.current_drive.turret_target_relative->x;
      relative.y = data_.current_drive.turret_target_relative->y;
      turret.target_relative = relative;
    } else if (data_.current_drive.turret_rate_dps) {
      turret.rate = TurretCommand::Rate();
      turret.rate->x_deg_s = data_.current_drive.turret_rate_dps->yaw;
      turret.rate->y_deg_s = data_.current_drive.turret_rate_dps->pitch;
    }

    const auto body_offset_mm =
        data_.current_drive.body_offset_mm + p.body_offset_mm;
    gait.body_x_mm = body_offset_mm.x;
    gait.body_y_mm = body_offset_mm.y;
    gait.body_z_mm = body_offset_mm.z;

    // NOTE jpieper: Yeah, I know that rotations don't compose this
    // way.  This is really only a fudge factor though, so I'm intent
    // on getting away with it.
    const base::Euler body_attitude_deg = [&]() {
      base::Euler result;
      result.roll = (data_.current_drive.body_attitude_deg.roll +
                     p.body_attitude_deg.roll);
      result.pitch = (data_.current_drive.body_attitude_deg.pitch +
                      p.body_attitude_deg.pitch);
      result.yaw = (data_.current_drive.body_attitude_deg.yaw +
                    p.body_attitude_deg.yaw);
      return result;
    }();

    gait.body_pitch_deg = body_attitude_deg.pitch;
    gait.body_roll_deg = body_attitude_deg.roll;
    gait.body_yaw_deg = body_attitude_deg.yaw;

    auto body_mm_s = base::Quaternion::FromEuler(
        0.0,
        0.0,
        base::Radians(parent_->m_.turret->data().absolute.x_deg)).Rotate(
            data_.current_drive.drive_mm_s);

    const bool active = (data_.current_drive.drive_mm_s != base::Point3D());
    gait.lift_height_percent = active ? 100.0 : 0.0;

    if (active) {
      const double heading_deg = body_mm_s.heading_deg();
      const double forward_error_deg = base::WrapNeg180To180(heading_deg);
      const double drive_heading_deg =
          data_.current_drive.drive_mm_s.heading_deg();
      double error_deg = 0.0;
      if (std::abs(forward_error_deg) < 145 &&
          std::abs(drive_heading_deg) < 135) {
        // Work to end up being forward.
        error_deg = forward_error_deg;
      } else {
        // Work to end up being backward.
        const double reverse_error_deg =
            base::WrapNeg180To180(heading_deg - 180.0);
        error_deg = reverse_error_deg;
      }
      if (data_.current_drive.freeze_rotation) {
        error_deg = 0.0;
      }
      const double correction_dps = Limit(error_deg * p.drive_rotate_factor,
                                          p.drive_max_rotate_dps);
      gait.rotate_deg_s += correction_dps;

      // Limit the magnitude of body velocity based on error.
      const double max_translate_mm_s =
          Limit((1.0f - ((std::abs(error_deg) - p.drive_min_error_deg) /
                         (p.drive_max_error_deg - p.drive_min_error_deg))),
                0.0f, 1.0f) *
          p.drive_max_translate_mm_s;

      if (body_mm_s.length() > max_translate_mm_s) {
        body_mm_s = body_mm_s.scaled(max_translate_mm_s / body_mm_s.length());
      }
    }

    gait.translate_x_mm_s = body_mm_s.x;
    gait.translate_y_mm_s = body_mm_s.y;

    // Give the commands to our members.
    parent_->m_.turret->SetCommand(turret);
    parent_->m_.gait_driver->SetCommand(gait);
  }

  void HandleRead(mjlib::base::error_code ec, std::size_t size) {
    mjlib::base::FailIf(ec);

    std::string data(receive_buffer_, size);
    StartRead();

    log_.debug("got data: " + data);

    boost::property_tree::ptree tree;
    try {
      std::istringstream inf(data);
      boost::property_tree::read_json(inf, tree);
    } catch (std::exception& e) {
      std::cerr << "error reading network command: " << e.what() << "\n";
      return;
    }

    HandleMessage(tree);
  }

  void HandleMessage(const boost::property_tree::ptree& tree) {
    data_.last_command_timestamp = base::Now(service_);

    auto optional_drive = tree.get_child_optional("drive");
    if (optional_drive) {
      HandleMessageDrive(*optional_drive);
      return;
    }

    auto optional_gait = tree.get_child_optional("gait");
    if (optional_gait) { HandleMessageGait(*optional_gait); }

    auto optional_turret = tree.get_child_optional("turret");
    if (optional_turret) { HandleMessageTurret(*optional_turret); }
  }

  void StartTurretBias() {
    data_.turret_bias_start_timestamp = base::Now(service_);
    parent_->m_.turret->StartBias();

    // During this bias period, we will also verify that each of our
    // servos is configured properly.
    servos_to_configure_ =
        ServoMonitor::SplitServoIds(parent_->parameters_.config_servos);
    DoNextServoConfigure();
  }

  void DoNextServoConfigure() {
    // For now, we are only verifying a single RAM address.  This will
    // be harder when we want to verify multiple things.
    if (servos_to_configure_.empty()) { return; }

    const int this_id = servos_to_configure_.back();
    servos_to_configure_.pop_back();

    parent_->m_.servo_base->RamRead(
        this_id, HC::min_voltage(),
        std::bind(&Impl::HandleServoConfigure, this, this_id,
                  std::placeholders::_1, std::placeholders::_2));
  }

  void HandleServoConfigure(int servo_id,
                            mjlib::base::error_code ec,
                            int value) {
    if (ec == boost::asio::error::operation_aborted ||
        ec == herkulex_error::synchronization_error) {
      // Ignore this.
    } else {
      mjlib::base::FailIf(ec);
    }

    const int measured = value;
    const int expected =
        static_cast<int>(parent_->parameters_.servo_min_voltage_counts);
    if (measured != expected) {
      log_.warn(fmt::format(
                    "Servo {} has misconfigured min_voltage 0x{:02X} != 0x{:02X}",
                    servo_id, measured, expected));
    }

    DoNextServoConfigure();
  }

  void MaybeEnterActiveMode(Data::Mode mode) {
    const auto now = base::Now(service_);
    switch (data_.mode) {
      case Data::Mode::kIdle: {
        // Start the turret bias process.
        StartTurretBias();
        data_.mode = Data::Mode::kTurretBias;
        break;
      }
      case Data::Mode::kTurretBias: {
        // Wait until the turret bias process is finished and all
        // servos are configured.
        if (!servos_to_configure_.empty()) {
          break;
        }

        const double elapsed_s = base::ConvertDurationToSeconds(
            now - data_.turret_bias_start_timestamp);
        if (elapsed_s > parent_->parameters_.turret_bias_timeout_s) {
          parent_->m_.gait_driver->CommandPrepositioning();
          data_.prepositioning_start_timestamp = now;
          data_.mode = Data::Mode::kPrepositioning;
          parent_->m_.servo_monitor->ExpectTorqueOn();
        }
        break;
      }
      case Data::Mode::kPrepositioning: {
        const double elapsed_s = base::ConvertDurationToSeconds(
            now - data_.prepositioning_start_timestamp);
        if (elapsed_s > parent_->parameters_.prepositioning_timeout_s) {
          parent_->m_.gait_driver->CommandStandup();
          data_.standing_start_timestamp = now;
          data_.mode = Data::Mode::kStanding;
        }
        break;
      }
      case Data::Mode::kStanding: {
        const double elapsed_s = base::ConvertDurationToSeconds(
            now - data_.standing_start_timestamp);
        if (elapsed_s > parent_->parameters_.standing_timeout_s) {
          data_.mode = mode;
        }
        break;
      }
      case Data::Mode::kSitting: {
        // We just have to wait for this to finish.
        break;
      }
      case Data::Mode::kManual: // fall-through
      case Data::Mode::kDrive: {
        data_.mode = mode;
        break;
      }
    }
  }

  void HandleMessageGait(const boost::property_tree::ptree& tree) {
    Command command;
    base::PropertyTreeReadArchive(
        tree, base::PropertyTreeReadArchive::kErrorOnMissing).Accept(&command);

    MaybeEnterActiveMode(Data::Mode::kManual);

    if (data_.mode == Data::Mode::kManual) {
      parent_->m_.gait_driver->SetCommand(command);
    }
  }

  void HandleMessageTurret(const boost::property_tree::ptree& tree) {
    TurretCommand command;
    base::PropertyTreeReadArchive(
        tree, base::PropertyTreeReadArchive::kErrorOnMissing).Accept(&command);

    MaybeEnterActiveMode(Data::Mode::kManual);

    if (data_.mode == Data::Mode::kManual) {
      parent_->m_.turret->SetCommand(command);
    }
  }

  void HandleMessageDrive(const boost::property_tree::ptree& tree) {
    DriveCommand command;
    base::PropertyTreeReadArchive(
        tree, base::PropertyTreeReadArchive::kErrorOnMissing).Accept(&command);

    data_.current_drive = command;
    MaybeEnterActiveMode(Data::Mode::kDrive);

    if (data_.mode == Data::Mode::kDrive) {
      // Immediately update ourselves.
      DoDrive();
    }
  }

  void HandleServoData(const ServoMonitor::ServoData* data) {
    bool first = true;
    for (const auto& servo: data->servos) {
      if (servo.last_update.is_not_a_date_time()) { continue; }
      if (servo.voltage_V == 0.0) { continue; }
      if (first || servo.voltage_V < servo_min_voltage_V_) {
        servo_min_voltage_V_ = servo.voltage_V;
      }
      if (first || servo.voltage_V > servo_max_voltage_V_) {
        servo_max_voltage_V_ = servo.voltage_V;
      }
      first = false;
    }
  }

  MechWarfare* const parent_;
  base::Context& context_;
  boost::asio::io_service& service_;
  boost::program_options::options_description options_;

  base::LogRef log_ = base::GetLogInstance("MechWarfare");

  boost::asio::ip::udp::socket server_;
  char receive_buffer_[3000] = {};
  boost::asio::ip::udp::endpoint receive_endpoint_;

  mjlib::io::DeadlineTimer timer_;

  std::shared_ptr<McastTelemetryInterface> telemetry_;

  std::vector<int> servos_to_configure_;
  double servo_min_voltage_V_ = 0.0;
  double servo_max_voltage_V_ = 0.0;

  Data data_;
  boost::signals2::signal<void (const Data*)> data_signal_;
};

MechWarfare::MechWarfare(base::Context& context)
    : service_(context.service),
      impl_(new Impl(this, context)) {
  m_.multiplex_client = std::make_unique<MultiplexClient>(service_, *context.factory);
  m_.servo_base.reset(new Mech::ServoBase(service_, *context.factory));
  m_.servo.reset(new Mech::Servo(m_.servo_base.get()));
  m_.moteus_servo = std::make_unique<MoteusServo>(service_);

  m_.multiplex_client->RequestClient([this](const auto& ec, auto* client) {
        mjlib::base::FailIf(ec);
        m_.moteus_servo->SetClient(client);
      });

  m_.servo_selector = std::make_unique<ServoSelector>();
  m_.servo_selector->AddInterface("herkulex", m_.servo.get());
  m_.servo_selector->AddInterface("moteus", m_.moteus_servo.get());

  m_.imu.reset(new MjmechImuDriver(context));
  m_.ahrs.reset(new Ahrs(context, m_.imu->imu_data_signal()));
  m_.gait_driver.reset(new GaitDriver(service_,
                                      context.telemetry_registry.get(),
                                      m_.servo_selector.get(),
                                      m_.ahrs->ahrs_data_signal()));
  m_.servo_iface.reset(
      new ServoMonitor::HerkuleXServoConcrete<Mech::ServoBase>(
          m_.servo_base.get()));
  m_.servo_monitor.reset(new ServoMonitor(context, m_.servo_iface.get()));
  m_.servo_monitor->parameters()->servos = "0-11,98";
  m_.turret.reset(new Turret(context, m_.servo_base.get()));
  m_.video.reset(new VideoSenderApp(context));

  impl_->telemetry_ = m_.video->telemetry_interface().lock();
  m_.servo_monitor->data_signal()->connect(
      std::bind(&Impl::HandleServoData, impl_.get(),
                std::placeholders::_1));

  m_.video->target_tracker()->data_signal()->connect([this](const TargetTrackerData* data) {
      std::optional<base::Point3D> point3d;
      if (data->target) {
        point3d = base::Point3D(data->target->center.x,
                                data->target->center.y,
                                0.0);
      }
      m_.turret->UpdateTrackedTarget(point3d);
    });

  mjlib::base::ProgramOptionsArchive(&impl_->options_).Accept(&parameters_);
}

MechWarfare::~MechWarfare() {}

void MechWarfare::AsyncStart(mjlib::io::ErrorCallback handler) {
  impl_->AsyncStart(handler);
}

boost::program_options::options_description* MechWarfare::options() {
  return &impl_->options_;
}

}
}
