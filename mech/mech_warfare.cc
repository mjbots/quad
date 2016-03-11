// Copyright 2014-2016 Josh Pieper, jjp@pobox.com.  All rights reserved.
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

#include <boost/property_tree/json_parser.hpp>

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

  void AsyncStart(base::ErrorHandler handler) {
    try {
      RippleConfig ripple_config;
      ripple_config = LoadRippleConfig();

      parent_->m_.gait_driver->SetGait(std::unique_ptr<RippleGait>(
                                           new RippleGait(ripple_config)));

      NetworkListen();
      StartTimer();
    } catch (base::SystemError& se) {
      service_.post(std::bind(handler, se.error_code()));
      return;
    }

    parent_->parameters_.children.Start(handler);
  }

  RippleConfig LoadRippleConfig() {
    RippleConfig ripple_config;
    try {
      std::ifstream inf(parent_->parameters_.gait_config);
      if (!inf.is_open()) {
        throw base::SystemError::syserrno("error opening config");
      }

      boost::property_tree::ptree tree;
      boost::property_tree::read_json(inf, tree);
      auto optional_child = tree.get_child_optional("gaitconfig.ripple");
      if (!optional_child) {
        throw base::SystemError::einval("could not find ripple config in file");
      }

      base::PropertyTreeReadArchive(
          *optional_child,
          base::PropertyTreeReadArchive::kErrorOnMissing).Accept(&ripple_config);

      std::string type = tree.get<std::string>("ikconfig.iktype");
      auto& leg_configs = ripple_config.mechanical.leg_config;
      for (size_t i = 0; i < leg_configs.size(); i++) {
        if (type == "Mammal") {
          MammalIK::Config config;

          std::string field = (boost::format("ikconfig.leg.%d") % i).str();
          auto optional_child = tree.get_child_optional(field);
          if (!optional_child) {
            throw base::SystemError::einval("could not locate field: " + field);
          }

          base::PropertyTreeReadArchive(
              *optional_child,
              base::PropertyTreeReadArchive::kErrorOnMissing).Accept(&config);
          leg_configs[i].leg_ik =
              boost::shared_ptr<IKSolver>(new MammalIK(config));
        } else if (type == "Lizard") {
          LizardIK::Config config;

          std::string field = (boost::format("ikconfig.leg.%d") % i).str();
          auto optional_child = tree.get_child_optional(field);
          if (!optional_child) {
            throw base::SystemError::einval("could not locate field: " + field);
          }

          base::PropertyTreeReadArchive(
              *optional_child,
              base::PropertyTreeReadArchive::kErrorOnMissing).Accept(&config);
          leg_configs[i].leg_ik =
              boost::shared_ptr<IKSolver>(new LizardIK(config));
        } else {
          throw base::SystemError::einval("unknown iktype: " + type);
        }
      }
    } catch (base::SystemError& se) {
      se.error_code().Append(
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

  void HandleTimer(base::ErrorCode ec) {
    if (ec == boost::asio::error::operation_aborted) { return; }
    FailIf(ec);

    StartTimer();

    if (data_.mode != Data::Mode::kIdle) {
      const double elapsed_s = base::ConvertDurationToSeconds(
          base::Now(service_) - data_.last_command_timestamp);
      if (elapsed_s > parent_->parameters_.idle_timeout_s) {
        data_.mode = Data::Mode::kIdle;
        parent_->m_.gait_driver->SetFree();
        parent_->m_.servo_monitor->ExpectTorqueOff();
      }
    }

    switch (data_.mode) {
      case Data::Mode::kIdle: { break; }
      case Data::Mode::kTurretBias: { break; }
      case Data::Mode::kManual: { break; }
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

    telemetry_->SetTelemetry(
        "mech",
        base::TelemetryWriteArchive<MechTelemetry>::Serialize(&telemetry),
        base::Now(service_) +
        base::ConvertSecondsToDuration(kTelemetryTimeoutS));
  }

  void DoDrive() {
    const auto& p = parent_->parameters_;

    TurretCommand turret;
    Command gait;

    turret.fire_control = data_.current_drive.fire_control;
    turret.rate = TurretCommand::Rate();
    turret.rate->x_deg_s = data_.current_drive.turret_rate_dps.yaw;
    turret.rate->y_deg_s = data_.current_drive.turret_rate_dps.pitch;

    gait.body_x_mm = data_.current_drive.body_offset_mm.x;
    gait.body_y_mm = data_.current_drive.body_offset_mm.y;
    gait.body_z_mm = data_.current_drive.body_offset_mm.z;

    gait.body_pitch_deg = data_.current_drive.body_attitude_deg.pitch;
    gait.body_roll_deg = data_.current_drive.body_attitude_deg.roll;
    gait.body_yaw_deg = data_.current_drive.body_attitude_deg.yaw;

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

  void HandleRead(base::ErrorCode ec, std::size_t size) {
    FailIf(ec);

    std::string data(receive_buffer_, size);
    StartRead();

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
                            base::ErrorCode ec,
                            int value) {
    if (ec == boost::asio::error::operation_aborted ||
        ec == herkulex_error::synchronization_error) {
      // Ignore this.
    } else {
      base::FailIf(ec);
    }

    const int measured = value;
    const int expected =
        static_cast<int>(parent_->parameters_.servo_min_voltage_counts);
    if (measured != expected) {
      log_.warn((boost::format(
                     "Servo %d has misconfigured min_voltage 0x%02X != 0x%02X") %
                 servo_id %
                 measured %
                 expected).str());
    }

    DoNextServoConfigure();
  }

  void MaybeEnterActiveMode(Data::Mode mode) {
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

        const auto now = base::Now(service_);
        const double elapsed_s = base::ConvertDurationToSeconds(
            now - data_.turret_bias_start_timestamp);
        if (elapsed_s > parent_->parameters_.turret_bias_timeout_s) {
          data_.mode = mode;
          parent_->m_.servo_monitor->ExpectTorqueOn();
        }
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

  base::LogRef log_ = base::GetLogInstance("MechWarfare");

  boost::asio::ip::udp::socket server_;
  char receive_buffer_[3000] = {};
  boost::asio::ip::udp::endpoint receive_endpoint_;

  base::DeadlineTimer timer_;

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
  m_.servo_base.reset(new Mech::ServoBase(service_, *context.factory));
  m_.servo.reset(new Mech::Servo(m_.servo_base.get()));
  m_.imu.reset(new MjmechImuDriver(context));
  m_.ahrs.reset(new Ahrs(context, m_.imu->imu_data_signal()));
  m_.gait_driver.reset(new GaitDriver(service_,
                                      context.telemetry_registry.get(),
                                      m_.servo.get(),
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
}

MechWarfare::~MechWarfare() {}

void MechWarfare::AsyncStart(base::ErrorHandler handler) {
  impl_->AsyncStart(handler);
}

}
}
