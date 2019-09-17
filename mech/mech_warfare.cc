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

#include <fstream>
#include <optional>

#include <fmt/format.h>

#include "mjlib/base/fail.h"
#include "mjlib/base/json5_read_archive.h"
#include "mjlib/base/program_options_archive.h"

#include "mjlib/io/repeating_timer.h"

#include "base/common.h"
#include "base/context_full.h"
#include "base/now.h"

#include "drive_command.h"
#include "mech_telemetry.h"
#include "mech_warfare_data.h"

namespace pl = std::placeholders;

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

struct JointData {
  boost::posix_time::ptime timestamp;
  std::vector<ServoInterface::JointStatus> joints;
  int32_t missing = 0;
  mjlib::multiplex::ThreadedClient::Stats serial_stats;
  double cycle_time_s = 0.0;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(timestamp));
    a->Visit(MJ_NVP(joints));
    a->Visit(MJ_NVP(missing));
    a->Visit(MJ_NVP(serial_stats));
    a->Visit(MJ_NVP(cycle_time_s));
  }
};
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
    context_.telemetry_registry->Register("gait_data", &gait_data_signal_);
    context_.telemetry_registry->Register("joint_data", &joint_data_signal_);
    context_.telemetry_registry->Register("gait_command_state",
                                          &command_state_signal_);
    context_.telemetry_registry->Register("mech_warfare", &data_signal_);
  }

  void AsyncStart(mjlib::io::ErrorCallback handler) {
    try {
      RippleConfig ripple_config;
      ripple_config = LoadRippleConfig();

      parent_->m_.gait_driver->SetGait(std::unique_ptr<RippleGait>(
                                           new RippleGait(ripple_config)));

      NetworkListen();
    } catch (mjlib::base::system_error& se) {
      service_.post(std::bind(handler, se.code()));
      return;
    }

    parent_->parameters_.children.Start(
        [this, handler](auto ec) {
          mjlib::base::FailIf(ec);
          timer_.start(
              mjlib::base::ConvertSecondsToDuration(
                  parent_->parameters_.period_s),
              std::bind(&Impl::HandleTimer, this, pl::_1));
          service_.post(std::bind(handler, ec));
        });
  }

  struct LoadRipple {
    struct GaitConfig {
      RippleConfig ripple;

      template <typename Archive>
      void Serialize(Archive* a) {
        a->Visit(MJ_NVP(ripple));
      }
    };

    struct IkConfig {
      std::string iktype;
      std::vector<MammalIK::Config> leg;

      template <typename Archive>
      void Serialize(Archive* a) {
        a->Visit(MJ_NVP(iktype));
        a->Visit(MJ_NVP(leg));
      }
    };

    GaitConfig gaitconfig;
    IkConfig ikconfig;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(gaitconfig));
      a->Visit(MJ_NVP(ikconfig));
    }
  };

  RippleConfig LoadRippleConfig() {
    RippleConfig ripple_config;
    try {
      std::ifstream inf(parent_->parameters_.gait_config);
      if (!inf.is_open()) {
        throw mjlib::base::system_error::syserrno("error opening config");
      }

      auto load_ripple = mjlib::base::Json5ReadArchive::Read<LoadRipple>(inf);
      ripple_config = load_ripple.gaitconfig.ripple;
      const auto type = load_ripple.ikconfig.iktype;
      MJ_ASSERT(type == "Mammal");

      auto& leg_configs = ripple_config.mechanical.leg_config;
      for (size_t i = 0; i < leg_configs.size(); i++) {
        leg_configs[i].leg_ik =
            boost::shared_ptr<IKSolver>(
                new MammalIK(load_ripple.ikconfig.leg[i]));
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
        std::bind(&Impl::HandleRead, this, pl::_1, pl::_2));
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
        }
        break;
      }
      case Data::Mode::kFault: {
        // We never exit a fault case through this mechanism.
        break;
      }
    }
  }

  void HandleTimer(const mjlib::base::error_code& ec) {
    if (ec == boost::asio::error::operation_aborted) { return; }
    mjlib::base::FailIf(ec);

    const auto start = base::Now(service_);

    if (outstanding_) {
      data_.skipped_updates++;
      return;
    }

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
      case Data::Mode::kSitting: {
        break;
      }
      case Data::Mode::kManual: {
        DoManual();
        break;
      }
      case Data::Mode::kDrive: {
        DoDrive();
        break;
      }
      case Data::Mode::kFault: {
        // Everything should be stopped.
        parent_->m_.gait_driver->SetFree();
        break;
      }
    }

    auto result = parent_->m_.gait_driver->Update(
        parent_->parameters_.period_s);

    joint_command_ = result.command_state.joints;
    joint_data_.joints.clear();

    outstanding_ = true;
    parent_->m_.moteus_servo->Update(
        result.command_state.power_state,
        []() {
          ServoInterface::StatusOptions status;
          status.pose = true;
          status.velocity = true;

          // Our raspberry PI UART and multiplex register encoding
          // limit the amount of data that can be returned.  If we ask
          // for torque, we go over that limit. :(
          status.torque = false;

          status.voltage = false;
          status.temperature = false;
          status.error = true;
          return status;
        }(),
        &joint_command_,
        &joint_data_.joints,
        std::bind(&Impl::HandleServoUpdate, this, pl::_1, start));

    gait_data_signal_(&result.gait_data);

    std::sort(result.command_state.joints.begin(),
              result.command_state.joints.end(),
              [](const auto& lhs, const auto& rhs) {
                return lhs.address < rhs.address;
              });
    command_state_signal_(&result.command_state);
  }

  void CheckForJointErrors() {
    for (const auto& joint : joint_data_.joints) {
      if (joint.error) {
        // We need to immediately cut power to everything.
        log_.warn(fmt::format("servo {} reported error {}, idling",
                              joint.address, joint.error));
        data_.mode = Data::Mode::kFault;
        return;
      }
    }
  }

  void UpdateStatus() {
    auto min_max = [&](auto getter) -> std::pair<double, double> {
      bool first = true;
      std::pair<double, double> result = {};
      for (const auto& servo : joint_data_.joints) {
        const auto value = getter(servo);
        if (!value) { continue; }
        if (first || *value < result.first) {
          result.first = *value;
        }
        if (first || *value > result.second) {
          result.second = *value;
        }
        first = false;
      }
      return result;
    };

    std::tie(servo_min_voltage_V_, servo_max_voltage_V_) =
        min_max([](const auto& servo) { return servo.voltage; });
    std::tie(servo_min_temp_C_, servo_max_temp_C_) =
        min_max([](const auto& servo) { return servo.temperature_C; });
  }

  void CleanupJointData() {
    // Ensure that we have something for all 12 servos, even if some
    // are not present.
    std::set<int> present;
    for (const auto& joint : joint_data_.joints) {
      present.insert(joint.address);
    }
    joint_data_.missing = 0;
    for (int i = 1; i <= 12; i++) {
      if (present.count(i) == 0) {
        joint_data_.missing++;
        joint_data_.joints.push_back({});
        joint_data_.joints.back().address = i;
      }
    }

    if (multiplex_client_) {
      joint_data_.serial_stats = multiplex_client_->stats();
    }

    std::sort(joint_data_.joints.begin(), joint_data_.joints.end(),
              [](const auto& lhs, const auto& rhs) {
                return lhs.address < rhs.address;
              });
  }

  void HandleServoUpdate(const mjlib::base::error_code& ec,
                         boost::posix_time::ptime start) {
    outstanding_ = false;
    mjlib::base::FailIf(ec);

    joint_data_.timestamp = base::Now(service_);
    joint_data_.cycle_time_s =
        mjlib::base::ConvertDurationToSeconds(joint_data_.timestamp - start);
    CleanupJointData();

    joint_data_signal_(&joint_data_);

    CheckForJointErrors();
    UpdateStatus();

    data_.timestamp = base::Now(context_.service);
    data_signal_(&data_);

    MechTelemetry telemetry;
    telemetry.timestamp = data_.timestamp;
    telemetry.turret_absolute_deg = parent_->m_.turret->data().absolute.x_deg;
    telemetry.total_fire_time_s = parent_->m_.turret->data().total_fire_time_s;
    telemetry.servo_min_voltage_V = servo_min_voltage_V_;
    telemetry.servo_max_voltage_V = servo_max_voltage_V_;
    telemetry.servo_min_temp_C = servo_min_temp_C_;
    telemetry.servo_max_temp_C = servo_max_temp_C_;
    telemetry.mech_mode = static_cast<int>(data_.mode);
    telemetry.target_data = parent_->m_.video->target_tracker()->data();

    const auto& input_command = parent_->m_.gait_driver->input_command();
    telemetry.gait_x_mm_s = input_command.translate_x_mm_s;
    telemetry.gait_y_mm_s = input_command.translate_y_mm_s;
    telemetry.gait_rot_deg_s = input_command.rotate_deg_s;

    telemetry_->SetTelemetry(
        "mech",
        mjlib::telemetry::TelemetryWriteArchive<MechTelemetry>::Serialize(&telemetry),
        base::Now(service_) +
        mjlib::base::ConvertSecondsToDuration(kTelemetryTimeoutS));
  }

  void UpdateActive(Command* gait, bool want_active) {
    if (want_active && !data_.active) {
      // We can instantly go active.  Reset gait phase.
      gait->reset_phase = true;
      data_.active = true;
    } else if (data_.active && !want_active) {
      // We need to wait until a whole gait cycle has completed, and
      // all legs are on the ground before stopping.

      if (parent_->m_.gait_driver->gait()->zero_phase_count() >= 2 &&
          parent_->m_.gait_driver->gait()->are_all_legs_stance()) {
        data_.active = false;
      }
    }
    gait->lift_height_percent = data_.active ? 100.0 : 0.0;
  }

  void DoManual() {
    const bool want_active =
        manual_gait_.translate_x_mm_s != 0 ||
        manual_gait_.translate_y_mm_s != 0 ||
        manual_gait_.rotate_deg_s != 0;
    UpdateActive(&manual_gait_, want_active);
    parent_->m_.gait_driver->SetCommand(manual_gait_);
  }

  void DoDrive() {
    const auto& p = parent_->parameters_;

    TurretCommand turret;
    Command gait;

    turret.fire_control = data_.current_drive.fire_control;
    if (data_.current_drive.turret_target_relative) {
      TurretCommand::TargetRelative relative;
      relative.x = data_.current_drive.turret_target_relative->x();
      relative.y = data_.current_drive.turret_target_relative->y();
      turret.target_relative = relative;
    } else if (data_.current_drive.turret_rate_dps) {
      turret.rate = TurretCommand::Rate();
      turret.rate->x_deg_s = data_.current_drive.turret_rate_dps->yaw;
      turret.rate->y_deg_s = data_.current_drive.turret_rate_dps->pitch;
    }

    const auto body_offset_mm =
        data_.current_drive.body_offset_mm + p.body_offset_mm;
    gait.body_x_mm = body_offset_mm.x();
    gait.body_y_mm = body_offset_mm.y();
    gait.body_z_mm = body_offset_mm.z();

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

    const bool want_active = (data_.current_drive.drive_mm_s != base::Point3D());
    UpdateActive(&gait, want_active);

    // NOTE: We explicitly use "want_active" here, because we don't
    // want to be messing around with rotations if we are trying to
    // stop.
    if (want_active) {
      const double heading_deg = base::Point3DHeadingDeg(body_mm_s);
      const double forward_error_deg = base::WrapNeg180To180(heading_deg);
      const double drive_heading_deg =
          base::Point3DHeadingDeg(data_.current_drive.drive_mm_s);
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

      if (body_mm_s.norm() > max_translate_mm_s) {
        body_mm_s = body_mm_s * (max_translate_mm_s / body_mm_s.norm());
      }
    }

    // HACK: Don't allow side stepping unless we are frozen.  It just
    // causes the current robot to fall over.
    gait.translate_x_mm_s = data_.current_drive.freeze_rotation ? body_mm_s.x() : 0;
    gait.translate_y_mm_s = body_mm_s.y();

    // Give the commands to our members.
    parent_->m_.turret->SetCommand(turret);
    parent_->m_.gait_driver->SetCommand(gait);
  }

  struct Message {
    std::optional<DriveCommand> drive;
    std::optional<Command> gait;
    std::optional<TurretCommand> turret;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(drive));
      a->Visit(MJ_NVP(gait));
      a->Visit(MJ_NVP(turret));
    }
  };

  void HandleRead(mjlib::base::error_code ec, std::size_t size) {
    mjlib::base::FailIf(ec);

    std::string data(receive_buffer_, size);
    StartRead();

    const auto message = mjlib::base::Json5ReadArchive::Read<Message>(data);

    data_.last_command_timestamp = base::Now(service_);

    if (message.drive) {
      HandleMessageDrive(*message.drive);
      return;
    }
    if (message.gait) {
      HandleMessageGait(*message.gait);
    }
    if (message.turret) {
      HandleMessageTurret(*message.turret);
    }
  }

  void StartTurretBias() {
    data_.turret_bias_start_timestamp = base::Now(service_);
    parent_->m_.turret->StartBias();
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
        // Wait until the turret bias process is finished.

        const double elapsed_s = base::ConvertDurationToSeconds(
            now - data_.turret_bias_start_timestamp);
        if (elapsed_s > parent_->parameters_.turret_bias_timeout_s) {
          parent_->m_.gait_driver->CommandPrepositioning();
          data_.prepositioning_start_timestamp = now;
          data_.mode = Data::Mode::kPrepositioning;
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
      case Data::Mode::kFault: {
        break;
      }
    }
  }

  void HandleMessageGait(const Command& command) {
    MaybeEnterActiveMode(Data::Mode::kManual);
    manual_gait_ = command;

    if (data_.mode == Data::Mode::kManual) {
      parent_->m_.gait_driver->SetCommand(manual_gait_);
    }
  }

  void HandleMessageTurret(const TurretCommand& command) {
    MaybeEnterActiveMode(Data::Mode::kManual);

    if (data_.mode == Data::Mode::kManual) {
      parent_->m_.turret->SetCommand(command);
    }
  }

  void HandleMessageDrive(const DriveCommand& command) {
    data_.current_drive = command;
    MaybeEnterActiveMode(Data::Mode::kDrive);

    if (data_.mode == Data::Mode::kDrive) {
      // Immediately update ourselves.
      DoDrive();
    }
  }


  MechWarfare* const parent_;
  base::Context& context_;
  boost::asio::io_context& service_;
  boost::program_options::options_description options_;

  base::LogRef log_ = base::GetLogInstance("MechWarfare");

  boost::asio::ip::udp::socket server_;
  char receive_buffer_[3000] = {};
  boost::asio::ip::udp::endpoint receive_endpoint_;

  mjlib::io::RepeatingTimer timer_;

  std::shared_ptr<McastTelemetryInterface> telemetry_;

  double servo_min_voltage_V_ = 0.0;
  double servo_max_voltage_V_ = 0.0;
  double servo_min_temp_C_ = 0.0;
  double servo_max_temp_C_ = 0.0;

  bool outstanding_ = false;
  Data data_;
  Command manual_gait_;
  std::vector<ServoInterface::Joint> joint_command_;
  JointData joint_data_;
  mjlib::multiplex::ThreadedClient* multiplex_client_ = nullptr;

  boost::signals2::signal<void (const GaitDriver::GaitData*)> gait_data_signal_;
  boost::signals2::signal<void (const JointData*)> joint_data_signal_;
  boost::signals2::signal<void (const GaitDriver::CommandState*)> command_state_signal_;
  boost::signals2::signal<void (const Data*)> data_signal_;
};

MechWarfare::MechWarfare(base::Context& context)
    : service_(context.service),
      impl_(new Impl(this, context)) {
  m_.multiplex_client = std::make_unique<MultiplexClient>(service_);
  m_.servo_base.reset(new Mech::ServoBase(service_, *context.factory));
  m_.servo.reset(new Mech::Servo(m_.servo_base.get()));
  m_.moteus_servo = std::make_unique<MoteusServo>(service_, context.telemetry_registry.get());

  m_.servo_selector = std::make_unique<ServoSelector>();
  m_.servo_selector->AddInterface("herkulex", m_.servo.get());
  m_.servo_selector->AddInterface("moteus", m_.moteus_servo.get());

  m_.imu.reset(new MjmechImuDriver(context));
  m_.ahrs.reset(new Ahrs(context, m_.imu->imu_data_signal()));
  m_.gait_driver.reset(new GaitDriver(service_,
                                      context.telemetry_registry.get(),
                                      m_.ahrs->ahrs_data_signal()));
  m_.turret.reset(new Turret(context, m_.servo_base.get()));

  m_.multiplex_client->RequestClient([this](const auto& ec, auto* client) {
        mjlib::base::FailIf(ec);
        m_.moteus_servo->SetClient(client);
        m_.turret->SetMultiplexClient(client);
        this->impl_->multiplex_client_ = client;
      });

  m_.video.reset(new VideoSenderApp(context));

  impl_->telemetry_ = m_.video->telemetry_interface().lock();

  m_.video->target_tracker()->data_signal()->connect([this](const TargetTrackerData* data) {
      std::optional<base::Point3D> point3d;
      if (data->target) {
        point3d = base::Point3D(data->target->center.x(),
                                data->target->center.y(),
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
