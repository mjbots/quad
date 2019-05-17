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

#include "gait_driver.h"

#include "mjlib/base/fail.h"
#include "mjlib/base/program_options_archive.h"
#include "mjlib/io/deadline_timer.h"

#include "base/logging.h"
#include "base/now.h"
#include "base/telemetry_registry.h"

#include "ripple.h"
#include "servo_interface.h"

namespace mjmech {
namespace mech {
namespace {
struct Parameters {
  /// Update the gait engine (and send servo commands) at this rate.
  double period_s = 0.05; // 20Hz

  /// This long with no commands will result in stopping the gait
  /// engine and setting all servos to unpowered.
  double command_timeout_s = 0.0;

  /// The maximum amount that the gait engine can accelerate or
  /// decelerate in each axis.
  base::Point3D max_acceleration_mm_s2 = base::Point3D(50., 50., 50.);

  double preposition_speed_dps = 60.0;
  double standup_speed_dps = 30.0;
  double sitting_speed_dps = 30.0;
  double stand_sit_time_s = 1.5;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(period_s));
    a->Visit(MJ_NVP(command_timeout_s));
    a->Visit(MJ_NVP(max_acceleration_mm_s2));
    a->Visit(MJ_NVP(preposition_speed_dps));
    a->Visit(MJ_NVP(standup_speed_dps));
    a->Visit(MJ_NVP(sitting_speed_dps));
    a->Visit(MJ_NVP(stand_sit_time_s));
  }
};

enum State : int {
  kUnpowered,
  kActive,
  kPrepositioning,
  kStandup,
  kSitting,
};

static std::map<State, const char*> StateMapper() {
  return std::map<State, const char*>{
    { kUnpowered, "kUnpowered" },
    { kActive, "kActive" },
    { kPrepositioning, "kPrepositioning" },
    { kStandup, "kStandup" },
    { kSitting, "kSitting" },
};
}

struct GaitData {
  boost::posix_time::ptime timestamp;

  State state;
  base::Transform body_robot;
  base::Transform cog_robot;
  base::Transform body_world;
  base::Transform robot_world;

  base::Quaternion attitude;
  base::Point3D body_rate_dps;

  std::array<base::Point3D, 4> legs;
  // The command as sent by the user.
  JointCommand command;

  // The command as given to the gait engine.
  Command input_command;
  Command gait_command;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(timestamp));
    a->Visit(MJ_ENUM(state, StateMapper));
    a->Visit(MJ_NVP(body_robot));
    a->Visit(MJ_NVP(cog_robot));
    a->Visit(MJ_NVP(body_world));
    a->Visit(MJ_NVP(robot_world));
    a->Visit(MJ_NVP(attitude));
    a->Visit(MJ_NVP(body_rate_dps));
    a->Visit(MJ_NVP(legs));
    a->Visit(MJ_NVP(command));
    a->Visit(MJ_NVP(input_command));
    a->Visit(MJ_NVP(gait_command));
  }
};

struct CommandData {
  boost::posix_time::ptime timestamp;

  Command command;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(timestamp));
    a->Visit(MJ_NVP(command));
  }
};

class FailHandler {
 public:
  void operator()(mjlib::base::error_code ec) const {
    mjlib::base::FailIf(ec);
  }
};
}

class GaitDriver::Impl : boost::noncopyable {
 public:
  Impl(boost::asio::io_service& service,
       base::TelemetryRegistry* telemetry_registry,
       ServoInterface* servo)
      : service_(service),
        servo_(servo),
        timer_(service) {
    mjlib::base::ProgramOptionsArchive(&options_).Accept(&parameters_);

    telemetry_registry->Register("gait", &gait_data_signal_);
    telemetry_registry->Register("gait_command", &command_data_signal_);
  }

  void AsyncStart(mjlib::io::ErrorCallback callback) {
    log_.warn("AsyncStart");
    timer_.expires_from_now(base::ConvertSecondsToDuration(0.0));
    StartTimer();
    service_.post(std::bind(callback, mjlib::base::error_code()));
  }

  void SetGait(std::unique_ptr<RippleGait> gait) {
    gait_ = std::move(gait);
  }

  void SetCommand(const Command& input_command) {
    input_command_ = input_command;

    const double old_translate_x_mm_s = gait_command_.translate_x_mm_s;
    const double old_translate_y_mm_s = gait_command_.translate_y_mm_s;
    gait_command_ = input_command;
    gait_command_.translate_x_mm_s = old_translate_x_mm_s;
    gait_command_.translate_y_mm_s = old_translate_y_mm_s;

    CommandData data;
    data.timestamp = base::Now(service_);
    data.command = input_command_;
    command_data_signal_(&data);

    last_command_timestamp_ = data.timestamp;

    if (state_ != kActive) {
      // This may already be the case, but it doesn't hurt to do it
      // again.
      servo_->EnablePower(ServoInterface::kPowerEnable, {}, FailHandler());
    }
    state_ = kActive;

    gait_->SetCommand(gait_command_);
  }

  void SetFree() {
    servo_->EnablePower(ServoInterface::kPowerBrake, {}, FailHandler());
    state_ = kUnpowered;

    Emit();
  }

  std::vector<ServoInterface::Joint> DerateTibiaCommands(
      const std::vector<ServoInterface::Joint>& input,
      double scale) {
    std::vector<ServoInterface::Joint> result;
    // TOTAL HACK!  I need to do prepositioning in IK space, this is
    // just an expedient way to make it not barf.
    for (auto item : input) {
      if (item.address == 2 || item.address == 5 ||
          item.address == 8 || item.address == 11) {
        item.power = scale;
      }
      // We want no feedforward in this phase, since we aren't
      // touching the ground.
      item.torque_Nm = 0.0;
      result.push_back(item);
    }
    return result;
  }

  std::vector<ServoInterface::Joint> DerateStandupCommands(
      const std::vector<ServoInterface::Joint>& input,
      double scale) {
    // For the very beginning of the standup, (and the end of the
    // sitting down), we want to have 0 torque, with a quick ramp up
    // to full torque shortly after that.
    const double torque_scale =
        (scale < 0.05) ?
        0.0 :
        (scale < 0.2) ?
        ((scale - 0.05) / 0.15) :
        1.0;

    std::vector<ServoInterface::Joint> result;
    for (const auto& item : input) {
      auto copy = item;
      copy.torque_Nm = torque_scale * item.torque_Nm;
      result.push_back(copy);
    }
    return result;
  }

  std::vector<ServoInterface::Joint> MakeServoCommands(
      const JointCommand& gait_joints,
      double joint_speed_dps) {
    std::vector<ServoInterface::Joint> servo_commands;

    // Switch all of these to be a goal position with a non-zero
    // velocity so that we gently go there from wherever we might
    // start at (assuming the servos support that).
    for (auto& command : gait_joints.joints) {
      ServoInterface::Joint joint_command;
      joint_command.address = command.servo_number;
      joint_command.goal_deg = command.angle_deg;
      joint_command.torque_Nm = command.torque_Nm;
      joint_command.angle_deg = std::numeric_limits<double>::quiet_NaN();
      joint_command.velocity_dps = joint_speed_dps;
      joint_command.kp = command.kp;

      servo_commands.push_back(joint_command);

      // TODO: Set a lower than maximum power?
    }

    return servo_commands;
  }

  void CommandPrepositioning() {
    log_.debug("CommandPrepositioning");
    stand_sit_start_time_ = base::Now(service_);
    state_ = kPrepositioning;

    SendPrepositionCommand();

    Emit();
  }

  void SendPrepositionCommand() {
    gait_commands_ = gait_->MakeJointCommand(
        gait_->GetPrepositioningState(1.0));
    const auto servo_commands =
        MakeServoCommands(gait_commands_, parameters_.preposition_speed_dps);
    servo_->SetPose(DerateTibiaCommands(servo_commands, GetSitStandRatio()),
                    FailHandler());

    Emit();
  }

  void CommandStandup() {
    log_.debug("CommandStandup");
    stand_sit_start_time_ = base::Now(service_);
    state_ = kStandup;

    SendStandupCommand();

    Emit();
  }

  void SendStandupCommand() {
    gait_commands_ = gait_->MakeJointCommand(
        gait_->GetPrepositioningState(1.0 - GetSitStandRatio()));
    servo_->SetPose(
        DerateStandupCommands(
            MakeRegularServoCommands(gait_commands_.joints),
            GetSitStandRatio()),
        FailHandler());

    Emit();
  }

  void CommandSitting() {
    log_.debug("CommandSitting");
    stand_sit_start_time_ = base::Now(service_);
    state_ = kSitting;

    SendSittingCommand();

    Emit();
  }

  double GetSitStandRatio() {
    const auto now = base::Now(service_);
    const double delay_s = base::ConvertDurationToSeconds(now - stand_sit_start_time_);
    const double ratio = std::max(0.0, std::min(1.0, delay_s / parameters_.stand_sit_time_s));
    return ratio;
  }

  void SendSittingCommand() {
    gait_commands_ = gait_->MakeJointCommand(
        gait_->GetPrepositioningState(GetSitStandRatio()));
    servo_->SetPose(
        DerateStandupCommands(
            MakeRegularServoCommands(gait_commands_.joints),
            1.0 - GetSitStandRatio()),
        FailHandler());
    Emit();
  }

  void ProcessBodyAhrs(boost::posix_time::ptime,
                       bool valid,
                       const base::Quaternion& world_attitude,
                       const base::Point3D& body_rate_dps) {
    if (!valid) {
      attitude_ = base::Quaternion();
      body_rate_dps_ = base::Point3D();
      return;
    }

    attitude_ = world_attitude;
    body_rate_dps_ = body_rate_dps;
  }

  boost::program_options::options_description* options() {
    return &options_;
  }

  const Gait* gait() const {
    return gait_.get();
  }

  const Command& input_command() const {
    return input_command_;
  }

  const Command& gait_command() const {
    return gait_command_;
  }

 private:
  void StartTimer() {
    timer_.expires_at(
        timer_.expires_at() +
        base::ConvertSecondsToDuration(parameters_.period_s));
    timer_.async_wait(std::bind(&Impl::HandleTimer, this,
                                std::placeholders::_1));
  }

  void UpdateAxisAccel(double input_mm_s,
                       double* gait_mm_s,
                       double accel_mm_s2) {
    double delta_mm_s = input_mm_s - *gait_mm_s;
    const double max_step_mm_s = parameters_.period_s * accel_mm_s2;
    const double step_mm_s = [&]() {
      if (delta_mm_s > max_step_mm_s) { return max_step_mm_s; }
      if (delta_mm_s < -max_step_mm_s) { return -max_step_mm_s; }
      return delta_mm_s;
    }();

    *gait_mm_s += step_mm_s;
  }

  ServoInterface::Joint MakeRegularServoCommand(const JointCommand::Joint& joint) {
    ServoInterface::Joint result;
    result.address = joint.servo_number;
    result.angle_deg = joint.angle_deg;
    result.torque_Nm = joint.torque_Nm;
    result.kp = joint.kp;
    return result;
  }

  std::vector<ServoInterface::Joint> MakeRegularServoCommands(
      const std::vector<JointCommand::Joint>& joints) {
    std::vector<ServoInterface::Joint> result;
    for (const auto& joint : joints) {
      result.push_back(MakeRegularServoCommand(joint));
    }
    return result;
  }

  void HandleTimer(const boost::system::error_code& ec) {
    if (ec == boost::asio::error::operation_aborted) { return; }
    StartTimer();

    switch (state_) {
      case kUnpowered: {
        SetFree();
        return;
      }
      case kPrepositioning: {
        SendPrepositionCommand();
        return;
      }
      case kSitting: {
        SendSittingCommand();
        return;
      }
      case kStandup: {
        SendStandupCommand();
        return;
      }
      case kActive: {
        break;
      }
    }


    const auto now = base::Now(service_);
    const auto elapsed = now - last_command_timestamp_;
    const bool timeout =
        (parameters_.command_timeout_s > 0.0) &&
        (elapsed > base::ConvertSecondsToDuration(
            parameters_.command_timeout_s));
    if (timeout) {
      SetFree();
      return;
    }

    UpdateAxisAccel(input_command_.translate_x_mm_s,
                    &gait_command_.translate_x_mm_s,
                    parameters_.max_acceleration_mm_s2.x);
    UpdateAxisAccel(input_command_.translate_y_mm_s,
                    &gait_command_.translate_y_mm_s,
                    parameters_.max_acceleration_mm_s2.y);

    // Advance our gait, then send the requisite servo commands out.
    gait_commands_ = gait_->AdvanceTime(parameters_.period_s);

    servo_->SetPose(MakeRegularServoCommands(gait_commands_.joints), FailHandler());

    Emit();
  }

  void Emit() {
    const auto& state = gait_->state();

    GaitData data;
    data.timestamp = base::Now(service_);

    data.state = state_;
    data.command = gait_commands_;
    std::sort(data.command.joints.begin(), data.command.joints.end(),
              [](const auto& lhs_joint, const auto& rhs_joint) {
                return lhs_joint.servo_number < rhs_joint.servo_number;
              });
    data.body_robot = state.body_frame.TransformToFrame(&state.robot_frame);
    data.cog_robot = state.cog_frame.TransformToFrame(&state.robot_frame);
    data.body_world = state.body_frame.TransformToFrame(&state.world_frame);
    data.robot_world = state.robot_frame.TransformToFrame(&state.world_frame);
    data.attitude = attitude_;
    data.body_rate_dps = body_rate_dps_;
    BOOST_ASSERT(state.legs.size() == 4);
    for (size_t i = 0; i < state.legs.size(); i++) {
      data.legs[i] = state.robot_frame.MapFromFrame(
          state.legs[i].frame, state.legs[i].point);
    }
    data.input_command = input_command_;
    data.gait_command = gait_command_;
    gait_data_signal_(&data);
  }

  base::LogRef log_ = base::GetLogInstance("GaitDriver");

  boost::asio::io_service& service_;
  std::unique_ptr<RippleGait> gait_;
  ServoInterface* const servo_;

  boost::program_options::options_description options_;

  Parameters parameters_;

  boost::signals2::signal<void (const GaitData*)> gait_data_signal_;
  boost::signals2::signal<void (const CommandData*)> command_data_signal_;

  mjlib::io::DeadlineTimer timer_;
  State state_ = kUnpowered;
  boost::posix_time::ptime last_command_timestamp_;
  base::Quaternion attitude_;
  base::Point3D body_rate_dps_;
  JointCommand gait_commands_;

  Command input_command_;
  Command gait_command_;

  boost::posix_time::ptime stand_sit_start_time_;
};

GaitDriver::GaitDriver(boost::asio::io_service& service,
                       base::TelemetryRegistry* registry,
                       ServoInterface* servo)
    : impl_(new Impl(service, registry, servo)) {}

GaitDriver::~GaitDriver() {}

void GaitDriver::AsyncStart(mjlib::io::ErrorCallback callback) {
  impl_->AsyncStart(callback);
}

void GaitDriver::SetGait(std::unique_ptr<RippleGait> gait) {
  impl_->SetGait(std::move(gait));
}

void GaitDriver::SetCommand(const Command& command) {
  impl_->SetCommand(command);
}

void GaitDriver::SetFree() {
  impl_->SetFree();
}

void GaitDriver::CommandPrepositioning() {
  impl_->CommandPrepositioning();
}

void GaitDriver::CommandStandup() {
  impl_->CommandStandup();
}

void GaitDriver::CommandSitting() {
  impl_->CommandSitting();
}

boost::program_options::options_description*
GaitDriver::options() { return impl_->options(); }

const Gait* GaitDriver::gait() const {
  return impl_->gait();
}

const Command& GaitDriver::input_command() const {
  return impl_->input_command();
}

const Command& GaitDriver::gait_command() const {
  return impl_->gait_command();
}

void GaitDriver::ProcessBodyAhrs(boost::posix_time::ptime timestamp,
                                 bool valid,
                                 const base::Quaternion& attitude,
                                 const base::Point3D& body_rate_dps) {
  impl_->ProcessBodyAhrs(timestamp, valid, attitude, body_rate_dps);
}

}
}
