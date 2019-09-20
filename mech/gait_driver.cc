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
#include "mjlib/base/limit.h"
#include "mjlib/base/program_options_archive.h"
#include "mjlib/io/deadline_timer.h"
#include "mjlib/io/now.h"

#include "base/logging.h"
#include "base/telemetry_registry.h"

#include "ripple.h"
#include "servo_interface.h"

namespace mjmech {
namespace mech {
namespace {
struct Parameters {
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

  double joint_speed_scale = 0.1;
  double joint_max_speed_dps = 1000.0;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(command_timeout_s));
    a->Visit(MJ_NVP(max_acceleration_mm_s2));
    a->Visit(MJ_NVP(preposition_speed_dps));
    a->Visit(MJ_NVP(standup_speed_dps));
    a->Visit(MJ_NVP(sitting_speed_dps));
    a->Visit(MJ_NVP(stand_sit_time_s));
    a->Visit(MJ_NVP(joint_speed_scale));
    a->Visit(MJ_NVP(joint_max_speed_dps));
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

}

class FailHandler {
 public:
  void operator()(mjlib::base::error_code ec) const {
    mjlib::base::FailIf(ec);
  }
};

class GaitDriver::Impl : boost::noncopyable {
 public:
  Impl(const boost::asio::executor& executor,
       base::TelemetryRegistry* telemetry_registry)
      : executor_(executor) {
    mjlib::base::ProgramOptionsArchive(&options_).Accept(&parameters_);
    telemetry_registry->Register("gait_command", &command_data_signal_);
  }

  void AsyncStart(mjlib::io::ErrorCallback callback) {
    log_.warn("AsyncStart");
    boost::asio::post(
        executor_,
        std::bind(callback, mjlib::base::error_code()));
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
    data.timestamp = Now();
    data.command = input_command_;
    command_data_signal_(&data);

    last_command_timestamp_ = data.timestamp;

    state_ = kActive;

    gait_->SetCommand(gait_command_);
  }

  CommandState SetFree() {
    state_ = kUnpowered;

    CommandState result;
    result.timestamp = Now();
    result.power_state = ServoInterface::kPowerFree;

    for (int i = 1; i <= 12; i++) {
      ServoInterface::Joint joint;
      joint.address = i;
      result.joints.push_back(joint);
    }
    return result;
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
        item.max_torque_scale = scale;
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
    stand_sit_start_time_ = Now();
    state_ = kPrepositioning;
  }

  CommandState MakeCommandState(std::vector<ServoInterface::Joint> joints) {
    CommandState result;
    result.timestamp = Now();
    result.joints = std::move(joints);
    result.power_state = ServoInterface::kPowerEnable;
    return result;
  }

  CommandState SendPrepositionCommand() {
    auto state = gait_->GetPrepositioningState(0.0);
    gait_->SetState(state);
    gait_commands_ = gait_->MakeJointCommand(state);
    const auto servo_commands =
        MakeServoCommands(gait_commands_, parameters_.preposition_speed_dps);
    return MakeCommandState(
        DerateTibiaCommands(servo_commands, GetSitStandRatio()));
  }

  void CommandStandup() {
    log_.debug("CommandStandup");
    MJ_ASSERT(state_ == kPrepositioning);
    stand_sit_start_time_ = Now();
    state_ = kStandup;
    gait_->SetState(gait_->GetPrepositioningState(0.0));
  }

  CommandState SendStandupCommand() {
    auto state = gait_->GetPrepositioningState(GetSitStandRatio());
    gait_->SetState(state);
    gait_commands_ = gait_->MakeJointCommand(state);
    return
        MakeCommandState(
            DerateStandupCommands(
                MakeRegularServoCommands(gait_commands_.joints),
                GetSitStandRatio()));
  }

  void CommandPrepareToSit() {
    log_.debug("CommandPrepareToSit");
    stand_sit_start_time_ = Now();
    state_ = kPreparingToSit;

    auto desired_sitting_state = gait_->GetPrepositioningState(1.0);
    Command command = gait_command_;

    // We need to sit now, so just instantaneously stop our velocity.
    command.translate_x_mm_s = 0.0;
    command.translate_y_mm_s = 0.0;
    command.rotate_deg_s = 0.0;
    command.lift_height_percent = 100.0;

    for (size_t i = 0; i < desired_sitting_state.legs.size(); i++) {
      const auto& leg = desired_sitting_state.legs[i];

      Command::RobotFrameLeg cmd_leg;
      cmd_leg.leg_num = i;
      cmd_leg.point = desired_sitting_state.robot_frame.MapFromFrame(
          leg.frame, leg.point);
      command.override_foot_placement.push_back(cmd_leg);
    }

    CommandData command_data;
    command_data.command = command;
    command_data.timestamp = Now();
    command_data_signal_(&command_data);

    gait_->SetCommand(command);
    gait_->RezeroPhaseCount();
  }

  void CommandSitDown() {
    log_.debug("CommandSitDown");
    MJ_ASSERT(state_ == kPreparingToSit);
    stand_sit_start_time_ = Now();
    state_ = kSitting;
  }

  double GetSitStandRatio() {
    const auto now = Now();
    const double delay_s = base::ConvertDurationToSeconds(now - stand_sit_start_time_);
    const double ratio = std::max(0.0, std::min(1.0, delay_s / parameters_.stand_sit_time_s));
    return ratio;
  }

  CommandState SendSittingCommand() {
    auto state = gait_->GetPrepositioningState(1.0 - GetSitStandRatio());
    gait_->SetState(state);
    gait_commands_ = gait_->MakeJointCommand(state);;
    return MakeCommandState(
        DerateStandupCommands(
            MakeRegularServoCommands(gait_commands_.joints),
            1.0 - GetSitStandRatio()));
  }

  UpdateResult Update(double period_s) {
    switch (state_) {
      case kUnpowered: {
        return PackResult(SetFree());
      }
      case kPrepositioning: {
        return PackResult(SendPrepositionCommand());
      }
      case kSitting: {
        return PackResult(SendSittingCommand());
      }
      case kStandup: {
        return PackResult(SendStandupCommand());
      }
      case kPreparingToSit:
      case kActive: {
        break;
      }
    }


    if (state_ != kPreparingToSit) {
      const auto now = Now();
      const auto elapsed = now - last_command_timestamp_;
      const bool timeout =
          (parameters_.command_timeout_s > 0.0) &&
          (elapsed > base::ConvertSecondsToDuration(
              parameters_.command_timeout_s));
      if (timeout) {
        return PackResult(SetFree());
      }

      UpdateAxisAccel(input_command_.translate_x_mm_s,
                      &gait_command_.translate_x_mm_s,
                      parameters_.max_acceleration_mm_s2.x(),
                      period_s);
      UpdateAxisAccel(input_command_.translate_y_mm_s,
                      &gait_command_.translate_y_mm_s,
                      parameters_.max_acceleration_mm_s2.y(),
                      period_s);
    }

    // Advance our gait, then send the requisite servo commands out.
    gait_commands_ = gait_->AdvanceTime(period_s);

    return PackResult(MakeCommandState(MakeRegularServoCommands(gait_commands_.joints)));
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
  void UpdateAxisAccel(double input_mm_s,
                       double* gait_mm_s,
                       double accel_mm_s2,
                       double period_s) {
    double delta_mm_s = input_mm_s - *gait_mm_s;
    const double max_step_mm_s = period_s * accel_mm_s2;
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
    result.velocity_dps = mjlib::base::Limit(
        parameters_.joint_speed_scale * joint.velocity_deg_s,
        -parameters_.joint_max_speed_dps,
        parameters_.joint_max_speed_dps);
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

  UpdateResult PackResult(CommandState command_state) {
    UpdateResult result;
    result.command_state = std::move(command_state);

    const auto& state = gait_->state();

    auto& data = result.gait_data;

    data.timestamp = Now();

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
    BOOST_ASSERT(state.legs.size() == 4);
    for (size_t i = 0; i < state.legs.size(); i++) {
      data.legs[i] = state.robot_frame.MapFromFrame(
          state.legs[i].frame, state.legs[i].point);
    }
    data.input_command = input_command_;
    data.gait_command = gait_command_;

    return result;
  }

  boost::posix_time::ptime Now() {
    return mjlib::io::Now(executor_.context());
  }

  base::LogRef log_ = base::GetLogInstance("GaitDriver");

  boost::asio::executor executor_;
  std::unique_ptr<RippleGait> gait_;

  boost::program_options::options_description options_;

  Parameters parameters_;

  boost::signals2::signal<void (const CommandData*)> command_data_signal_;

  State state_ = kUnpowered;
  boost::posix_time::ptime last_command_timestamp_;
  JointCommand gait_commands_;

  Command input_command_;
  Command gait_command_;

  boost::posix_time::ptime stand_sit_start_time_;
};

GaitDriver::GaitDriver(const boost::asio::executor& executor,
                       base::TelemetryRegistry* registry)
    : impl_(new Impl(executor, registry)) {}

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

void GaitDriver::CommandPrepareToSit() {
  impl_->CommandPrepareToSit();
}

void GaitDriver::CommandSitDown() {
  impl_->CommandSitDown();
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

GaitDriver::UpdateResult GaitDriver::Update(double period_s) {
  return impl_->Update(period_s);
}

}
}
