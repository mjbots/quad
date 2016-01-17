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

#include "gait_driver.h"

#include "base/deadline_timer.h"
#include "base/fail.h"
#include "base/now.h"

#include "ripple.h"
#include "servo_interface.h"

namespace mjmech {
namespace mech {
namespace {
class FailHandler {
 public:
  void operator()(base::ErrorCode ec) const {
    FailIf(ec);
  }
};
}

class GaitDriver::Impl : boost::noncopyable {
 public:
  Impl(GaitDriver* parent,
       boost::asio::io_service& service,
       ServoInterface* servo)
      : parent_(parent),
        service_(service),
        servo_(servo),
        timer_(service) {}

  void SetGait(std::unique_ptr<RippleGait> gait) {
    gait_ = std::move(gait);
  }

  void SetCommand(const Command& command) {
    CommandData data;
    data.timestamp = base::Now(service_);
    data.command = command;

    last_command_timestamp_ = data.timestamp;

    parent_->command_data_signal_(&data);

    if (state_ != kActive) {
      servo_->EnablePower(ServoInterface::kPowerEnable, {}, FailHandler());
    }
    state_ = kActive;

    // If our timer isn't started yet, then start it now.
    if (!timer_started_) {
      timer_started_ = true;
      timer_.expires_from_now(base::ConvertSecondsToDuration(0.0));
      StartTimer();
    }

    gait_->SetCommand(command);
  }

  void SetFree() {
    servo_->EnablePower(ServoInterface::kPowerBrake, {}, FailHandler());
    state_ = kUnpowered;
  }

  void ProcessBodyAhrs(boost::posix_time::ptime timestamp,
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

 private:
  void StartTimer() {
    timer_.expires_at(
        timer_.expires_at() +
        base::ConvertSecondsToDuration(parent_->parameters_.period_s));
    timer_.async_wait(std::bind(&Impl::HandleTimer, this,
                                std::placeholders::_1));
  }

  void HandleTimer(const boost::system::error_code& ec) {
    if (ec == boost::asio::error::operation_aborted) { return; }

    auto now = base::Now(service_);
    auto elapsed = now - last_command_timestamp_;
    if (elapsed > base::ConvertSecondsToDuration(
            parent_->parameters_.command_timeout_s)) {
      timer_started_ = false;
      SetFree();
      return;
    }

    StartTimer();

    if (elapsed > (base::ConvertSecondsToDuration(
                       parent_->parameters_.command_timeout_s -
                       parent_->parameters_.idle_time_s))) {
      Command idle_command;
      idle_command.lift_height_percent = 0.0;
      gait_->SetCommand(idle_command);
    }

    // Advance our gait, then send the requisite servo commands out.
    auto gait_commands = gait_->AdvanceTime(parent_->parameters_.period_s);

    std::vector<ServoInterface::Joint> servo_commands;
    for (const auto& joint: gait_commands.joints) {
      servo_commands.emplace_back(
          ServoInterface::Joint{joint.servo_number, joint.angle_deg});
    }

    servo_->SetPose(servo_commands, FailHandler());

    const auto& state = gait_->state();

    GaitData data;
    data.timestamp = base::Now(service_);

    data.state = state_;
    data.command = gait_commands;
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
    parent_->gait_data_signal_(&data);
  }

  GaitDriver* const parent_;
  boost::asio::io_service& service_;
  std::unique_ptr<RippleGait> gait_;
  ServoInterface* const servo_;

  base::DeadlineTimer timer_;
  bool timer_started_ = false;
  State state_ = kUnpowered;
  boost::posix_time::ptime last_command_timestamp_;
  base::Quaternion attitude_;
  base::Point3D body_rate_dps_;
};

GaitDriver::GaitDriver(boost::asio::io_service& service,
                       ServoInterface* servo)
    : impl_(new Impl(this, service, servo)) {}

GaitDriver::~GaitDriver() {}

void GaitDriver::SetGait(std::unique_ptr<RippleGait> gait) {
  impl_->SetGait(std::move(gait));
}

void GaitDriver::SetCommand(const Command& command) {
  impl_->SetCommand(command);
}

void GaitDriver::SetFree() {
  impl_->SetFree();
}

void GaitDriver::ProcessBodyAhrs(boost::posix_time::ptime timestamp,
                                 bool valid,
                                 const base::Quaternion& attitude,
                                 const base::Point3D& body_rate_dps) {
  impl_->ProcessBodyAhrs(timestamp, valid, attitude, body_rate_dps);
}

}
}
