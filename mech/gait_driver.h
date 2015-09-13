// Copyright 2014-2015 Josh Pieper, jjp@pobox.com.  All rights reserved.
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

#pragma once

#include <boost/asio/io_service.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <boost/signals2/signal.hpp>

#include "base/tf.h"
#include "base/visitor.h"

#include "gait.h"

namespace legtool {
class RippleGait;
class ServoInterface;

/// Advance a gait through time, accept commands, and send those gait
/// commands out to some servos.
class GaitDriver : boost::noncopyable {
 public:
  template <typename TelemetryRegistry>
  GaitDriver(boost::asio::io_service& service,
             TelemetryRegistry* telemetry_registry,
             ServoInterface* servo)
      : GaitDriver(service, servo) {
    telemetry_registry->Register("gait", &gait_data_signal_);
    telemetry_registry->Register("gait_command", &command_data_signal_);
  }

  ~GaitDriver();

  void SetGait(std::unique_ptr<RippleGait>);

  /// Start the gait engine if it is not running and make it execute
  /// the given command.
  void SetCommand(const Command& command);

  /// Set all servos to be unpowered and stop executing the gait
  /// engine.
  void SetFree();

  struct Parameters {
    /// Update the gait engine (and send servo commands) at this rate.
    double period_s = 0.05; // 20Hz

    /// This long with no commands will result in stopping the gait
    /// engine and setting all servos to unpowered.
    double command_timeout_s = 15.0;

    /// Before entering brake mode, spend this long getting into the
    /// idle pose.
    double idle_time_s = 1.0;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(LT_NVP(period_s));
      a->Visit(LT_NVP(command_timeout_s));
      a->Visit(LT_NVP(idle_time_s));
    }
  };

  Parameters* parameters() { return &parameters_; }

 private:
  GaitDriver(boost::asio::io_service& service,
             ServoInterface* servo);

  enum State {
    kUnpowered,
    kActive,
  };

  struct GaitData {
    boost::posix_time::ptime timestamp;

    State state;
    Transform body_robot;
    Transform cog_robot;
    Transform body_world;
    Transform robot_world;

    std::array<Point3D, 4> legs;
    JointCommand command;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(LT_NVP(timestamp));
      //a->Visit(LT_NVP(state));
      a->Visit(LT_NVP(body_robot));
      a->Visit(LT_NVP(cog_robot));
      a->Visit(LT_NVP(body_world));
      a->Visit(LT_NVP(robot_world));
      a->Visit(LT_NVP(legs));
      a->Visit(LT_NVP(command));
    }
  };

  struct CommandData {
    boost::posix_time::ptime timestamp;

    Command command;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(LT_NVP(timestamp));
      a->Visit(LT_NVP(command));
    }
  };

  Parameters parameters_;

  boost::signals2::signal<void (const GaitData*)> gait_data_signal_;
  boost::signals2::signal<void (const CommandData*)> command_data_signal_;

  class Impl;
  std::unique_ptr<Impl> impl_;
};
}
