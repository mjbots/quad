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

namespace mjmech {
namespace mech {
class RippleGait;
class ServoInterface;

/// Advance a gait through time, accept commands, and send those gait
/// commands out to some servos.
class GaitDriver : boost::noncopyable {
 public:
  template <typename TelemetryRegistry,
            typename AhrsData>
  GaitDriver(boost::asio::io_service& service,
             TelemetryRegistry* telemetry_registry,
             ServoInterface* servo,
             boost::signals2::signal<void (const AhrsData*)>* body_ahrs_signal)
      : GaitDriver(service, servo) {
    body_ahrs_signal->connect(
        std::bind(&GaitDriver::HandleBodyAhrs<AhrsData>,
                  this, std::placeholders::_1));
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
      a->Visit(MJ_NVP(period_s));
      a->Visit(MJ_NVP(command_timeout_s));
      a->Visit(MJ_NVP(idle_time_s));
    }
  };

  Parameters* parameters() { return &parameters_; }

 private:
  GaitDriver(boost::asio::io_service& service,
             ServoInterface* servo);

  template <typename AhrsData>
  void HandleBodyAhrs(const AhrsData* data) {
    ProcessBodyAhrs(
        data->timestamp, data->valid,
        data->attitude, data->body_rate_deg_s);
  }

  void ProcessBodyAhrs(boost::posix_time::ptime timestamp,
                       bool valid,
                       const base::Quaternion& attitude,
                       const base::Point3D& body_rate_deg_s);

  enum State : int {
    kUnpowered,
    kActive,
  };

  static std::map<State, const char*> StateMapper() {
    return std::map<State, const char*>{
      { kUnpowered, "kUnpowered" },
      { kActive, "kActive" },
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
    base::Point3D body_rate_deg_s;

    std::array<base::Point3D, 4> legs;
    JointCommand command;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(timestamp));
      a->Visit(MJ_ENUM(state, StateMapper));
      a->Visit(MJ_NVP(body_robot));
      a->Visit(MJ_NVP(cog_robot));
      a->Visit(MJ_NVP(body_world));
      a->Visit(MJ_NVP(robot_world));
      a->Visit(MJ_NVP(attitude));
      a->Visit(MJ_NVP(body_rate_deg_s));
      a->Visit(MJ_NVP(legs));
      a->Visit(MJ_NVP(command));
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

  Parameters parameters_;

  boost::signals2::signal<void (const GaitData*)> gait_data_signal_;
  boost::signals2::signal<void (const CommandData*)> command_data_signal_;

  class Impl;
  std::unique_ptr<Impl> impl_;
};
}
}
