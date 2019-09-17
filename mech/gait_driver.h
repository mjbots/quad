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

#pragma once

#include <boost/asio/io_service.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <boost/program_options.hpp>
#include <boost/signals2/signal.hpp>

#include "mjlib/base/visitor.h"

#include "base/context.h"
#include "base/tf.h"

#include "mech/gait.h"
#include "mech/servo_interface.h"

namespace mjmech {
namespace mech {
class RippleGait;

/// Advance a gait through time, accept commands, and send those gait
/// commands out to some servos.
class GaitDriver : boost::noncopyable {
 public:
  template <typename AhrsData>
  GaitDriver(boost::asio::io_context& service,
             base::TelemetryRegistry* telemetry_registry,
             boost::signals2::signal<void (const AhrsData*)>* body_ahrs_signal)
      : GaitDriver(service, telemetry_registry) {
    body_ahrs_signal->connect(
        std::bind(&GaitDriver::HandleBodyAhrs<AhrsData>,
                  this, std::placeholders::_1));
  }

  ~GaitDriver();

  void AsyncStart(mjlib::io::ErrorCallback);

  void SetGait(std::unique_ptr<RippleGait>);

  /// Start the gait engine if it is not running and make it execute
  /// the given command.
  void SetCommand(const Command& command);

  /// Set all servos to be unpowered and stop executing the gait
  /// engine.
  void SetFree();

  /// Prepare legs to stand up by gently positioning them above the
  /// idle state.
  void CommandPrepositioning();

  /// Gently stand up.
  void CommandStandup();

  /// Gently sit down.
  void CommandSitting();


  struct CommandState {
    boost::posix_time::ptime timestamp;
    ServoInterface::PowerState power_state = ServoInterface::kPowerFree;
    std::vector<ServoInterface::Joint> joints;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(timestamp));
      a->Visit(MJ_ENUM(power_state, ServoInterface::PowerStateMapper));
      a->Visit(MJ_NVP(joints));
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

  struct UpdateResult {
    CommandState command_state;
    GaitData gait_data;
  };

  /// This should be called at a high rate when the gait driver is
  /// active.  It returns the current set of commands to request.
  UpdateResult Update(double period_s);

  boost::program_options::options_description* options();

  const Command& input_command() const;
  const Command& gait_command() const;
  const Gait* gait() const;

 private:
  GaitDriver(boost::asio::io_context& service,
             base::TelemetryRegistry*);

  template <typename AhrsData>
  void HandleBodyAhrs(const AhrsData* data) {
    ProcessBodyAhrs(
        data->timestamp, data->valid,
        data->attitude, data->body_rate_dps);
  }

  void ProcessBodyAhrs(boost::posix_time::ptime timestamp,
                       bool valid,
                       const base::Quaternion& attitude,
                       const base::Point3D& body_rate_dps);

  class Impl;
  std::unique_ptr<Impl> impl_;
};
}
}
