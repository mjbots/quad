// Copyright 2014-2020 Josh Pieper, jjp@pobox.com.
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

#include <clipp/clipp.h>

#include <boost/asio/executor.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>
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
  GaitDriver(const boost::asio::executor&,
             base::TelemetryRegistry*);
  ~GaitDriver();

  void AsyncStart(mjlib::io::ErrorCallback);

  void SetGait(std::unique_ptr<RippleGait>);

  /// Start the gait engine if it is not running and make it execute
  /// the given command.
  void SetCommand(const Command& command);

  /// Set all servos to be unpowered and stop executing the gait
  /// engine.
  void SetFree();

  /// Set all servos to a "safe" configuration.  This should be usable
  /// from any state the machine could be in to achieve minimal
  /// possible damage due to loss of control.
  void SetSafe();

  /// Prepare legs to stand up by gently positioning them above the
  /// idle state.
  void CommandPrepositioning();

  /// Gently stand up.
  void CommandStandup();

  /// Position feet to sit down.
  void CommandPrepareToSit();

  /// Gently sit down.
  void CommandSitDown();


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
    kPreparingToSit,
    kSitting,
    kSafe,
  };

  static std::map<State, const char*> StateMapper() {
    return std::map<State, const char*>{
      { kUnpowered, "kUnpowered" },
      { kActive, "kActive" },
      { kPrepositioning, "kPrepositioning" },
      { kStandup, "kStandup" },
      { kPreparingToSit, "kPreparingToSit" },
      { kSitting, "kSitting" },
      { kSafe, "kSafe" },
    };
  }

  struct GaitData {
    boost::posix_time::ptime timestamp;

    State state;
    base::Transform body_robot;
    base::Transform cog_robot;
    base::Transform body_world;
    base::Transform robot_world;

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

  clipp::group program_options();

  const Command& input_command() const;
  const Command& gait_command() const;
  const Gait* gait() const;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};
}
}
