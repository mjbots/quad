// Copyright 2015-2020 Josh Pieper, jjp@pobox.com.  All rights reserved.
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

#include <set>
#include <vector>

#include "mjlib/base/error_code.h"
#include "mjlib/io/async_types.h"

namespace mjmech {
namespace mech {
class ServoInterface : boost::noncopyable {
 public:
  virtual ~ServoInterface() {}

  struct Joint {
    int32_t address = 0;

    /// The position to aim to be at *right now*.  !finite means start
    /// from wherever we happen to be.
    double angle_deg = 0.0;

    /// The speed to aim to be at *right now*.
    double velocity_dps = 0.0;

    /// The position to stop at if velocity is non-zero.  !finite
    /// means don't stop moving.
    double goal_deg = std::numeric_limits<double>::quiet_NaN();

    /// The maximum joint for the motor to apply.
    static constexpr double kDefaultMaxTorque = 40.0;
    double max_torque_Nm = kDefaultMaxTorque;
    double max_torque_scale = 1.0;

    /// A "feedforward" torque to apply even with no control error.
    double torque_Nm = 0.0;

    double kp = 1.0;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(address));
      a->Visit(MJ_NVP(angle_deg));
      a->Visit(MJ_NVP(velocity_dps));
      a->Visit(MJ_NVP(goal_deg));
      a->Visit(MJ_NVP(max_torque_Nm));
      a->Visit(MJ_NVP(max_torque_scale));
      a->Visit(MJ_NVP(torque_Nm));
      a->Visit(MJ_NVP(kp));
    }
  };
  virtual void SetPose(const std::vector<Joint>&, mjlib::io::ErrorCallback) = 0;

  enum PowerState {
    kPowerFree,
    kPowerBrake,
    kPowerEnable,
  };

  static std::map<PowerState, const char*> PowerStateMapper() {
    return {
      { kPowerFree, "kPowerFree" },
      { kPowerBrake, "kPowerBrake" },
      { kPowerEnable, "kPowerEnable" },
    };
  }

  virtual void EnablePower(PowerState power_state, const std::vector<int>&,
                           mjlib::io::ErrorCallback) = 0;

  struct StatusOptions {
    bool pose = false;
    bool velocity = false;
    bool temperature = false;
    bool torque = false;
    bool voltage = false;
    bool error = false;
  };

  struct JointStatus {
    int address = 0;
    bool torque_on = false;
    int8_t mode = 0;
    uint32_t error = 0;
    std::optional<double> angle_deg;
    std::optional<double> temperature_C;
    std::optional<double> voltage;
    std::optional<double> velocity_dps;
    std::optional<double> torque_Nm;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(address));
      a->Visit(MJ_NVP(torque_on));
      a->Visit(MJ_NVP(mode));
      a->Visit(MJ_NVP(error));
      a->Visit(MJ_NVP(angle_deg));
      a->Visit(MJ_NVP(temperature_C));
      a->Visit(MJ_NVP(voltage));
      a->Visit(MJ_NVP(velocity_dps));
      a->Visit(MJ_NVP(torque_Nm));
    }
  };

  typedef std::function<
    void (const mjlib::base::error_code&,
          const std::vector<JointStatus>&)> StatusHandler;

  virtual void GetStatus(const std::vector<int>&, const StatusOptions&,
                         StatusHandler) = 0;

  // If there are errors which can be cleared in a way that does not
  // affect behavior, do so here.
  virtual void ClearErrors(const std::vector<int>& ids,
                           mjlib::io::ErrorCallback) = 0;


  /// Command all servos and query their state in one operation.  This
  /// is intended to be the fastest way to update the state of the
  /// entire system.
  ///
  /// @p command and @p result are aliased and must remain valid until
  /// @p callback is invoked.
  virtual void Update(
      PowerState,
      const StatusOptions&,
      const std::vector<Joint>* command,
      std::vector<JointStatus>* result,
      mjlib::io::ErrorCallback callback) = 0;
};

}
}
