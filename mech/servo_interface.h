// Copyright 2015-2019 Josh Pieper, jjp@pobox.com.  All rights reserved.
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
    int address = 0;

    /// The position to aim to be at *right now*.  !finite means start
    /// from wherever we happen to be.
    double angle_deg = 0.0;

    /// The speed to aim to be at *right now*.
    double velocity_dps = 0.0;

    /// The position to stop at if velocity is non-zero.  !finite
    /// means don't stop moving.
    double goal_deg = std::numeric_limits<double>::quiet_NaN();

    double power = 1.0;
  };
  virtual void SetPose(const std::vector<Joint>&, mjlib::io::ErrorCallback) = 0;

  enum PowerState {
    kPowerFree,
    kPowerBrake,
    kPowerEnable,
  };
  virtual void EnablePower(PowerState power_state, const std::vector<int>&,
                           mjlib::io::ErrorCallback) = 0;

  typedef std::function<
    void (mjlib::base::error_code, std::vector<Joint>)> PoseHandler;

  virtual void GetPose(const std::vector<int>&, PoseHandler) = 0;

  struct Temperature {
    int address;
    double temperature_C;
  };

  typedef std::function<
    void (mjlib::base::error_code, std::vector<Temperature>)> TemperatureHandler;
  virtual void GetTemperature(const std::vector<int>&, TemperatureHandler) = 0;

  struct Voltage {
    int address;
    double voltage;
  };

  typedef std::function<void (
      mjlib::base::error_code, std::vector<Voltage>)> VoltageHandler;

  virtual void GetVoltage(const std::vector<int>&, VoltageHandler) = 0;
};

}
}
