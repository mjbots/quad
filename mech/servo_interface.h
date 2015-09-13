// Copyright 2015 Josh Pieper, jjp@pobox.com.  All rights reserved.
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

#include "base/comm.h"
#include "base/error_code.h"

namespace legtool {
class ServoInterface : boost::noncopyable {
 public:
  struct Joint {
    int address;
    double angle_deg;
  };
  virtual void SetPose(const std::vector<Joint>&, ErrorHandler) = 0;

  enum PowerState {
    kPowerFree,
    kPowerBrake,
    kPowerEnable,
  };
  virtual void EnablePower(PowerState power_state, const std::vector<int>&,
                           ErrorHandler) = 0;

  typedef std::function<
    void (ErrorCode, std::vector<Joint>)> PoseHandler;

  virtual void GetPose(const std::vector<int>&, PoseHandler) = 0;

  struct Temperature {
    int address;
    double temperature_C;
  };

  typedef std::function<
    void (ErrorCode, std::vector<Temperature>)> TemperatureHandler;
  virtual void GetTemperature(const std::vector<int>&, TemperatureHandler) = 0;

  struct Voltage {
    int address;
    double voltage;
  };

  typedef std::function<void (
      ErrorCode, std::vector<Voltage>)> VoltageHandler;

  virtual void GetVoltage(const std::vector<int>&, VoltageHandler) = 0;

  /// @return the list of addresses which have been commanded at one
  /// point.
  virtual const std::set<int>& GetUsedAddresses() const = 0;
};

}
