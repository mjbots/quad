// Copyright 2019 Josh Pieper, jjp@pobox.com.  All rights reserved.
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

#include "mech/moteus_servo.h"

namespace mjmech {
namespace mech {

class MoteusServo::Impl {
 public:
  Parameters parameters_;
  boost::program_options::options_description options_;
};

MoteusServo::MoteusServo() : impl_(std::make_unique<Impl>()) {}
MoteusServo::~MoteusServo() {}

MoteusServo::Parameters* MoteusServo::parameters() {
  return &impl_->parameters_;
}

boost::program_options::options_description* MoteusServo::options() {
  return &impl_->options_;
}

void MoteusServo::SetPose(const std::vector<Joint>&, mjlib::io::ErrorCallback) {
}

void MoteusServo::EnablePower(PowerState, const std::vector<int>&,
                              mjlib::io::ErrorCallback) {
}

void MoteusServo::GetPose(const std::vector<int>&, PoseHandler) {
}

void MoteusServo::GetTemperature(const std::vector<int>&, TemperatureHandler) {
}

void MoteusServo::GetVoltage(const std::vector<int>&, VoltageHandler) {
}

}
}
