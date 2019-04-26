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

#pragma once

#include <boost/program_options.hpp>

#include "mjlib/io/stream_factory.h"

#include "mech/servo_interface.h"

namespace mjmech {
namespace mech {

class MoteusServo : public ServoInterface {
 public:
  MoteusServo(boost::asio::io_service&, mjlib::io::StreamFactory&);
  ~MoteusServo() override;

  struct Parameters {
    double max_current = 30.0;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(max_current));
    }
  };

  Parameters* parameters();
  boost::program_options::options_description* options();

  void AsyncStart(mjlib::io::ErrorCallback);

  void SetPose(const std::vector<Joint>&, mjlib::io::ErrorCallback) override;
  void EnablePower(PowerState, const std::vector<int>&,
                   mjlib::io::ErrorCallback) override;
  void GetPose(const std::vector<int>&, PoseHandler) override;
  void GetTemperature(const std::vector<int>&, TemperatureHandler) override;
  void GetVoltage(const std::vector<int>&, VoltageHandler) override;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}
}
