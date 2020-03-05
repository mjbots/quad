// Copyright 2020 Josh Pieper, jjp@pobox.com.
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

#include <memory>
#include <string>

#include <boost/asio/executor.hpp>

#include "mjlib/io/async_types.h"

#include "mech/attitude_data.h"

namespace mjmech {
namespace mech {

/// Read IMU data from the mjbots quad pi3 hat.
///
/// For now, it operates in a polling manner.
class Rpi3HatImu {
 public:
  struct Options {
    std::string device = "/dev/spidev0.0";
    int speed = 10000000;
    int cpu_affinity = -1;
  };

  Rpi3HatImu(const boost::asio::executor&, const Options&);
  ~Rpi3HatImu();

  /// Read the next available IMU sample and store it in @p data.
  ///
  /// @param callback will be invoked when the operation completes or
  /// fails.
  void ReadImu(AttitudeData* data, mjlib::io::ErrorCallback callback);

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}
}
