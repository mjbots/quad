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
#include <boost/program_options.hpp>
#include <boost/signals2/signal.hpp>

#include "base/context.h"
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
  template <typename AhrsData>
  GaitDriver(boost::asio::io_service& service,
             base::TelemetryRegistry* telemetry_registry,
             ServoInterface* servo,
             boost::signals2::signal<void (const AhrsData*)>* body_ahrs_signal)
      : GaitDriver(service, telemetry_registry, servo) {
    body_ahrs_signal->connect(
        std::bind(&GaitDriver::HandleBodyAhrs<AhrsData>,
                  this, std::placeholders::_1));
  }

  ~GaitDriver();

  void SetGait(std::unique_ptr<RippleGait>);

  /// Start the gait engine if it is not running and make it execute
  /// the given command.
  void SetCommand(const Command& command);

  /// Set all servos to be unpowered and stop executing the gait
  /// engine.
  void SetFree();

  boost::program_options::options_description* options();

 private:
  GaitDriver(boost::asio::io_service& service,
             base::TelemetryRegistry*,
             ServoInterface* servo);

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
