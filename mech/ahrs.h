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

#include <boost/asio/io_service.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <boost/noncopyable.hpp>
#include <boost/program_options.hpp>
#include <boost/signals2/signal.hpp>

#include "mjlib/base/visitor.h"
#include "mjlib/io/async_types.h"

#include "base/context.h"
#include "base/point3d.h"
#include "base/quaternion.h"

#include "ahrs_data.h"

namespace mjmech {
namespace mech {

class Ahrs : boost::noncopyable {
 public:
  template <typename Context,
            typename ImuData>
  Ahrs(Context& context,
       boost::signals2::signal<void (const ImuData*)>* imu_signal)
      : Ahrs(context.service, context.telemetry_registry.get()) {
    imu_signal->connect(std::bind(&Ahrs::HandleImuData<ImuData>, this,
                                  std::placeholders::_1));
  }

  ~Ahrs();

  void AsyncStart(mjlib::io::ErrorCallback);

  boost::program_options::options_description* options();

  boost::signals2::signal<void (const AhrsData*)>* ahrs_data_signal();

 private:
  Ahrs(boost::asio::io_service&, base::TelemetryRegistry*);

  template <typename ImuData>
  void HandleImuData(const ImuData* data) {
    ProcessImu(data->timestamp, data->accel_mps2, data->body_rate_dps);
  }

  void ProcessImu(boost::posix_time::ptime timestamp,
                  const base::Point3D& accel_mps2,
                  const base::Point3D& body_rate_dps);

  class Impl;
  std::unique_ptr<Impl> impl_;
};
}
}
