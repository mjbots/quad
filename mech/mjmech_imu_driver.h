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
#include <boost/signals2/signal.hpp>

#include "base/comm.h"
#include "base/point3d.h"
#include "base/visitor.h"

namespace mjmech {
namespace mech {
/// Driver for the MAX21000 MMA8451Q driver on the mjmech
/// daughterboards.
class MjmechImuDriver : boost::noncopyable {
 public:
  template <typename Context>
  MjmechImuDriver(Context& context)
      : MjmechImuDriver(context.service,
                        &context.telemetry_registry) {}

  template <typename TelemetryRegistry>
  MjmechImuDriver(boost::asio::io_service& service,
                  TelemetryRegistry* telemetry_registry)
      : MjmechImuDriver(service) {
    telemetry_registry->Register("imu", &imu_data_signal_);
    telemetry_registry->Register("imu_config", &imu_config_signal_);
  }

  MjmechImuDriver(boost::asio::io_service&);
  ~MjmechImuDriver();

  void AsyncStart(base::ErrorHandler handler);

  struct Parameters {
    std::string i2c_device;
    int gyro_address = 0x59;
    int accel_address = 0x1d;

    double accel_g = 4.0;
    double rotation_deg_s = 500.0;
    double rate_hz = 100.0;

    double roll_deg = 0;
    double pitch_deg = 0;
    double yaw_deg = 0;
    base::Point3D gyro_scale = base::Point3D(1, 1, 1);
    base::Point3D accel_scale = base::Point3D(1, 1, 1);

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(i2c_device));
      a->Visit(MJ_NVP(gyro_address));
      a->Visit(MJ_NVP(accel_address));
      a->Visit(MJ_NVP(accel_g));
      a->Visit(MJ_NVP(rotation_deg_s));
      a->Visit(MJ_NVP(rate_hz));
      a->Visit(MJ_NVP(roll_deg));
      a->Visit(MJ_NVP(pitch_deg));
      a->Visit(MJ_NVP(yaw_deg));
      a->Visit(MJ_NVP(gyro_scale));
      a->Visit(MJ_NVP(accel_scale));
    }
  };

  Parameters* parameters() { return &parameters_; }

  /// Data recorded here should have the following conventions:
  ///
  ///  +x axis is to the right
  ///  +y axis is forward
  ///  +z axis is up
  ///
  ///  Positive rotations are clockwise while looking down the
  ///  relevant axis.
  struct ImuData {
    boost::posix_time::ptime timestamp;
    base::Point3D accel_mps2;
    base::Point3D body_rate_deg_s;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(timestamp));
      a->Visit(MJ_NVP(accel_mps2));
      a->Visit(MJ_NVP(body_rate_deg_s));
    }
  };

  struct ImuConfig {
    boost::posix_time::ptime timestamp;
    double rate_hz = 0.0;
    double gyro_bw_hz = 0.0;
    double accel_g = 0.0;
    double rotation_deg_s = 0.0;

    double roll_deg = 0;
    double pitch_deg = 0;
    double yaw_deg = 0;

    base::Point3D gyro_scale = base::Point3D(1, 1, 1);
    base::Point3D accel_scale = base::Point3D(1, 1, 1);

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(timestamp));
      a->Visit(MJ_NVP(rate_hz));
      a->Visit(MJ_NVP(gyro_bw_hz));
      a->Visit(MJ_NVP(accel_g));
      a->Visit(MJ_NVP(rotation_deg_s));
      a->Visit(MJ_NVP(roll_deg));
      a->Visit(MJ_NVP(pitch_deg));
      a->Visit(MJ_NVP(yaw_deg));
      a->Visit(MJ_NVP(gyro_scale));
      a->Visit(MJ_NVP(accel_scale));
    }
  };

  boost::signals2::signal<void (const ImuData*)>* imu_data_signal() {
    return &imu_data_signal_;
  }

 private:
  boost::signals2::signal<void (const ImuData*)> imu_data_signal_;
  boost::signals2::signal<void (const ImuConfig*)> imu_config_signal_;
  Parameters parameters_;

  class Impl;
  std::unique_ptr<Impl> impl_;
};
}
}
