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

#include <boost/asio/io_service.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <boost/noncopyable.hpp>
#include <boost/signals2/signal.hpp>

#include "base/comm.h"
#include "base/point3d.h"
#include "base/quaternion.h"
#include "base/visitor.h"

namespace mjmech {
namespace mech {

class Ahrs : boost::noncopyable {
 public:
  template <typename Context,
            typename ImuData>
  Ahrs(Context& context,
       boost::signals2::signal<void (const ImuData*)>* imu_signal)
      : Ahrs(context.service) {
    context.telemetry_registry->Register("ahrs", &ahrs_data_signal_);
    context.telemetry_registry->Register("ahrs_debug", &ahrs_debug_signal_);
    imu_signal->connect(std::bind(&Ahrs::HandleImuData<ImuData>, this,
                                  std::placeholders::_1));
  }

  ~Ahrs();

  void AsyncStart(base::ErrorHandler);

  struct Parameters {
    double process_noise_attitude_dps = 0.01;
    double process_noise_bias_dps = 0.0256;
    double measurement_noise_accel_mps2 = 0.5;
    double measurement_noise_stationary_mps2 = 0.5;
    double initial_noise_attitude_deg = 2.0;
    double initial_noise_bias_dps = 0.2;
    double init_time_s = 1.0;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(process_noise_attitude_dps));
      a->Visit(MJ_NVP(process_noise_bias_dps));
      a->Visit(MJ_NVP(measurement_noise_accel_mps2));
      a->Visit(MJ_NVP(measurement_noise_stationary_mps2));
      a->Visit(MJ_NVP(initial_noise_attitude_deg));
      a->Visit(MJ_NVP(initial_noise_bias_dps));
      a->Visit(MJ_NVP(init_time_s));
    }
  };

  Parameters* parameters();

  enum State : int {
    kUninitialized,
    kInitializing,
    kOperational,
    kFault,
  };

  static std::map<State, const char*> StateMapper() {
    return std::map<State, const char*>{
      { kUninitialized, "kUninitialized" },
      { kInitializing, "kInitializing" },
      { kOperational, "kOperational" },
      { kFault, "kFault" },
    };
  };

  struct AhrsData {
    boost::posix_time::ptime timestamp;

    State state = kUninitialized;
    bool valid = false;
    base::Quaternion attitude;
    double yaw_deg = 0.0;
    double pitch_deg = 0.0;
    double roll_deg = 0.0;

    base::Point3D body_rate_deg_s;
    base::Point3D body_accel_mps2;
    base::Point3D world_accel_mps2;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(timestamp));
      a->Visit(MJ_ENUM(state, StateMapper));
      a->Visit(MJ_NVP(valid));
      a->Visit(MJ_NVP(attitude));
      a->Visit(MJ_NVP(yaw_deg));
      a->Visit(MJ_NVP(pitch_deg));
      a->Visit(MJ_NVP(roll_deg));
      a->Visit(MJ_NVP(body_rate_deg_s));
      a->Visit(MJ_NVP(body_accel_mps2));
      a->Visit(MJ_NVP(world_accel_mps2));
    }
  };

  struct AhrsDebugData {
    enum {
      kFilterSize = 7
    };

    boost::posix_time::ptime timestamp;

    std::array<double, kFilterSize> ukf_state = {};
    std::array<double, kFilterSize * kFilterSize> ukf_covariance = {};
    base::Point3D bias_body_deg_s;
    base::Point3D init_accel_mps2;
    int init_count = 0;
    boost::posix_time::ptime init_start;
    boost::posix_time::ptime last_measurement;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(timestamp));
      a->Visit(MJ_NVP(ukf_state));
      a->Visit(MJ_NVP(ukf_covariance));
      a->Visit(MJ_NVP(bias_body_deg_s));
      a->Visit(MJ_NVP(init_accel_mps2));
      a->Visit(MJ_NVP(init_count));
      a->Visit(MJ_NVP(init_start));
      a->Visit(MJ_NVP(last_measurement));
    }
  };

  boost::signals2::signal<void (const AhrsData*)>* ahrs_data_signal() {
    return &ahrs_data_signal_;
  }

 private:
  boost::signals2::signal<void (const AhrsData*)> ahrs_data_signal_;
  boost::signals2::signal<void (const AhrsDebugData*)> ahrs_debug_signal_;

  Ahrs(boost::asio::io_service&);

  template <typename ImuData>
  void HandleImuData(const ImuData* data) {
    ProcessImu(data->timestamp, data->accel_mps2, data->body_rate_deg_s);
  }

  void ProcessImu(boost::posix_time::ptime timestamp,
                  const base::Point3D& accel_mps2,
                  const base::Point3D& body_rate_deg_s);

  class Impl;
  std::unique_ptr<Impl> impl_;
};
}
}
