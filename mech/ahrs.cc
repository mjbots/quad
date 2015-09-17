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

#include "ahrs.h"

#include "base/attitude_estimator.h"
#include "base/common.h"

namespace mjmech {
namespace mech {
namespace {
using base::Point3D;
using base::Radians;
using base::Degrees;
const double kGravity = 9.81;
}

class Ahrs::Impl : boost::noncopyable {
 public:
  Impl(Ahrs* parent, boost::asio::io_service& service)
      : parent_(parent),
        service_(service) {}

  void Start() {
    estimator_.reset(
        new base::AttitudeEstimator(
            Radians(parameters_.process_noise_attitude_dps),
            Radians(parameters_.process_noise_bias_dps),
            parameters_.measurement_noise_accel_mps2 / kGravity,
            parameters_.measurement_noise_accel_mps2 / kGravity,
            Radians(parameters_.initial_noise_attitude_deg),
            Radians(parameters_.initial_noise_bias_dps)));
  }

  void ProcessImu(boost::posix_time::ptime timestamp,
                  const base::Point3D& accel_mps2,
                  const base::Point3D& body_rate_deg_s) {
    switch (data_.state) {
      case kUninitialized: // fall-through
      case kInitializing: {
        DoInitializing(accel_mps2, body_rate_deg_s);
        break;
      }
      case kOperational: {
        DoOperational(timestamp, accel_mps2, body_rate_deg_s);
        break;
      }
      case kFault: {
        break;
      }
    }

    data_debug_.last_measurement = timestamp;

    Emit(accel_mps2, body_rate_deg_s);
  }

  void DoInitializing(const base::Point3D& accel_mps2,
                      const base::Point3D& body_rate_deg_s) {
    if (!estimator_) {
      // We aren't started yet.
      return;
    }

    data_.state = kInitializing;
    if (data_debug_.init_start.is_not_a_date_time()) {
      data_debug_.init_start =
          boost::posix_time::microsec_clock::universal_time();
    }

    Point3D body_total_deg_s =
        data_debug_.bias_body_deg_s.scaled(data_debug_.init_count);
    body_total_deg_s = body_total_deg_s - body_rate_deg_s;

    Point3D total_accel_mps2 =
        data_debug_.init_accel_mps2.scaled(data_debug_.init_count);
    total_accel_mps2 = total_accel_mps2 + accel_mps2;

    data_debug_.init_count++;
    data_debug_.bias_body_deg_s =
        body_total_deg_s.scaled(1.0 / data_debug_.init_count);
    data_debug_.init_accel_mps2 =
        total_accel_mps2.scaled(1.0 / data_debug_.init_count);

    // Make an initial guess of pitch and roll, use that to populate
    // attitude while we are initializing.
    const auto a_g = data_debug_.init_accel_mps2.scaled(1.0 / kGravity);
    data_.attitude =
        base::AttitudeEstimator::AccelToOrientation(a_g.x, a_g.y, a_g.z);

    const auto now = boost::posix_time::microsec_clock::universal_time();
    auto elapsed = (now - data_debug_.init_start);

    if (elapsed >
        base::ConvertSecondsToDuration(parameters_.init_time_s)) {
      // We have sufficient bias.
      data_.state = kOperational;

      // TODO jpieper: Tell the estimator our initial stuff.
      Point3D filter_bias_rps = data_debug_.bias_body_deg_s.scaled(Radians(1));
      estimator_->SetInitialGyroBias(
          -filter_bias_rps.z,
          filter_bias_rps.x,
          filter_bias_rps.y);

      Point3D accel_g = data_debug_.init_accel_mps2.scaled(1.0 / kGravity);
      estimator_->SetInitialAccel(accel_g.x, accel_g.y, accel_g.z);
    }
  }

  void DoOperational(boost::posix_time::ptime timestamp,
                     const Point3D& accel_mps2,
                     const Point3D& body_rate_deg_s) {
    const double delta_t_s =
        base::ConvertDurationToSeconds(
            timestamp - data_debug_.last_measurement);
    const Point3D filter_rate_rps = body_rate_deg_s.scaled(Radians(1));
    const Point3D filter_accel_g = accel_mps2.scaled(1.0 / kGravity);
    estimator_->ProcessMeasurement(
        delta_t_s,
        -filter_rate_rps.z,
        filter_rate_rps.x,
        filter_rate_rps.y,
        filter_accel_g.x,
        filter_accel_g.y,
        filter_accel_g.z);

    // Update our output attitude and bias.
    data_.attitude = estimator_->attitude();
    data_debug_.bias_body_deg_s =
        estimator_->gyro_bias_rps().scaled(Degrees(1));
  }

  void Emit(const Point3D& accel_mps2,
            const Point3D& body_rate_deg_s) {
    data_.timestamp = boost::posix_time::microsec_clock::universal_time();
    data_debug_.timestamp = data_.timestamp;

    data_.valid = (data_.state == kOperational);

    auto euler = data_.attitude.euler();

    data_.yaw_deg = base::Degrees(euler.yaw_rad);
    data_.pitch_deg = base::Degrees(euler.pitch_rad);
    data_.roll_deg = base::Degrees(euler.roll_rad);

    data_.body_rate_deg_s =
        body_rate_deg_s + data_debug_.bias_body_deg_s;

    // Find our nominal accel given our current attitude.
    auto body_gravity_g =
        base::AttitudeEstimator::OrientationToAccel(data_.attitude);
    Point3D body_gravity_mps2 = Point3D(body_gravity_g[0],
                                        body_gravity_g[1],
                                        body_gravity_g[2]).scaled(kGravity);
    data_.body_accel_mps2 =
        accel_mps2 - body_gravity_mps2;
    data_.world_accel_mps2 =
        data_.attitude.Rotate(data_.body_accel_mps2);

    const auto& state = estimator_->state();
    const auto& covariance = estimator_->covariance();

    for (size_t i = 0; i < base::AttitudeEstimator::kNumStates; i++) {
      data_debug_.ukf_state[i] = state[i];
      for (size_t j = 0; j < base::AttitudeEstimator::kNumStates; j++) {
        data_debug_.ukf_covariance[
            i * base::AttitudeEstimator::kNumStates + j] = covariance(i, j);
      }
    }

    parent_->ahrs_data_signal_(&data_);
    parent_->ahrs_debug_signal_(&data_debug_);
  }

  Ahrs* const parent_;
  boost::asio::io_service& service_;
  Parameters parameters_;

  std::unique_ptr<base::AttitudeEstimator> estimator_;

  AhrsData data_;
  AhrsDebugData data_debug_;

  boost::posix_time::ptime start_init_timestamp_;
};

Ahrs::Ahrs(boost::asio::io_service& service)
    : impl_(new Impl(this, service)) {}
Ahrs::~Ahrs() {}

void Ahrs::AsyncStart(base::ErrorHandler handler) {
  impl_->Start();
  impl_->service_.post(std::bind(handler, base::ErrorCode()));
}

Ahrs::Parameters* Ahrs::parameters() { return &impl_->parameters_; }

void Ahrs::ProcessImu(boost::posix_time::ptime timestamp,
                      const base::Point3D& accel_mps2,
                      const base::Point3D& body_rate_deg_s) {
  impl_->ProcessImu(timestamp, accel_mps2, body_rate_deg_s);
}

}
}
