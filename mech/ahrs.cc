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

#include "ahrs.h"

#include "mjlib/base/program_options_archive.h"

#include "base/attitude_estimator.h"
#include "base/common.h"
#include "base/now.h"
#include "base/telemetry_registry.h"

namespace mjmech {
namespace mech {
namespace {
using base::Point3D;
using base::Radians;
using base::Degrees;
const double kGravity = 9.81;

struct Parameters {
  double process_noise_attitude_dps = 0.01;
  double process_noise_bias_dps = 0.0256;
  double measurement_noise_accel_mps2 = 0.5;
  double measurement_noise_stationary_dps = 0.1;
  double initial_noise_attitude_deg = 2.0;
  double initial_noise_bias_dps = 0.2;
  double init_time_s = 1.0;
  double stationary_threshold_dps = 1.0;
  double stationary_threshold_delay_s = 1.0;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(process_noise_attitude_dps));
    a->Visit(MJ_NVP(process_noise_bias_dps));
    a->Visit(MJ_NVP(measurement_noise_accel_mps2));
    a->Visit(MJ_NVP(measurement_noise_stationary_dps));
    a->Visit(MJ_NVP(initial_noise_attitude_deg));
    a->Visit(MJ_NVP(initial_noise_bias_dps));
    a->Visit(MJ_NVP(init_time_s));
    a->Visit(MJ_NVP(stationary_threshold_dps));
    a->Visit(MJ_NVP(stationary_threshold_delay_s));
  }
};

struct AhrsDebugData {
  enum {
    kFilterSize = 7
  };

  boost::posix_time::ptime timestamp;

  std::array<double, kFilterSize> ukf_state = {};
  std::array<double, kFilterSize * kFilterSize> ukf_covariance = {};
  base::Point3D bias_body_dps;
  base::Point3D init_accel_mps2;
  int init_count = 0;
  boost::posix_time::ptime init_start;
  boost::posix_time::ptime last_measurement;
  boost::posix_time::ptime last_movement;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(timestamp));
    a->Visit(MJ_NVP(ukf_state));
    a->Visit(MJ_NVP(ukf_covariance));
    a->Visit(MJ_NVP(bias_body_dps));
    a->Visit(MJ_NVP(init_accel_mps2));
    a->Visit(MJ_NVP(init_count));
    a->Visit(MJ_NVP(init_start));
    a->Visit(MJ_NVP(last_measurement));
    a->Visit(MJ_NVP(last_movement));
  }
};
}

class Ahrs::Impl : boost::noncopyable {
 public:
  Impl(boost::asio::io_service& service,
       base::TelemetryRegistry* registry)
      : service_(service) {
    mjlib::base::ProgramOptionsArchive(&options_).Accept(&parameters_);

    registry->Register("ahrs", &ahrs_data_signal_);
    registry->Register("ahrs_debug", &ahrs_debug_signal_);
  }

  void Start() {
    estimator_.reset(
        new base::AttitudeEstimator(
            Radians(parameters_.process_noise_attitude_dps),
            Radians(parameters_.process_noise_bias_dps),
            parameters_.measurement_noise_accel_mps2 / kGravity,
            Radians(parameters_.measurement_noise_stationary_dps),
            Radians(parameters_.initial_noise_attitude_deg),
            Radians(parameters_.initial_noise_bias_dps)));
  }

  void ProcessImu(boost::posix_time::ptime timestamp,
                  const base::Point3D& accel_mps2,
                  const base::Point3D& body_rate_dps) {
    switch (data_.state) {
      case AhrsData::kUninitialized: // fall-through
      case AhrsData::kInitializing: {
        DoInitializing(accel_mps2, body_rate_dps);
        break;
      }
      case AhrsData::kOperational: {
        DoOperational(timestamp, accel_mps2, body_rate_dps);
        break;
      }
      case AhrsData::kFault: {
        break;
      }
    }

    data_debug_.last_measurement = timestamp;

    Emit(accel_mps2, body_rate_dps);
  }

  void DoInitializing(const base::Point3D& accel_mps2,
                      const base::Point3D& body_rate_dps) {
    if (!estimator_) {
      // We aren't started yet.
      return;
    }

    data_.state = AhrsData::kInitializing;
    if (data_debug_.init_start.is_not_a_date_time()) {
      data_debug_.init_start = base::Now(service_);
    }

    Point3D body_total_dps =
        data_debug_.bias_body_dps.scaled(data_debug_.init_count);
    body_total_dps = body_total_dps - body_rate_dps;

    Point3D total_accel_mps2 =
        data_debug_.init_accel_mps2.scaled(data_debug_.init_count);
    total_accel_mps2 = total_accel_mps2 + accel_mps2;

    data_debug_.init_count++;
    data_debug_.bias_body_dps =
        body_total_dps.scaled(1.0 / data_debug_.init_count);
    data_debug_.init_accel_mps2 =
        total_accel_mps2.scaled(1.0 / data_debug_.init_count);

    // Make an initial guess of pitch and roll, use that to populate
    // attitude while we are initializing.
    const auto a_g = data_debug_.init_accel_mps2.scaled(1.0 / kGravity);
    data_.attitude = base::AttitudeEstimator::AccelToOrientation(a_g);

    const auto now = base::Now(service_);
    auto elapsed = (now - data_debug_.init_start);

    if (elapsed >
        base::ConvertSecondsToDuration(parameters_.init_time_s)) {
      // We have sufficient bias.
      data_.state = AhrsData::kOperational;

      // Tell the estimator our initial stuff.
      Point3D filter_bias_rps = data_debug_.bias_body_dps.scaled(Radians(1));
      estimator_->SetInitialGyroBias(filter_bias_rps);

      Point3D accel_g = data_debug_.init_accel_mps2.scaled(1.0 / kGravity);
      estimator_->SetInitialAccel(accel_g);
    }
  }

  void DoOperational(boost::posix_time::ptime timestamp,
                     const Point3D& accel_mps2,
                     const Point3D& body_rate_dps) {
    const double delta_t_s =
        base::ConvertDurationToSeconds(
            timestamp - data_debug_.last_measurement);
    const Point3D filter_rate_rps = body_rate_dps.scaled(Radians(1));
    const Point3D filter_accel_g = accel_mps2.scaled(1.0 / kGravity);
    estimator_->ProcessMeasurement(
        delta_t_s,
        filter_rate_rps,
        filter_accel_g);

    if (body_rate_dps.length() > parameters_.stationary_threshold_dps) {
      data_debug_.last_movement = timestamp;
    }
    const double stationary_s = base::ConvertDurationToSeconds(
        timestamp - data_debug_.last_movement);
    if (stationary_s > parameters_.stationary_threshold_delay_s) {
      estimator_->ProcessStationary();
    }

    // Update our output attitude and bias.
    data_.attitude = estimator_->attitude();
    data_debug_.bias_body_dps =
        estimator_->gyro_bias_rps().scaled(Degrees(1));
  }

  void Emit(const Point3D& accel_mps2,
            const Point3D& body_rate_dps) {
    data_.timestamp = base::Now(service_);
    data_debug_.timestamp = data_.timestamp;

    data_.valid = (data_.state == AhrsData::kOperational);

    auto euler_rad = data_.attitude.euler_rad();

    data_.yaw_deg = base::Degrees(euler_rad.yaw);
    data_.pitch_deg = base::Degrees(euler_rad.pitch);
    data_.roll_deg = base::Degrees(euler_rad.roll);

    data_.body_rate_dps =
        body_rate_dps + data_debug_.bias_body_dps;

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

    ahrs_data_signal_(&data_);
    ahrs_debug_signal_(&data_debug_);
  }

  boost::asio::io_service& service_;
  Parameters parameters_;
  boost::program_options::options_description options_;

  std::unique_ptr<base::AttitudeEstimator> estimator_;

  AhrsData data_;
  AhrsDebugData data_debug_;

  boost::signals2::signal<void (const AhrsData*)> ahrs_data_signal_;
  boost::signals2::signal<void (const AhrsDebugData*)> ahrs_debug_signal_;

  boost::posix_time::ptime start_init_timestamp_;
};

Ahrs::Ahrs(boost::asio::io_service& service,
           base::TelemetryRegistry* registry)
    : impl_(new Impl(service, registry)) {}
Ahrs::~Ahrs() {}

void Ahrs::AsyncStart(mjlib::io::ErrorCallback handler) {
  impl_->Start();
  impl_->service_.post(std::bind(handler, mjlib::base::error_code()));
}

boost::program_options::options_description*
Ahrs::options() { return &impl_->options_; }

boost::signals2::signal<void (const AhrsData*)>*
Ahrs::ahrs_data_signal() { return &impl_->ahrs_data_signal_; }

void Ahrs::ProcessImu(boost::posix_time::ptime timestamp,
                      const base::Point3D& accel_mps2,
                      const base::Point3D& body_rate_dps) {
  impl_->ProcessImu(timestamp, accel_mps2, body_rate_dps);
}

}
}
