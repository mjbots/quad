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

#include <functional>

#include "point3d.h"
#include "quaternion.h"
#include "ukf_filter.h"

namespace mjmech {
namespace base {

class AttitudeEstimator {
 public:
  enum {
    kNumStates = 7,
  };
  typedef UkfFilter<double, kNumStates> Filter;

  AttitudeEstimator(double process_noise_gyro,
                    double process_noise_bias,
                    double measurement_noise_accel,
                    double measurement_noise_stationary,
                    double initial_noise_attitude,
                    double initial_noise_bias)
      : initialized_(false),
        filter_(
          (Filter::State() <<
           1.0, 0.0, 0.0, 0.0,
           0.0, 0.0, 0.0).finished(),
          Eigen::DiagonalMatrix<double, 7, 7>(
              (Filter::State() <<
               initial_noise_attitude,
               initial_noise_attitude,
               initial_noise_attitude,
               initial_noise_attitude,
               initial_noise_bias,
               initial_noise_bias,
               initial_noise_bias).finished()),
          Eigen::DiagonalMatrix<double, 7, 7>(
              (Filter::State() <<
               process_noise_gyro,
               process_noise_gyro,
               process_noise_gyro,
               process_noise_gyro,
               process_noise_bias,
               process_noise_bias,
               process_noise_bias).finished())),
        measurement_noise_accel_(measurement_noise_accel),
        measurement_noise_stationary_(measurement_noise_stationary),
        initial_bias_uncertainty_(initial_noise_bias) {
  }

  std::vector<std::string> state_names() const {
    return { "w", "x", "y", "z", "gx", "gy", "gz" };
  }

  void SetInitialGyroBias(const Point3D& body_rate_rps) {
    filter_.state()(4) = body_rate_rps.x;
    filter_.state()(5) = body_rate_rps.y;
    filter_.state()(6) = body_rate_rps.z;
  }

  void SetInitialAccel(const Point3D& accel_g) {
    Point3D normalized = accel_g.scaled(accel_g.length());

    Quaternion a = AccelToOrientation(normalized);
    filter_.state()(0) = a.w();
    filter_.state()(1) = a.x();
    filter_.state()(2) = a.y();
    filter_.state()(3) = a.z();

    initialized_ = true;
  }

  const Filter::State& state() const {
    return filter_.state();
  }

  const Filter::Covariance& covariance() const {
    return filter_.covariance();
  }

  std::vector<double> covariance_diag() const {
    std::vector<double> result;
    for (int i = 0; i < 7; i++) {
      result.push_back(filter_.covariance()(i, i));
    }
    return result;
  }

  double pitch_error(double pitch) const {
    return (attitude() *
            Quaternion::FromEuler(0., pitch, 0).
            conjugated()).euler().pitch_rad;
  }

  double yaw_rad() const { return attitude().euler().yaw_rad; }
  double pitch_rad() const { return attitude().euler().pitch_rad; }
  double roll_rad() const { return attitude().euler().roll_rad; }

  double pitch_rps() const {
    return current_gyro_rps_.x + filter_.state()(4);
  }

  double roll_rps() const {
    return current_gyro_rps_.y + filter_.state()(5);
  }

  double yaw_rps() const {
    return -(current_gyro_rps_.z + filter_.state()(6));
  }

  Point3D gyro_bias_rps() const {
    return Point3D(filter_.state()(4, 0),
                   filter_.state()(5, 0),
                   filter_.state()(6, 0));
  }

  Quaternion attitude() const {
    return Quaternion(filter_.state()(0),
                      filter_.state()(1),
                      filter_.state()(2),
                      filter_.state()(3));
  }

  Filter::State ProcessFunction(
      const Filter::State& state, double dt_s) const {
    Filter::State result = state;

    Quaternion this_attitude = Quaternion(
        result(0), result(1), result(2), result(3)).normalized();
    Quaternion delta;

    Quaternion advanced = Quaternion::IntegrateRotationRate(
        current_gyro_rps_ + Point3D(result(4), result(5), result(6)),
        dt_s);
    delta = delta * advanced;

    Quaternion next_attitude = (this_attitude * delta).normalized();
    result(0) = next_attitude.w();
    result(1) = next_attitude.x();
    result(2) = next_attitude.y();
    result(3) = next_attitude.z();
    return result;
  }

  static Eigen::Matrix<double, 3, 1> OrientationToAccel(
      const Quaternion& attitude) {
    Point3D gravity(0., 0., 1.);
    Point3D expected =
        attitude.conjugated().Rotate(gravity);
    return Eigen::Matrix<double, 3, 1>(expected.x, expected.y, expected.z);;
  }

  static Eigen::Matrix<double, 3, 1> MeasureAccel(
      const Filter::State& s) {
    return OrientationToAccel(
        Quaternion(s(0), s(1), s(2), s(3)).normalized());
  }

  Eigen::Matrix<double, 1, 1> MeasureRotation(
      const Filter::State& s) const {
    Point3D bias_rps(s(4), s(5), s(6));
    Point3D rotation = current_gyro_rps_ + bias_rps;
    return (Eigen::Matrix<double, 1, 1>() << rotation.length()).finished();
  }

  static Quaternion AccelToOrientation(const Point3D& n) {
    Quaternion::Euler euler;
    euler.roll_rad = std::atan2(-n.x, n.z);
    euler.pitch_rad = std::atan2(n.y, std::sqrt(n.x * n.x + n.z * n.z));

    return Quaternion::FromEuler(euler);
  }

  void ProcessStationary() {
    using namespace std::placeholders;
    filter_.UpdateMeasurement(
        std::bind(&AttitudeEstimator::MeasureRotation, this,
                  std::placeholders::_1),
        (Eigen::Matrix<double, 1, 1>() << 0.0).finished(),
        (Eigen::Matrix<double, 1, 1>() <<
         measurement_noise_stationary_).finished());
  }

  void ProcessMeasurement(
      double delta_t_s,
      const Point3D& body_rate_rps,
      const Point3D& accel_g) {
    current_gyro_rps_ = body_rate_rps;

    const Point3D norm_g = accel_g.scaled(1.0 / accel_g.length());

    if (!initialized_) {
      initialized_ = true;
      Quaternion start = AccelToOrientation(norm_g);
      filter_.state()(0) = start.w();
      filter_.state()(1) = start.x();
      filter_.state()(2) = start.y();
      filter_.state()(3) = start.z();
    }

    using namespace std::placeholders;
    filter_.UpdateState(
        delta_t_s,
        std::bind(&AttitudeEstimator::ProcessFunction, this,
                  std::placeholders::_1, std::placeholders::_2));
    filter_.UpdateMeasurement(
        MeasureAccel,
        (Eigen::Matrix<double, 3, 1>() <<
         norm_g.x, norm_g.y, norm_g.z).finished(),
        (Eigen::DiagonalMatrix<double, 3, 3>(
            (Eigen::Matrix<double, 3, 1>() <<
             measurement_noise_accel_,
             measurement_noise_accel_,
             measurement_noise_accel_).finished())));
  }

 private:
  bool initialized_;
  Filter filter_;
  double measurement_noise_accel_;
  double measurement_noise_stationary_;
  double initial_bias_uncertainty_;

  Point3D current_gyro_rps_;
};
}
}
