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

  void SetInitialGyroBias(
      double yaw_rps, double pitch_rps, double roll_rps) {
    filter_.state()(4) = yaw_rps;
    filter_.state()(5) = pitch_rps;
    filter_.state()(6) = roll_rps;
  }

  void SetInitialAccel(double x_g, double y_g, double z_g) {
    double norm = std::sqrt(x_g * x_g + y_g * y_g + z_g * z_g);
    x_g /= norm;
    y_g /= norm;
    z_g /= norm;

    Quaternion a = AccelToOrientation(x_g, y_g, z_g);
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

  double yaw_rps() const {
    return current_gyro_.yaw_rps + filter_.state()(4);
  }

  double pitch_rps() const {
    return current_gyro_.pitch_rps + filter_.state()(5);
  }

  double roll_rps() const {
    return current_gyro_.roll_rps + filter_.state()(6);
  }

  Point3D gyro_bias_rps() const {
    return Point3D(filter_.state()(5, 0),
                   filter_.state()(4, 0),
                   -filter_.state()(6, 0));
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
        current_gyro_.roll_rps + result(6),
        current_gyro_.pitch_rps + result(5),
        current_gyro_.yaw_rps + result(4),
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
    Point3D rotation(
        current_gyro_.pitch_rps + s(5),
        current_gyro_.roll_rps + s(6),
        current_gyro_.yaw_rps + s(4));
    return (Eigen::Matrix<double, 1, 1>() << rotation.length()).finished();
  }

  static Quaternion AccelToOrientation(
      double x, double y, double z) {
    double roll = std::atan2(-x, z);
    double pitch = std::atan2(y, std::sqrt(x * x + z * z));

    return Quaternion::FromEuler(roll, pitch, 0.0);
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
      double yaw_rps, double pitch_rps, double roll_rps,
      double x_g, double y_g, double z_g) {
    current_gyro_ = Gyro(yaw_rps, pitch_rps, roll_rps);

    double norm = std::sqrt(x_g * x_g + y_g * y_g + z_g * z_g);

    x_g /= norm;
    y_g /= norm;
    z_g /= norm;

    if (!initialized_) {
      initialized_ = true;
      Quaternion start = AccelToOrientation(x_g, y_g, z_g);
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
        (Eigen::Matrix<double, 3, 1>() << x_g, y_g, z_g).finished(),
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

  struct Gyro {
    double yaw_rps;
    double pitch_rps;
    double roll_rps;

    Gyro(double yaw_rps, double pitch_rps, double roll_rps)
        : yaw_rps(yaw_rps), pitch_rps(pitch_rps), roll_rps(roll_rps) {}
    Gyro() : yaw_rps(0.), pitch_rps(0.), roll_rps(0.) {}
  };

  Gyro current_gyro_;
};
}
}
