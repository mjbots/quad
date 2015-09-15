// Copyright 2014 Josh Pieper, jjp@pobox.com.  All rights reserved.
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

#include <type_traits>

#include <Eigen/Core>
#include <Eigen/Cholesky>
#include <Eigen/Dense>

#include <boost/assert.hpp>

namespace mjmech {
namespace base {

/// An Unscented Kalman Filter, as described in "Optimal State
/// Estimation", by Dan Simon.
template <typename _Scalar, int _NumStates>
class UkfFilter {
 public:
  typedef Eigen::Matrix<_Scalar, _NumStates, 1> State;
  typedef Eigen::Matrix<_Scalar, _NumStates, _NumStates> Covariance;

  UkfFilter(const State& initial_state,
            const Covariance& initial_covariance,
            const Covariance& process_noise)
      : state_(initial_state),
        covariance_(initial_covariance),
        process_noise_(process_noise) {
  }

  const State& state() const { return state_; }
  State& state() { return state_; }
  const Covariance& covariance() const { return covariance_; }
  Covariance& covariance() { return covariance_; }

  template <typename ProcessFunction>
  void UpdateState(_Scalar dt_s, ProcessFunction process_function) {
    const int N = _NumStates * 2;

    State sigma_points[N];
    StoreSigmaPoints(sigma_points);

    // Equation 14.59
    State xhat[N];
    for (int i = 0; i < N; i++) {
      xhat[i] = process_function(sigma_points[i], dt_s);
    }

    // Equation 14.60
    State xhatminus = (1.0 / N) * ArraySum(xhat);

    // Equation 14.61

    Covariance Pminus = Covariance::Zero();
    for (int i = 0; i < N; i++) {
      State c = xhat[i] - xhatminus;
      Pminus += c * c.transpose();
    }
    Pminus *= (1.0 / N);
    Pminus += dt_s * process_noise_;

    for (size_t i = 0; i < _NumStates; i++) {
      BOOST_ASSERT(std::isfinite(xhatminus[i]));
    }

    state_ = xhatminus;
    covariance_ = ConditionCovariance(Pminus);
  }

  template <typename Array>
  auto ArraySum(const Array& array)
      -> typename std::remove_reference<decltype(*array)>::type {
    static_assert(sizeof(array) / sizeof(array[0]) > 0,
                  "array must be non-empty");

    auto result = array[0];
    for (int i = 1; i < (sizeof(array) / sizeof(*array)); i++) {
      result += array[i];
    }
    return result;
  }

  template <typename MeasurementFunction,
            typename Measurement,
            typename MeasurementNoise>
  void UpdateMeasurement(MeasurementFunction measurement_function,
                         Measurement measurement,
                         MeasurementNoise measurement_noise) {
    static_assert(Measurement::ColsAtCompileTime == 1,
                  "measurement must be column vector");

    // Equation 14.62
    const int N = _NumStates * 2;
    State sigma_points[N];
    StoreSigmaPoints(sigma_points);

    // Equation 14.63
    Measurement yhatin[N];
    for (int i = 0; i < N; i++) {
      yhatin[i] = measurement_function(sigma_points[i]);
    }

    // Equation 14.64
    Measurement yhat = (1.0 / N) * ArraySum(yhatin);

    // Equation 14.65
    typedef Eigen::Matrix<_Scalar,
                          Measurement::RowsAtCompileTime,
                          Measurement::RowsAtCompileTime> PyMatrix;
    PyMatrix Py = PyMatrix::Zero();
    for (int i = 0; i < N; i++) {
      Py += (yhatin[i] - yhat) * (yhatin[i] - yhat).transpose();
    }
    Py *= (1.0 / N);
    Py += measurement_noise;

    // Equation 14.66
    typedef Eigen::Matrix<_Scalar,
                          State::RowsAtCompileTime,
                          Measurement::RowsAtCompileTime> PxyMatrix;
    PxyMatrix Pxy = PxyMatrix::Zero();
    for (int i = 0; i < N; i++) {
      Pxy += (sigma_points[i] - state_) * (yhatin[i] - yhat).transpose();
    }
    Pxy *= (1.0 / N);

    // Equation 14.67
    typedef Eigen::Matrix<_Scalar,
                          State::RowsAtCompileTime,
                          Measurement::RowsAtCompileTime> KMatrix;
    KMatrix K = Pxy * Py.inverse();
    State xplus = state_ + K * (measurement - yhat);
    Covariance Pplus = covariance_ - ((K * Py) * K.transpose());

    for (size_t i = 0; i < _NumStates; i++) {
      BOOST_ASSERT(std::isfinite(xplus[i]));
    }

    state_ = xplus;
    covariance_ = ConditionCovariance(Pplus);
  }

  template <typename Array>
  void StoreSigmaPoints(Array& array) {
    _Scalar n = _NumStates;

    // Lower Cholesky decomposition to calculate the matrix square
    // root.
    Covariance delta = (n * covariance_).llt().matrixL();

    for (int i = 0; i < _NumStates; i++) {
      array[i] = state_ + delta.col(i);
      array[i + _NumStates] = state_ - delta.col(i);
    }
  }

  Covariance ConditionCovariance(const Covariance& P) {
    Covariance result = P;

    // For the result to be symmetric.
    result = 0.5 * (result + result.transpose());

    return result;
  }

 private:
  State state_;
  Covariance covariance_;
  Covariance process_noise_;
};

}
}
