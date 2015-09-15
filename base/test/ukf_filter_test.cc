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

#include "ukf_filter.h"

#include <boost/test/auto_unit_test.hpp>

namespace {
typedef mjmech::base::UkfFilter<double, 3> UkfFilter;
}

BOOST_AUTO_TEST_CASE(BasicUkfFilter) {
  auto test_process = [](const UkfFilter::State& s,
                         double dt_s) -> UkfFilter::State {
    UkfFilter::State delta;
    delta(0, 0) = 0.;
    delta(1, 0) = s(0) * dt_s;
    delta(2, 0) = s(1) * dt_s + 0.5 * s(0) * dt_s * dt_s;
    return s + delta;
  };

  auto test_measurement = [](const UkfFilter::State& s) {
    Eigen::Matrix<double, 1, 1> r;
    r(0, 0) = s(2);
    return r;
  };

  UkfFilter::Covariance cov = UkfFilter::Covariance::Zero();
  cov(0, 0) = 1.0;
  cov(1, 1) = 2.0;
  cov(2, 2) = 3.0;

  UkfFilter::Covariance proc = UkfFilter::Covariance::Zero();
  proc(0, 0) = 0.1;
  proc(1, 1) = 0.1;
  proc(2, 2) = 0.1;
  UkfFilter dut(UkfFilter::State(0.2, 0.0, 0.0),
                cov,
                proc);

  Eigen::Matrix<double, 1, 1> meas;
  meas(0, 0) = 0.5;
  Eigen::Matrix<double, 1, 1> meas_noise;
  meas_noise(0, 0) = 2.0;

  for (int i = 0; i < 200; i++) {
    meas(0, 0) += 0.5;
    dut.UpdateState(0.1, test_process);
    dut.UpdateMeasurement(test_measurement, meas, meas_noise);
  }

  BOOST_CHECK_SMALL(dut.state()(2) - meas(0), 1e-2);
  BOOST_CHECK_SMALL(dut.state()(1) - 0.5 / 0.1, 1e-2);
  BOOST_CHECK(dut.covariance()(0, 0) > 0.0);
}
