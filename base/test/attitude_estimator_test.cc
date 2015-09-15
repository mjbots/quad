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

#include "attitude_estimator.h"

#include <boost/test/auto_unit_test.hpp>

BOOST_AUTO_TEST_CASE(BasicAttitudeEstimator) {
  mjmech::base::AttitudeEstimator estimator(
      0.0008 * 0.0008,
      0.0512 * 0.0512,
      1.0 * 1.0,
      0.1 * 0.1,
      1e-3,
      1e-8);

  for (int i = 0; i < 1000; i++) {
    estimator.ProcessMeasurement(0.01,
                                 0.0, 0., 0.,
                                 0., 0.4794, 0.8776);
  }

  BOOST_CHECK_SMALL(estimator.pitch_rad() - 0.5, 1e-2);
}
