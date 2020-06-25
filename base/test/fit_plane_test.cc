// Copyright 2020 Josh Pieper, jjp@pobox.com.
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

#include "base/fit_plane.h"

#include <boost/test/auto_unit_test.hpp>

using mjmech::base::FitPlane;

BOOST_AUTO_TEST_CASE(BasicFitPlane, * boost::unit_test::tolerance(1e-4)) {
  {
    std::vector<Eigen::Vector3d> points = {
      { -2, -2, -1 },
      { -2, 2, -1 },
      { 2, -2, 1 },
      { 2, 2, 1 },
    };

    const auto result = FitPlane(points);
    BOOST_TEST(std::abs(result.c - 0.0) < 0.001);
    BOOST_TEST(result.a == 0.5);
    BOOST_TEST(result.b == 0.0);
  }

  {
    std::vector<Eigen::Vector3d> points = {
      { -2, -2, 0 },
      { -2, 2, 2 },
      { 2, -2, 0 },
      { 2, 2, 2 },
    };

    const auto result = FitPlane(points);
    BOOST_TEST(result.c == 1.0);
    BOOST_TEST(result.a == 0.0);
    BOOST_TEST(result.b == 0.5);
  }
}
