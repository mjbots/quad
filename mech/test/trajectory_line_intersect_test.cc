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

#include "mech/trajectory_line_intersect.h"

#include <boost/test/auto_unit_test.hpp>

using mjmech::mech::TrajectoryLineIntersectTime;

BOOST_AUTO_TEST_CASE(TrajectoryLineIntersect1,
                     * boost::unit_test::tolerance(1e-3)) {
  auto DUT = TrajectoryLineIntersectTime;
  // Some straight line cases.
  BOOST_TEST(DUT({1, 0}, 0, {1, -1}, {1, 1}) == 1.0);
  BOOST_TEST(DUT({0.5, 0}, 0, {1, -1}, {1, 1}) == 2.0);
  BOOST_TEST(DUT({0, 1}, 0, {-1, 1}, {1, 1}) == 1.0);
  BOOST_TEST(DUT({0, 1}, 0, {1, 1}, {-1, 1}) == 1.0);
  BOOST_TEST(DUT({0, 1}, 0, {-1, 2}, {1, 2}) == 2.0);
  BOOST_TEST(DUT({0, 1}, 0, {-1, -2}, {1, -2}) == -2.0);
  BOOST_TEST(DUT({3, 4}, 0, {0, 10}, {20, 0}) == 1.818);

  // Now some with curvature.
  BOOST_TEST(DUT({1, 0}, 0.1, {1, -1}, {1, 1}) == 1.002);
  BOOST_TEST(DUT({1, 0}, 0.1, {1, 1}, {1, -1}) == 1.002);
  BOOST_TEST(DUT({1, 0}, -0.1, {1, -1}, {1, 1}) == 1.002);
  BOOST_TEST(DUT({1, 0}, 0.1, {-1, 0.02}, {1, 0.01}) == 0.5);
  BOOST_TEST(DUT({1, 0}, 0.1, {-1, 0.01}, {1, 0.02}) == -0.5);
  BOOST_TEST(DUT({2, 0}, 0.1, {-1, 0.01}, {1, 0.02}) == -0.3405);
  BOOST_TEST(DUT({2, 0}, -0.1, {-1, 0.01}, {1, 0.02}) ==
             (std::numeric_limits<double>::infinity()));

  BOOST_TEST(DUT({1, 0}, 0.1, {-1, 20}, {1, 20}) == 31.415);
  BOOST_TEST(DUT({1, 0}, 0.1, {-1, 20.1}, {1, 20.1}) ==
             (std::numeric_limits<double>::infinity()));

  BOOST_TEST(DUT({0, 1}, 0.1, {-1, 1}, {1, 1}) == 1.002);
  BOOST_TEST(DUT({0, 1}, -0.1, {-1, 1}, {1, 1}) == 1.002);
  BOOST_TEST(DUT({0, 2}, -0.1, {-1, 1}, {1, 1}) == 0.5002);
  BOOST_TEST(DUT({0, 2}, -0.2, {-1, 1}, {1, 1}) == 0.50084);
  BOOST_TEST(DUT({3, 4}, 0.001, {-1, 4}, {1, 4}) == 0.9996);
  BOOST_TEST(DUT({3, 4}, 0.001, {-1, -4}, {1, -4}) == -0.9996);
}
