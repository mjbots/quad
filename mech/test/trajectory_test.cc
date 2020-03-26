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

#include "mech/trajectory.h"

#include <boost/test/auto_unit_test.hpp>

namespace base = mjmech::base;
namespace mech = mjmech::mech;
using mech::TrajectoryState;
using mech::CalculateAccelerationLimitedTrajectory;

BOOST_AUTO_TEST_CASE(BasicTrajectoryTest, *boost::unit_test::tolerance(1e-5)) {
  struct TestCase {
    base::Point3D initial_pose_l;
    base::Point3D initial_velocity_l_s;
    base::Point3D target_l;
    double target_velocity_l_s;
    double max_acceleration_l_s2;
    double delta_s;

    base::Point3D expected_pose_l;
    base::Point3D expected_velocity_l_s;
  };

  TestCase tests[] = {
    // The acceleration limit is high enough that it doesn't matter.
    { {}, {}, {10, 0, 0}, 2, 1000, 0.1,  {0.2, 0., 0.}, {2., 0., 0.}},

    // A lower acceleration limit.
    { {}, {}, {10, 0, 0}, 2, 10, 0.1,  {0.1, 0., 0.}, {1., 0., 0.}},

    // Reaching the end with a high acceleration limit.
    { {9.5, 0, 0}, {2, 0, 0}, {10, 0, 0}, 2, 1000, 0.1,
      {9.7, 0., 0.}, {2., 0., 0.} },

    // Reaching the end with a lower acceleration limit.
    { {9.5, 0, 0}, {2, 0, 0}, {10, 0, 0}, 2, 2, 0.1,
      {9.68, 0., 0.}, {1.8, 0., 0.} },

    // No acceleration in 2D.
    { {}, {}, {30, 40, 0}, 50, 1000, 0.1,  {3, 4, 0}, {30, 40, 0}},

    // No acceleration near the end in 2D.
    { {29.7, 39.7, 0}, {3, 4, 0}, {30, 40, 0}, 50, 10000, .1,
      {30.0, 40.0, 0}, {0, 0, 0}},

    // Acceleration limited at the start in 2D.
    { {}, {}, {30, 40, 0}, 50, 10, 0.1,   {.06, .08, 0}, {.6, .8, 0}},

    // Acceleration limited at the end in 2D.
    { {29.7, 39.6, 0}, {3, 4, 0}, {30, 40, 0}, 50, 10, .1,
      {29.94, 39.92, 0}, {2.4, 3.2, 0}},

    // A real test case.
    { {2.092, 0., 0.}, {-585.786, 0, 0}, {0, 0, 0}, 585.786, 40000.0, 0.0025,
      {0.877535, 0., 0.}, {-485.786, 0., 0.} },

    { {183.091, 0., 0.}, {-800., 0, 0}, {180, 0, 0}, 800, 40000, 0.025,
      {180.0, 0., 0.}, {0., 0., 0. }},
  };
  for (const auto& test : tests) {
    const auto result =
        CalculateAccelerationLimitedTrajectory(
            {test.initial_pose_l, test.initial_velocity_l_s},
            test.target_l,
            test.target_velocity_l_s,
            test.max_acceleration_l_s2,
            test.delta_s);
    BOOST_TEST(result.pose_l.x() == test.expected_pose_l.x());
    BOOST_TEST(result.pose_l.y() == test.expected_pose_l.y());
    BOOST_TEST(result.pose_l.z() == test.expected_pose_l.z());
    BOOST_TEST(result.velocity_l_s.x() == test.expected_velocity_l_s.x());
    BOOST_TEST(result.velocity_l_s.y() == test.expected_velocity_l_s.y());
    BOOST_TEST(result.velocity_l_s.z() == test.expected_velocity_l_s.z());

    // Loop until it converges and measure the count.
    auto current = result;
    int i = 0;
    for (; i < 1000; i++) {
      current = CalculateAccelerationLimitedTrajectory(
          current,
          test.target_l,
          test.target_velocity_l_s,
          test.max_acceleration_l_s2,
          0.1 * test.delta_s);
      if ((current.pose_l - test.target_l).norm() < 0.01) {
        break;
      }
    }
    BOOST_TEST(i < 1000);
  }
}
