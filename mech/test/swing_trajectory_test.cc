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

#include "mech/swing_trajectory.h"

#include <fmt/format.h>
#include <fmt/ostream.h>

#include <boost/test/auto_unit_test.hpp>

using DUT = mjmech::mech::SwingTrajectory;

namespace {
template <typename Vector>
void Compare(const Vector& a, const Vector& b) {
  BOOST_TEST_CONTEXT(fmt::format("({},{},{})==({},{},{})",
                                 a.x(), a.y(), a.z(),
                                 b.x(), b.y(), b.z())) {
    BOOST_TEST(a.x() == b.x());
    BOOST_TEST(a.y() == b.y());
    BOOST_TEST(a.z() == b.z());
  }
}
}

BOOST_AUTO_TEST_CASE(SwingTrajectoryTest,
                     * boost::unit_test::tolerance(1e-3)) {
  DUT dut({0, 0, 0}, {0, 0, 0}, {10, 0, 0}, 2.0, 0.1, 1.0);
  {
    const auto r = dut.Advance(0.0, {0., 0., 0.});
    Compare(r.position, Eigen::Vector3d(0, 0, 0));
    Compare(r.velocity_s, Eigen::Vector3d(0, 0, 0));
    Compare(r.acceleration_s2, Eigen::Vector3d(0, 0, -24));
  }
  {
    // Halfway through the lift cycle.
    const auto r = dut.Advance(0.05, {0., 0., 0.});
    Compare(r.position, Eigen::Vector3d(0, 0, -0.056));
    Compare(r.velocity_s, Eigen::Vector3d(0, 0, -2.16));
    Compare(r.acceleration_s2, Eigen::Vector3d(0, 0, -19.2));
  }
  {
    // All the way through the lift cycle.
    const auto r = dut.Advance(0.05, {0., 0., 0.});
    Compare(r.position, Eigen::Vector3d(0, 0, -0.208));
    Compare(r.velocity_s, Eigen::Vector3d(0, 0, -3.84));
    Compare(r.acceleration_s2, Eigen::Vector3d(75, 0, -14.4));
  }
  {
    // Halfway through the move cycle.
    const auto r = dut.Advance(0.4, {0., 0., 0.});
    Compare(r.position, Eigen::Vector3d(5.0, 0.0, -2.0));
    Compare(r.velocity_s, Eigen::Vector3d(18.75, 0.0, 0.0));
    Compare(r.acceleration_s2, Eigen::Vector3d(0.0, 0.0, 24.0));
  }
  {
    // To the end of the move cycle.
    const auto r = dut.Advance(0.4, {0., 0., 0.});
    Compare(r.position, Eigen::Vector3d(10.0, 0.0, -0.208));
    Compare(r.velocity_s, Eigen::Vector3d(0.0, 0.0, 3.84));
    Compare(r.acceleration_s2, Eigen::Vector3d(0.0, 0.0, -14.4));
  }
  {
    // Halway through the lower cycle.
    const auto r = dut.Advance(0.05, {0., 0., 0.});
    BOOST_TEST(r.phase == 0.95);
    Compare(r.position, Eigen::Vector3d(10.0, 0.0, -0.056));
    Compare(r.velocity_s, Eigen::Vector3d(0.0, 0.0, 2.16));
    Compare(r.acceleration_s2, Eigen::Vector3d(0.0, 0.0, -19.2));
  }
  {
    const auto r = dut.Advance(0.05, {0., 0., 0.});
    BOOST_TEST(r.phase == 1.0);
    Compare(r.position, Eigen::Vector3d(10.0, 0.0, 0.0));
    Compare(r.velocity_s, Eigen::Vector3d(0., 0., 0.));
    Compare(r.acceleration_s2, Eigen::Vector3d(0., 0., -24.0));
  }
}

BOOST_AUTO_TEST_CASE(SwingTrajectoryTest2,
                     * boost::unit_test::tolerance(1e-3)) {
  DUT dut({-2, -4, 1}, {-5, 0, 0}, {8, -5, 1}, 2.0, 0.1, 2.0);
  {
    const auto r = dut.Advance(0.0, {-10, 0, 0});
    BOOST_TEST(r.phase == 0.0);
    Compare(r.position, Eigen::Vector3d(-2, -4, 1));
    Compare(r.velocity_s, Eigen::Vector3d(-5, 0, 0));
    Compare(r.acceleration_s2, Eigen::Vector3d(25, 0, -12));
  }
  {
    // halfway through lift
    const auto r = dut.Advance(0.1, {-10, 0, 0});
    BOOST_TEST(r.phase == 0.05);
    Compare(r.position, Eigen::Vector3d(-2.375, -4, 0.944));
    Compare(r.velocity_s, Eigen::Vector3d(-2.5, 0, -1.08));
    Compare(r.acceleration_s2, Eigen::Vector3d(25, 0, -9.6));
  }
  {
    // all the way through lift
    const auto r = dut.Advance(0.1, {-10, 0, 0});
    BOOST_TEST(r.phase == 0.1);
    Compare(r.position, Eigen::Vector3d(-2.5, -4, 0.792));
    Compare(r.velocity_s, Eigen::Vector3d(0.0, 0.0, -1.92));
    Compare(r.acceleration_s2, Eigen::Vector3d(39.375, -3.75, -7.2));
  }
  {
    // halfway through move
    const auto r = dut.Advance(0.8, {-10, 0, 0});
    BOOST_TEST(r.phase == 0.5);
    Compare(r.position, Eigen::Vector3d(2.75, -4.5, -1));
    Compare(r.velocity_s, Eigen::Vector3d(9.844, -0.9375, 0.0));
    Compare(r.acceleration_s2, Eigen::Vector3d(0.0, 0.0, 12));
  }
  {
    // all the way through move
    const auto r = dut.Advance(0.8, {-10, 0, 0});
    BOOST_TEST(r.phase == 0.9);
    Compare(r.position, Eigen::Vector3d(8, -5, 0.792));
    Compare(r.velocity_s, Eigen::Vector3d(0, 0, 1.92));
    Compare(r.acceleration_s2, Eigen::Vector3d(-2, 0, -7.2));
  }
  {
    // halfway through lower
    const auto r = dut.Advance(0.1, {-10, 0, 0});
    BOOST_TEST(r.phase == 0.95);
    Compare(r.position, Eigen::Vector3d(7.5, -5, 0.944));
    Compare(r.velocity_s, Eigen::Vector3d(-5, 0, 1.08));
    Compare(r.acceleration_s2, Eigen::Vector3d(-2, 0, -9.6));
  }
  {
    // all the way done
    const auto r = dut.Advance(0.1, {-10, 0, 0});
    BOOST_TEST(r.phase == 1.0);
    Compare(r.position, Eigen::Vector3d(6.5, -5, 1.0));
    Compare(r.velocity_s, Eigen::Vector3d(-10, 0, 0));
    Compare(r.acceleration_s2, Eigen::Vector3d(-2, 0, -12));
  }
}
