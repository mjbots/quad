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

#include "base/bezier.h"

#include <Eigen/Core>

#include <boost/test/auto_unit_test.hpp>

template <typename T>
using DUT = mjmech::base::Bezier<T>;

BOOST_AUTO_TEST_CASE(BezierTest) {
  {
    DUT<double> dut(0.0, 1.0);
    BOOST_TEST(dut.position(0.0) == 0.0);
    BOOST_TEST(dut.velocity(0.0) == 0.0);
    BOOST_TEST(dut.acceleration(0.0) == 6.0);

    BOOST_TEST(dut.position(0.5) == 0.5);
    BOOST_TEST(dut.velocity(0.5) == 1.5);
    BOOST_TEST(dut.acceleration(0.5) == 0.0);

    BOOST_TEST(dut.position(1.0) == 1.0);
    BOOST_TEST(dut.velocity(1.0) == 0.0);
    BOOST_TEST(dut.acceleration(1.0) == -6.0);
  }

  {
    DUT<double> dut(-2, 8);
    BOOST_TEST(dut.position(0.0) == -2);
    BOOST_TEST(dut.velocity(0.0) == 0.0);
    BOOST_TEST(dut.acceleration(0.0) == 60.0);

    BOOST_TEST(dut.position(0.5) == 3.0);
    BOOST_TEST(dut.velocity(0.5) == 15);
    BOOST_TEST(dut.acceleration(0.5) == 0.0);

    BOOST_TEST(dut.position(1.0) == 8.0);
    BOOST_TEST(dut.velocity(1.0) == 0.0);
    BOOST_TEST(dut.acceleration(1.0) == -60.0);
  }
}

BOOST_AUTO_TEST_CASE(BezierEigen) {
  DUT<Eigen::Vector3d> dut(Eigen::Vector3d(0, 5, 10),
                           Eigen::Vector3d(1, 10, 20));
  BOOST_TEST(dut.position(0.0) == Eigen::Vector3d(0, 5, 10));
  BOOST_TEST(dut.velocity(0.0) == Eigen::Vector3d(0, 0, 0));
  BOOST_TEST(dut.acceleration(0.0) == Eigen::Vector3d(6, 30, 60));

  BOOST_TEST(dut.position(0.5) == Eigen::Vector3d(0.5, 7.5, 15));
  BOOST_TEST(dut.velocity(0.5) == Eigen::Vector3d(1.5, 7.5, 15));
  BOOST_TEST(dut.acceleration(0.5) == Eigen::Vector3d(0., 0., 0.));

  BOOST_TEST(dut.position(1.0) == Eigen::Vector3d(1, 10, 20));
  BOOST_TEST(dut.velocity(1.0) == Eigen::Vector3d(0, 0, 0));
  BOOST_TEST(dut.acceleration(1.0) == Eigen::Vector3d(-6, -30, -60));
}
