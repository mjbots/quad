// Copyright 2019-2020 Josh Pieper, jjp@pobox.com.
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

#include "mech/vertical_line_frame.h"

#include <boost/test/auto_unit_test.hpp>

using mjmech::mech::FindVerticalLinePlaneIntersect;
auto Dut = FindVerticalLinePlaneIntersect;

using V3 = Eigen::Vector3d;

BOOST_AUTO_TEST_CASE(FindVerticalLinePlaneIntersectTest) {
  {
    auto project = [](const V3& query) {
      return (
          Dut(Sophus::SE3d(), V3(1, 4, 100), V3(0, 0, 1),
              query));
    };
    BOOST_TEST(project(V3(10, 10, 0)) == V3(10, 10, 100));
    BOOST_TEST(project(V3(12, 10, 0)) == V3(12, 10, 100));
    BOOST_TEST(project(V3(12, 10, 10)) == V3(12, 10, 100));
    BOOST_TEST(project(V3(12, 15, 10)) == V3(12, 15, 100));
  }
  {
    auto project = [](const V3& query) {
      return (
          Dut(Sophus::SE3d(), V3(0, 0, 10),
              V3(0.707, 0, 0.707), query));
    };
    BOOST_TEST(project(V3(0, 0, 0)) == V3(0, 0, 10));
    BOOST_TEST(project(V3(0, 5, 100)) == V3(0, 5, 10));
    BOOST_TEST((project(V3(2, 0, 0)) - V3(2, 0, 8)).norm() < 1e-5);
    BOOST_TEST((project(V3(-2, 0, 0)) - V3(-2, 0, 12)).norm() < 1e-5);
    BOOST_TEST((project(V3(2, 5, 0)) - V3(2, 5, 8)).norm() < 1e-5);
  }
  {
    auto project = [](const V3& query) {
      return (
          Dut(Sophus::SE3d(Sophus::SO3d(), Eigen::Vector3d(0, 0, 10)),
              V3(0, 0, 0), V3(0, 0, 1), query));
    };
    BOOST_TEST(project(V3(0, 0, 0)) == V3(0, 0, 10));
    BOOST_TEST(project(V3(3, 0, 0)) == V3(3, 0, 10));
    BOOST_TEST(project(V3(3, 5, 0)) == V3(3, 5, 10));
  }
  {
    auto project = [](const V3& query) {
      return (
          Dut(Sophus::SE3d(
                  Sophus::SO3d(
                      Eigen::AngleAxisd(0.25 * M_PI, Eigen::Vector3d::UnitX()).
                      toRotationMatrix()),
                  Eigen::Vector3d(0, 0, 10)),
              V3(0, 0, 0), V3(0, 0,1), query));
    };
    BOOST_TEST(project(V3(0, 0, 0)) == V3(0, 0, 10));
    BOOST_TEST(project(V3(4, 0, 0)) == V3(4, 0, 10));
    BOOST_TEST((project(V3(4, 2, 0)) - V3(4, 2, 12)).norm() < 1e-5);
    BOOST_TEST((project(V3(4, -2, 0)) - V3(4, -2, 8)).norm() < 1e-5);
  }
}
