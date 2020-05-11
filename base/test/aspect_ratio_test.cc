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

#include "base/aspect_ratio.h"

#include <boost/test/auto_unit_test.hpp>

using mjmech::base::MaintainAspectRatio;

BOOST_AUTO_TEST_CASE(AspectRatio) {
  {
    auto result = MaintainAspectRatio({4, 4}, {4, 4});
    BOOST_TEST(result.min() == Eigen::Vector2i(0, 0));
    BOOST_TEST(result.max() == Eigen::Vector2i(4, 4));
    BOOST_TEST(result.sizes() == Eigen::Vector2i(4, 4));
  }

  {
    auto result = MaintainAspectRatio({4, 4}, {10, 4});
    BOOST_TEST(result.min() == Eigen::Vector2i(3, 0));
    BOOST_TEST(result.sizes() == Eigen::Vector2i(4, 4));
  }

  {
    auto result = MaintainAspectRatio({4, 4}, {20, 8});
    BOOST_TEST(result.min() == Eigen::Vector2i(6, 0));
    BOOST_TEST(result.sizes() == Eigen::Vector2i(8, 8));
  }

  {
    auto result = MaintainAspectRatio({4, 4}, {8, 20});
    BOOST_TEST(result.min() == Eigen::Vector2i(0, 6));
    BOOST_TEST(result.sizes() == Eigen::Vector2i(8, 8));
  }

  {
    auto result = MaintainAspectRatio({5, 4}, {10, 10});
    BOOST_TEST(result.min() == Eigen::Vector2i(0, 1));
    BOOST_TEST(result.sizes() == Eigen::Vector2i(10, 8));
  }
}
