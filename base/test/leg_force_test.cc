// Copyright 2020 Josh Pieper, jjp@pobox.com.  All rights reserved.
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

#include "base/leg_force.h"

#include <boost/test/auto_unit_test.hpp>

using mjmech::base::OptimizeLegForce;

BOOST_AUTO_TEST_CASE(LegForceTest, * boost::unit_test::tolerance(1e-3)) {
  {
    const auto result = OptimizeLegForce({
        {-1., -1.},
        {-1., 1.},
        {1., -1.},
        {1., 1.},
            });

    BOOST_TEST_REQUIRE(result.size() == 4);
    BOOST_TEST(result[0] == 0.25);
    BOOST_TEST(result[1] == 0.25);
    BOOST_TEST(result[2] == 0.25);
    BOOST_TEST(result[3] == 0.25);
  }

  {
    const auto result = OptimizeLegForce({{-4, 0}, {2, 0}});
    BOOST_TEST_REQUIRE(result.size() == 2);
    BOOST_TEST(result[0] == 0.3333);
    BOOST_TEST(result[1] == 0.6667);
  }
  {
    // Something that requires a simple balance.
    const auto result = OptimizeLegForce({{-4, 1}, {2, 1}});
    BOOST_TEST_REQUIRE(result.size() == 2);
    BOOST_TEST(result[0] == 0.3333);
    BOOST_TEST(result[1] == 0.6667);
  }

  {
    // No matter what we do, the Y axis will be off balance.
    const auto result = OptimizeLegForce({{-4, 1}, {2, 0}});
    BOOST_TEST_REQUIRE(result.size() == 2);
    BOOST_TEST(result[0] == 0.088235);
    BOOST_TEST(result[1] == 0.911764);
  }

  {
    // No matter what we do, the Y axis will be even more off balance.
    const auto result = OptimizeLegForce({{-4, 4}, {2, 0}});
    BOOST_TEST_REQUIRE(result.size() == 2);
    BOOST_TEST(result[0] == 0.007335);
    BOOST_TEST(result[1] == 0.992665);
  }
}
