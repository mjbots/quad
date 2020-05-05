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

#include "mech/expo_map.h"

#include <boost/test/auto_unit_test.hpp>

namespace {
using Dut = mjmech::mech::ExpoMap;
}

BOOST_AUTO_TEST_CASE(ExpoMapTest, * boost::unit_test::tolerance(1e-5)) {
  Dut dut;
  BOOST_TEST(dut(0.0) == 0.0);
  BOOST_TEST(dut(0.02) == 0.0);
  BOOST_TEST(dut(0.05) == 0.0);
  BOOST_TEST(dut(0.175) == 0.05);
  BOOST_TEST(dut(0.30) == 0.10);
  BOOST_TEST(dut(0.65) == 0.55);
  BOOST_TEST(dut(1.00) == 1.0);
  BOOST_TEST(dut(-0.05) == 0.0);
  BOOST_TEST(dut(-0.175) == -0.05);
  BOOST_TEST(dut(-0.30) == -0.10);
  BOOST_TEST(dut(-0.65) == -0.55);
  BOOST_TEST(dut(-1.00) == -1.0);
}
