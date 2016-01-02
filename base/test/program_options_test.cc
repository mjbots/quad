// Copyright 2016 Josh Pieper, jjp@pobox.com.  All rights reserved.
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

#include "base/program_options.h"

#include <boost/test/auto_unit_test.hpp>

using namespace mjmech;
namespace po = boost::program_options;

BOOST_AUTO_TEST_CASE(BasicProgramOptions) {
  po::options_description base;
  po::options_description sub;
  bool item1 = false;
  sub.add_options()
      ("item1", po::bool_switch(&item1), "item1 help")
      ;

  base::MergeProgramOptions(&sub, "sub.", &base);

  std::ostringstream ostr;
  ostr << base;
  std::string actual = ostr.str();
  BOOST_CHECK_EQUAL(actual, "  --sub.item1           item1 help\n");
}
