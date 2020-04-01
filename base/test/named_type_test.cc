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

#include "base/named_type.h"

#include <boost/test/auto_unit_test.hpp>

namespace {
using SimpleType = mjmech::base::NamedType<int, struct SimpleTypeStruct>;

int SimpleTransformer(SimpleType v) {
  return v.get();
}
}

BOOST_AUTO_TEST_CASE(NamedTypeTest) {
  BOOST_TEST(SimpleTransformer(SimpleType(3)) == 3);
}
