// Copyright 2015 Josh Pieper, jjp@pobox.com.  All rights reserved.
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

#include "json_archive.h"

#include <boost/test/auto_unit_test.hpp>

namespace {
using namespace legtool;

struct SubStruct {
  bool predicate = true;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(LT_NVP(predicate));
  }
};

struct TestStruct {
  int v = 1;
  std::string b = "hello";
  double c = 1.35;
  SubStruct sub;
  std::vector<int> vecint = {1, 2, 3};
  std::array<SubStruct, 2> arraysub;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(LT_NVP(v));
    a->Visit(LT_NVP(b));
    a->Visit(LT_NVP(c));
    a->Visit(LT_NVP(sub));
    a->Visit(LT_NVP(vecint));
    a->Visit(LT_NVP(arraysub));
  }
};
}

BOOST_AUTO_TEST_CASE(BasicJsonWriteTest) {
  std::ostringstream ostr;
  TestStruct test_struct;
  JsonWriteArchive(ostr).Accept(&test_struct);

  std::string expected = R"XX({
  "v":1,
  "b":"hello",
  "c":1.35,
  "sub":{
    "predicate":true
  },
  "vecint":[
    1,
    2,
    3
  ],
  "arraysub":[
    {
      "predicate":true
    },
    {
      "predicate":true
    }
  ]
})XX";
  BOOST_CHECK_EQUAL(ostr.str(), expected);
}
