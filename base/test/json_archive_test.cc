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

#include "base/json_archive.h"

#include <boost/test/auto_unit_test.hpp>

namespace {
using namespace mjmech::base;

struct SubStruct {
  bool predicate = true;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(predicate));
  }
};

struct TestStruct {
  int v = 1;
  std::string b = "hello";
  double c = 1.35;
  SubStruct sub;
  std::vector<int> vecint = {1, 2, 3};
  std::array<SubStruct, 2> arraysub;

  enum TestEnum {
    kFun,
    kStuff,
  } test_enum = kStuff;;

  static std::map<TestEnum, const char*> TestEnumMapper() {
    return std::map<TestEnum, const char*>{
      { kFun, "kFun" },
      { kStuff, "kStuff" },
    };
  }

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(v));
    a->Visit(MJ_NVP(b));
    a->Visit(MJ_NVP(c));
    a->Visit(MJ_NVP(sub));
    a->Visit(MJ_NVP(vecint));
    a->Visit(MJ_NVP(arraysub));
    a->Visit(MJ_ENUM(test_enum, TestEnumMapper));
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
  ],
  "test_enum":"kStuff"
})XX";
  BOOST_CHECK_EQUAL(ostr.str(), expected);
}
