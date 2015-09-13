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

#include "telemetry_archive.h"

#include <iostream>

#include <boost/test/auto_unit_test.hpp>

#include "telemetry_util.h"
#include "visitor.h"

using namespace mjmech::base;

namespace {
struct SubTest1 {
  uint32_t value_u32 = 3;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(LT_NVP(value_u32));
  }
};

struct Test1 {
  uint8_t value_u8 = 1;
  int8_t value_i8 = -1;
  std::pair<int16_t, uint16_t> value_pair = {};
  std::array<uint16_t, 6> value_array = {};
  std::vector<int32_t> value_vector;
  SubTest1 value_sub;
  boost::optional<SubTest1> value_optional;
  int32_t value_i32 = 6;
  uint64_t value_u64 = 7;
  int64_t value_i64 = -8;
  float value_f32 = 9.7;
  double value_f64 = 10.3;
  std::string value_str = "message";
  boost::posix_time::ptime value_time;
  std::vector<std::vector<int32_t> > value_vecvec = { { 1, 2 }, { 3} };
  std::vector<SubTest1> value_vecobj = { SubTest1(), SubTest1() };

  template <typename Archive>
  void Serialize(Archive* a ) {
    a->Visit(LT_NVP(value_u8));
    a->Visit(LT_NVP(value_i8));
    a->Visit(LT_NVP(value_pair));
    a->Visit(LT_NVP(value_array));
    a->Visit(LT_NVP(value_vector));
    a->Visit(LT_NVP(value_sub));
    a->Visit(LT_NVP(value_optional));
    a->Visit(LT_NVP(value_i32));
    a->Visit(LT_NVP(value_u64));
    a->Visit(LT_NVP(value_i64));
    a->Visit(LT_NVP(value_f32));
    a->Visit(LT_NVP(value_f64));
    a->Visit(LT_NVP(value_str));
    a->Visit(LT_NVP(value_time));
    a->Visit(LT_NVP(value_vecvec));
    a->Visit(LT_NVP(value_vecobj));
  }
};
}

BOOST_AUTO_TEST_CASE(TelemetryArchiveSchemaTest) {
  TelemetryWriteArchive<Test1> archive;
  std::istringstream istr(archive.schema());
  std::ostringstream repr;
  TelemetryReadStream<> stream(istr);

  TelemetrySchemaReader(stream, nullptr, repr).Read();
  std::string expected = R"XX([SchemaFlags]: 00000000;
{
  value_u8: kUInt8;
  value_i8: kInt8;
  value_pair: kPair<
    : kInt16
    : kUInt16
    >;
  value_array: kArray[6]
    : kUInt16;
  value_vector: kVector
    : kInt32;
  value_sub: kObject
    {
      value_u32: kUInt32;
    };
  value_optional: kOptional
    : kObject
      {
        value_u32: kUInt32;
      };
  value_i32: kInt32;
  value_u64: kUInt64;
  value_i64: kInt64;
  value_f32: kFloat32;
  value_f64: kFloat64;
  value_str: kString;
  value_time: kPtime;
  value_vecvec: kVector
    : kVector
      : kInt32;
  value_vecobj: kVector
    : kObject
      {
        value_u32: kUInt32;
      };
}
)XX";
  BOOST_CHECK_EQUAL(repr.str(), expected);
}

BOOST_AUTO_TEST_CASE(TelemetryArchiveDataTest) {
  TelemetryWriteArchive<Test1> archive;
  std::istringstream istr(archive.schema());
  std::ostringstream repr;
  TelemetryReadStream<> stream(istr);

  Test1 data;
  std::string result = archive.Serialize(&data);

  std::istringstream data_istr(result);
  TelemetryReadStream<> data_stream(data_istr);
  TelemetrySchemaReader(stream, &data_stream, repr).Read();
  std::string expected = R"XX([SchemaFlags]: 00000000;
{
  value_u8: kUInt8 = 1;
  value_i8: kInt8 = -1;
  value_pair: kPair<
    : kInt16 = 0
    : kUInt16 = 0
    >;
  value_array: kArray[6]
    : kUInt16 = [0,0,0,0,0,0];
  value_vector: kVector
    : kInt32 = [];
  value_sub: kObject
    {
      value_u32: kUInt32 = 3;
    };
  value_optional: kOptional
    : kObject
      {
        value_u32: kUInt32;
      } = [];
  value_i32: kInt32 = 6;
  value_u64: kUInt64 = 7;
  value_i64: kInt64 = -8;
  value_f32: kFloat32 = 9.7;
  value_f64: kFloat64 = 10.3;
  value_str: kString = "message";
  value_time: kPtime = not-a-date-time;
  value_vecvec: kVector
    : kVector
      : kInt32 = [[1,2],[3]];
  value_vecobj: kVector
    : kObject
      {
        value_u32: kUInt32;
      } = [
      {3},
      {3}];
}
)XX";
  BOOST_CHECK_EQUAL(repr.str(), expected);
}
