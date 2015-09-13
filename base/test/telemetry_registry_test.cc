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

#include "telemetry_registry.h"

#include <boost/test/auto_unit_test.hpp>

#include "telemetry_archive.h"
#include "visitor.h"

using namespace mjmech::base;

namespace {
struct TestData {
  int8_t foo = 0;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(LT_NVP(foo));
  }
};

class TestRegistrar {
 public:
  template <typename T>
  void Register(const std::string& name,
                boost::signals2::signal<void (const T*)>* signal) {
    names_.push_back(name);
    signal->connect(std::bind(&TestRegistrar::HandleData<T>, this,
                              std::placeholders::_1));
  }

  template <typename T>
  void HandleData(const T* data) {
    data_.push_back(TelemetryWriteArchive<T>{}.Serialize(data));
  }

  std::vector<std::string> names_;
  std::list<std::string> data_;
};
}

BOOST_AUTO_TEST_CASE(TelemetryBasicTest) {
  TelemetryRegistry<TestRegistrar> registry;
  auto callable = registry.Register<TestData>("data1");

  // Our name should appear in the registrar's list of names.
  TestRegistrar* registrar = registry.registrar<TestRegistrar>();

  BOOST_REQUIRE_EQUAL(registrar->names_.size(), 1u);
  BOOST_CHECK_EQUAL(registrar->names_[0], "data1");
  BOOST_CHECK_EQUAL(registrar->data_.size(), 0u);

  TestData data;
  callable(&data);

  // Now some data should appear.
  BOOST_REQUIRE_EQUAL(registrar->names_.size(), 1u);
  BOOST_CHECK_EQUAL(registrar->data_.size(), 1u);
}
