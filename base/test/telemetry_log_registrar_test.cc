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

#include "telemetry_log_registrar.h"

#include <boost/test/auto_unit_test.hpp>

#include "telemetry_registry.h"
#include "visitor.h"

namespace {
struct TestData {
  uint16_t value = 0;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(value));
  }
};

using namespace mjmech::base;
}

BOOST_AUTO_TEST_CASE(TelemetryLogRegistrarTest) {
  // TelemetryLog log;
  // TelemetryRegistry<TelemetryLogRegistrar> registry(&log);
  // auto callable = registry.Register<TestData>("test1");

  // // This should have resulted in a schema being written.

}
