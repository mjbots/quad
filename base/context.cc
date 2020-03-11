// Copyright 2014-2020 Josh Pieper, jjp@pobox.com.  All rights reserved.
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

#include "context_full.h"

namespace mjmech {
namespace base {

Context::Context()
    : telemetry_log(std::make_unique<mjlib::telemetry::FileWriter>([]() {
          mjlib::telemetry::FileWriter::Options options;
          options.blocking = false;
          return options;
        }())),
      remote_debug(std::make_unique<TelemetryRemoteDebugServer>(executor)),
      telemetry_registry(std::make_unique<TelemetryRegistry>(
                             telemetry_log.get(), remote_debug.get())),
      factory(std::make_unique<mjlib::io::StreamFactory>(executor))
{
}

Context::~Context() {}

}
}
