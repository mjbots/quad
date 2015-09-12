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

#pragma once

#include <boost/signals2/signal.hpp>

#include "telemetry_archive.h"
#include "telemetry_remote_debug_server.h"

namespace legtool {
class TelemetryRemoteDebugServer;

/// A registrar which supports emitting data over the network in a
/// human readable format for online diagnostics and debugging.
///
/// The implementation of this class is largely a pass-through, since
/// tuple's can't hold noncopyable things, we delegate to a shared_ptr
/// to something that is non-copyable.
class TelemetryRemoteDebugRegistrar {
 public:
  TelemetryRemoteDebugRegistrar(TelemetryRemoteDebugServer* server)
      : server_(server) {}

  template <typename T>
  void Register(const std::string& name,
                boost::signals2::signal<void (const T*)>* signal) {
    server_->Register(name, signal);
  }

  TelemetryRemoteDebugServer* const server_;
};
}
