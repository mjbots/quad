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

#include <map>

#include <boost/signals2/signal.hpp>

#include "base/telemetry_log_registrar.h"
#include "base/telemetry_remote_debug_registrar.h"

namespace mjmech {
namespace base {
/// Maintain a local publish-subscribe model used for data objects.
class TelemetryRegistry : boost::noncopyable {
 public:
  TelemetryRegistry(TelemetryLog* log,
                    TelemetryRemoteDebugServer* debug)
      : log_(log), debug_(debug) {}

  /// Register a serializable object, and return a function object
  /// which when called will disseminate the
  /// object to any observers.
  template <typename DataObject>
  std::function<void (const DataObject*)>
  Register(const std::string& record_name) {
    // NOTE jpieper: Ideally this would return auto, and just let the
    // compiler sort out what type the lambda is.  But since C++11
    // doesn't support that yet, we return the type erasing
    // std::function.

    auto* ptr = new Concrete<DataObject>();
    auto result = [ptr](const DataObject* object) { ptr->signal(object); };

    log_.Register(record_name, &ptr->signal);
    debug_.Register(record_name, &ptr->signal);

    records_.insert(
        std::make_pair(
            record_name, std::move(std::unique_ptr<Base>(ptr))));

    return result;
  };

  template <typename DataObject>
  void Register(const std::string& record_name,
                boost::signals2::signal<void (const DataObject*)>* signal) {
    signal->connect(Register<DataObject>(record_name));
  }

 private:
  struct Base {
    virtual ~Base() {}
  };

  template <typename Serializable>
  struct Concrete : public Base {
    virtual ~Concrete() {}

    boost::signals2::signal<void (const Serializable*)> signal;
  };

  std::map<std::string, std::unique_ptr<Base> > records_;

  TelemetryLogRegistrar log_;
  TelemetryRemoteDebugRegistrar debug_;
};

}
}
