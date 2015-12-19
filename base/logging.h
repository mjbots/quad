// Copyright 2015 Mikhail Afanasyev.  All rights reserved.
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

#include <log4cpp/Category.hh>

#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <boost/signals2/signal.hpp>

#include "visitor.h"

namespace boost {
namespace program_options {
class options_description;
}
}

namespace mjmech {
namespace base {

void AddLoggingOptions(boost::program_options::options_description*);

// Call after program_options has been notified, or if you do not support
// program options.
void InitLogging();

typedef log4cpp::Category& LogRef;
// Get a LogRef with a given name.
//  - there is a .-separated hierachy -- '-t cd' will enable 'cd.stats' logger
//  - set the first element to C++ file name (lowercase, _-separated)
LogRef GetLogInstance(const std::string& name);

LogRef GetSubLogger(LogRef parent, const std::string& name);


//class TelemetryRegistry;
//void WriteLogToTelemetryLog(TelemetryRegistry*);


// TODO theamk: ideally, all of the stuff below should be hidden, and a single
// function (with signature above) should be exposed. However,
// TelemetryRegsitry is way too templated for this to work, so we have to
// expose much more than we wanted to.
struct TextLogMessage {
  boost::posix_time::ptime timestamp;
  std::string thread;
  std::string priority;
  int priority_int;
  std::string ndc;
  std::string category;
  std::string message;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(timestamp));
    a->Visit(MJ_NVP(thread));
    a->Visit(MJ_NVP(priority));
    a->Visit(MJ_NVP(priority_int));
    a->Visit(MJ_NVP(ndc));
    a->Visit(MJ_NVP(category));
    a->Visit(MJ_NVP(message));
  }
};
typedef boost::signals2::signal<void (const TextLogMessage*)
                                > TextLogMessageSignal;

TextLogMessageSignal* GetLogMessageSignal();

}
}
