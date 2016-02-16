// Copyright 2016 Josh Pieper, jjp@pobox.com.  All rights reserved.
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

#include <boost/date_time/posix_time/posix_time_types.hpp>

#include "base/point3d.h"
#include "base/visitor.h"

namespace mjmech {
namespace mech {

struct TargetTrackerData {
  boost::posix_time::ptime timestamp;

  enum State {
    kIdle,
    kStarting,
    kTracking,
  };

  static std::map<State, const char*> StateMapper() {
    return std::map<State, const char*>{
      { kIdle, "kIdle" },
      { kStarting, "kStarting" },
      { kTracking, "kTracking" },
    };
  }

  State state = kIdle;

  base::Point3D initial;
  base::Point3D current;

  std::vector<base::Point3D> features;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(timestamp));
    a->Visit(MJ_ENUM(state, StateMapper));
    a->Visit(MJ_NVP(initial));
    a->Visit(MJ_NVP(current));
    a->Visit(MJ_NVP(features));
  }
};

typedef boost::signals2::signal<
  void (const TargetTrackerData*)> TargetTrackerDataSignal;

}
}
