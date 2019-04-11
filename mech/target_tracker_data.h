// Copyright 2016-2019 Josh Pieper, jjp@pobox.com.  All rights reserved.
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

#include <optional>

#include <boost/date_time/posix_time/posix_time_types.hpp>

#include "mjlib/base/visitor.h"

#include "base/point3d.h"

namespace mjmech {
namespace mech {

struct TargetTrackerData {
  boost::posix_time::ptime timestamp;

  struct Target {
    base::Point3D center;
    std::vector<base::Point3D> corners;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(center));
      a->Visit(MJ_NVP(corners));
    }
  };

  std::optional<Target> target;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(timestamp));
    a->Visit(MJ_NVP(target));
  }
};

typedef boost::signals2::signal<
  void (const TargetTrackerData*)> TargetTrackerDataSignal;

}
}
