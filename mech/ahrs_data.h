// Copyright 2015-2016 Josh Pieper, jjp@pobox.com.  All rights reserved.
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

namespace mjmech {
namespace mech {

struct AhrsData {
  boost::posix_time::ptime timestamp;

  enum State : int {
    kUninitialized,
    kInitializing,
    kOperational,
    kFault,
  };

  static std::map<State, const char*> StateMapper() {
    return std::map<State, const char*>{
      { kUninitialized, "kUninitialized" },
      { kInitializing, "kInitializing" },
      { kOperational, "kOperational" },
      { kFault, "kFault" },
    };
  };

  State state = kUninitialized;
  bool valid = false;
  base::Quaternion attitude;
  double yaw_deg = 0.0;
  double pitch_deg = 0.0;
  double roll_deg = 0.0;

  base::Point3D body_rate_dps;
  base::Point3D body_accel_mps2;
  base::Point3D world_accel_mps2;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(timestamp));
    a->Visit(MJ_ENUM(state, StateMapper));
    a->Visit(MJ_NVP(valid));
    a->Visit(MJ_NVP(attitude));
    a->Visit(MJ_NVP(yaw_deg));
    a->Visit(MJ_NVP(pitch_deg));
    a->Visit(MJ_NVP(roll_deg));
    a->Visit(MJ_NVP(body_rate_dps));
    a->Visit(MJ_NVP(body_accel_mps2));
    a->Visit(MJ_NVP(world_accel_mps2));
  }
};

}
}
