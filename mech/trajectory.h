// Copyright 2020 Josh Pieper, jjp@pobox.com.  All rights reserved.
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

#include "base/point3d.h"

namespace mjmech {
namespace mech {

struct TrajectoryState {
  base::Point3D pose_l;
  base::Point3D velocity_l_s;
};

TrajectoryState CalculateAccelerationLimitedTrajectory(
    const TrajectoryState& start,
    const base::Point3D& target_l,
    double target_velocity_l_s,
    double max_acceleration_l_s2,
    double delta_s);

}
}
