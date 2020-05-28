// Copyright 2020 Josh Pieper, jjp@pobox.com.
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

#include <Eigen/Core>

namespace mjmech {
namespace mech {

/// Given a trajectory and a line segment, return the time required for
/// the trajectory to intersect the line.  negative may be returned if
/// the trajectory would have intersected the line in the past, and
/// infinity will be returned if the trajectory will never intersect
/// the line.
///
/// The trajectory is specified as a path starting from (0, 0) facing
/// positive X.
double TrajectoryLineIntersectTime(
    const Eigen::Vector2d& velocity,
    double omega,
    const Eigen::Vector2d& p1,
    const Eigen::Vector2d& p2);

}
}
