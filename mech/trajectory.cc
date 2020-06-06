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

#include "mech/trajectory.h"

namespace mjmech {
namespace mech {

TrajectoryState CalculateAccelerationLimitedTrajectory(
    const TrajectoryState& start,
    const base::Point3D& target_pose_l,
    double target_velocity_l_s,
    double max_acceleration_l_s2,
    double delta_s) {
  // This merely is a rough approximation of a controller.  There
  // exist closed form solutions which are optimal, such as described
  // in: http://arl.cs.utah.edu/pubs/ICRA2013-1.pdf "Kinodynamic RRT*:
  // Asymptotically Optimal Motion Planning for Robots with Linear
  // Dynamics", I'm just not going to bother with it for now.

  const base::Point3D error_l = target_pose_l - start.pose_l;
  const double error_norm_l = error_l.norm();

  const double deceleration_max_velocity_l_s =
      0.5 * (
          std::sqrt(2 * error_norm_l * max_acceleration_l_s2) +
          std::sqrt(2 * std::max(
                        0.0,
                        error_norm_l - (0.5 * start.velocity_l_s.norm() * delta_s) *
                        max_acceleration_l_s2)));

  // We first limit velocity in the scalar space based on how close we
  // are.  This isn't sound if we are currently speeding away?
  const double decel_limited_scalar_velocity_l_s =
      std::min(target_velocity_l_s, deceleration_max_velocity_l_s);
  const base::Point3D decel_limited_velocity_l_s =
      error_l.normalized() * decel_limited_scalar_velocity_l_s;

  const base::Point3D delta_velocity_l_s =
      decel_limited_velocity_l_s - start.velocity_l_s;
  const base::Point3D accel_limited_velocity_l_s = [&]() -> base::Point3D {
    if (delta_velocity_l_s.norm() < max_acceleration_l_s2 * delta_s) {
      return decel_limited_velocity_l_s;
    }

    // Nope, we need to scale this back.
    return (start.velocity_l_s + delta_velocity_l_s.normalized() *
            (max_acceleration_l_s2 * delta_s));
  }();

  // We need to prevent pose from crossing the plane on the target
  // point, so that we converge even with a high acceleration limit
  // and/or high delta_s.
  const base::Point3D next_pose_l =
      start.pose_l + accel_limited_velocity_l_s * delta_s;

  const auto dot = (next_pose_l - target_pose_l).dot(start.pose_l - target_pose_l);

  TrajectoryState result;
  const auto [pose_l, velocity_l_s] = [&]() {
    if (dot < 0.0) {
      // We crossed the line.  Just jump to the target and set zero velocity.
      return std::make_pair(target_pose_l, base::Point3D(0, 0, 0));
    }
    return std::make_pair(next_pose_l, accel_limited_velocity_l_s);
  }();
  result.pose_l = pose_l;
  result.velocity_l_s = velocity_l_s;
  result.acceleration_l_s2 =
      (accel_limited_velocity_l_s - start.velocity_l_s) /
      std::max(0.001, delta_s);

  return result;
}

}
}
