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

#include "base/sophus.h"

namespace mjmech {
namespace mech {

/// Find the intersection between a vertical line in frame A (the
/// point through query_A along the Z axis), and the plane in frame B
/// described by onplane_B and normal_B.  Return the point in frame
/// A.
inline Eigen::Vector3d FindVerticalLinePlaneIntersect(
    const Sophus::SE3d& frame_AB,
    const Eigen::Vector3d& onplane_B,
    const Eigen::Vector3d& normal_B,
    const Eigen::Vector3d& query_A) {
  // The formula for a plane is
  // (p - onplane) * normal = 0.
  //
  // We'll first convert the B frame things into the A frame, and then
  // solve the equation.
  Eigen::Vector3d onplane_A = frame_AB * onplane_B;
  Eigen::Vector3d normal_A = frame_AB * (onplane_B + normal_B) - onplane_A;

  // The equation expanded out looks like:
  // (qx - ox) * nx + (qy - oy) * ny + (z - oz) * nz = 0
  //
  // Solved for Z that is:
  // z = oz - ((qx - ox) * nx - (qy - oy) * ny) / nz

  // If the normal is 0.0, then we don't have a solution.
  if (normal_A.z() == 0.0) {
    return query_A;
  }

  const double z_A =
      onplane_A.z() -
      ((query_A.head<2>() - onplane_A.head<2>()).dot(
          normal_A.head<2>()) / normal_A.z());
  return Eigen::Vector3d(query_A.x(), query_A.y(), z_A);
}

}
}
