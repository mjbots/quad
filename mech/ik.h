// Copyright 2014-2019 Josh Pieper, jjp@pobox.com.
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

#include <boost/noncopyable.hpp>

#include "mjlib/base/visitor.h"

#include "base/point3d.h"
#include "base/sophus.h"

namespace mjmech {
namespace mech {

class IkSolver : boost::noncopyable {
 public:
  struct Joint {
    int id = 0;

    // Angles, torques, and velocities are positive clockwise when
    // looking in the following directions:
    //   * shoulder: +x axis
    //   * femur: +y axis
    //   * tibia: +y axis
    double angle_deg = 0.0;
    double torque_Nm = 0.0;
    double velocity_dps = 0.0;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(id));
      a->Visit(MJ_NVP(angle_deg));
      a->Visit(MJ_NVP(torque_Nm));
      a->Visit(MJ_NVP(velocity_dps));
    }

    Joint& set_id(int v) {
      id = v;
      return *this;
    }

    Joint& set_angle_deg(double v) {
      angle_deg = v;
      return *this;
    }

    Joint& set_torque_Nm(double v) {
      torque_Nm = v;
      return *this;
    }

    Joint& set_velocity_dps(double v) {
      velocity_dps = v;
      return *this;
    }
  };

  using JointAngles = std::vector<Joint>;
  using InverseResult = std::optional<JointAngles>;

  // End effector positions are in the leg (G) frame.
  struct Effector {
    base::Point3D pose_mm;
    base::Point3D velocity_mm_s;
    base::Point3D force_N;

    friend Effector operator*(const Sophus::SE3d&, const Effector&);
  };

  virtual ~IkSolver() {}

  /// If @p current is present, then it will be used to calculate the
  /// individual joints required torque to achieve the given force
  /// request.
  virtual InverseResult Inverse(
      const Effector& effector_G,
      const std::optional<JointAngles>& current) const = 0;

  virtual Effector Forward_G(const JointAngles&) const = 0;
};

inline IkSolver::Effector operator*(const Sophus::SE3d& lhs_AB,
                                    const IkSolver::Effector& rhs_B) {
  auto result_A = rhs_B;
  result_A.pose_mm = lhs_AB * rhs_B.pose_mm;
  result_A.velocity_mm_s = lhs_AB.so3() * rhs_B.velocity_mm_s;
  result_A.force_N = lhs_AB.so3() * rhs_B.force_N;
  return result_A;
}

}
}
