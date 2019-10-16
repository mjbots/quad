// Copyright 2014-2019 Josh Pieper, jjp@pobox.com.  All rights reserved.
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

#include "sophus/se3.hpp"

#include "mjlib/base/visitor.h"

#include "base/point3d.h"

namespace mjmech {
namespace mech {

struct QuadrupedState {
  // The joint level.
  struct Joint {
    int id = 0;

    // These are the raw values reported by the actuator and are not
    // referenced to any particular frame.
    double angle_deg = 0.0;
    double velocity_dps = 0.0;
    double torque_Nm = 0.0;

    double temperature_C = 0.0;
    double voltage = 0.0;
    int32_t mode = 0;
    int32_t fault = 0;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(id));
      a->Visit(MJ_NVP(angle_deg));
      a->Visit(MJ_NVP(velocity_dps));
      a->Visit(MJ_NVP(torque_Nm));
      a->Visit(MJ_NVP(temperature_C));
      a->Visit(MJ_NVP(voltage));
      a->Visit(MJ_NVP(mode));
      a->Visit(MJ_NVP(fault));
    }
  };

  std::vector<Joint> joints;

  struct Link {
    // The topmost link is relative to the "Body" frame.  Each
    // subsequent link is relative to the previous.  The "child" frame
    // references the endpoint of this link.
    Sophus::SE3d pose_child_parent;

    // Each of the these velocities and torques is referenced to the
    // canonical frame for that joint.
    double angle_deg = 0.0;
    double velocity_dps = 0.0;
    double torque_Nm = 0.0;

    // Random diagnostics for this joint.
    int id = 0;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(pose_child_parent));
      a->Visit(MJ_NVP(angle_deg));
      a->Visit(MJ_NVP(velocity_dps));
      a->Visit(MJ_NVP(torque_Nm));
      a->Visit(MJ_NVP(id));
    }
  };

  // The leg end-effector level.
  struct Leg {
    int leg = 0;
    base::Point3D position_mm;
    base::Point3D velocity_mm_s;
    base::Point3D force_N;
    bool stance = false;

    std::vector<Link> links;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(leg));
      a->Visit(MJ_NVP(position_mm));
      a->Visit(MJ_NVP(velocity_mm_s));
      a->Visit(MJ_NVP(force_N));
      a->Visit(MJ_NVP(stance));
      a->Visit(MJ_NVP(links));
    }
  };

  std::vector<Leg> legs_B;

  // And finally, the robot level.
  struct Robot {
    Sophus::SO3d attitude_LB;
    base::Point3D v_mm_s_LB;  // velocity
    base::Point3D w_LB;  // angular rate
    base::Point3D cog_CB;  // relationship between CoG and body frame

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(attitude_LB));
      a->Visit(MJ_NVP(v_mm_s_LB));
      a->Visit(MJ_NVP(w_LB));
      a->Visit(MJ_NVP(cog_CB));
    }
  };

  Robot robot;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(joints));
    a->Visit(MJ_NVP(legs_B));
    a->Visit(MJ_NVP(robot));
  }
};

}
}
