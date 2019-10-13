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

#include <sophus/se3.hpp>

#include "base/point3d.h"

namespace mech {

struct QuadrupedState {
  boost::posix_time::ptime timestamp;

  struct Link {
    // The topmost link is relative to the "Body" frame.  Each
    // subsequent link is relative to the previous.  The "child" frame
    // references the endpoint of this link.
    Sophus::SE3d pose_child_parent;

    // Each of the these velocities and torques is the raw value
    // reported by the actuator, and is not referenced to any
    // particular frame.
    double velocity_dps = 0.0;
    double torque_Nm = 0.0;
  };

  // Then the leg end-effector level.
  struct Leg {
    int leg = 0;
    Point3D position_mm;
    Point3D velocity_mm_s;
    Point3D force_N;
    bool stance = false;

    std::vector<Link> links;
  };

  std::vector<Leg> legs_B;

  // And finally, the robot level.
  struct Robot {
    Sophus::SO3d attitude_LB;
    Point3D v_mm_s_LB;  // velocity
    Point3D w_LB;  // angular rate
    Point3D cog_CB;  // relationship between CoG and body frame
  };

  Robot robot;
};

}
