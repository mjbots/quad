// Copyright 2019 Josh Pieper, jjp@pobox.com.  All rights reserved.
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

#include <array>
#include <optional>
#include <vector>

#include "base/point3d.h"

namespace mjmech {
namespace mech {

struct QuadrupedCommand {
  enum Mode {
    // In this mode, all servos are powered off.
    kStopped = 0,

    // In this mode, all servos are set to zero velocity.  It is a
    // latched state.  The only valid transition from this state is to
    // kStopped.
    kFault = 1,

    // In this mode, all servos are set to zero velocity.  This is the
    // safest thing that can be done with no knowledge of the current
    // robot state.
    kZeroVelocity = 2,

    // In this mode, each joint is commanded individually with a
    // position, velocity, and torque.
    kJoint = 3,

    // In this mode, each leg is commanded individually with a
    // position, velocity, and force.
    kLeg = 4,

    kNumModes,
  };

  static std::map<Mode, const char*> ModeMapper() {
    return { {
        { kStopped, "stopped" },
        { kFault, "fault" },
        { kZeroVelocity, "zero_velocity" },
        { kJoint, "joint" },
        { kLeg, "leg" },
      }};
  }

  Mode mode = kStopped;

  struct Joint {
    int id = 0;
    bool power = false;
    double angle_deg = 0.0;
    double velocity_dps = 0.0;
    double feedforward_torque_Nm = 0.0;
    std::optional<double> kp_scale;
    std::optional<double> kd_scale;
    std::optional<double> max_torque_Nm;
    std::optional<double> stop_position_deg;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(id));
      a->Visit(MJ_NVP(power));
      a->Visit(MJ_NVP(angle_deg));
      a->Visit(MJ_NVP(velocity_dps));
      a->Visit(MJ_NVP(feedforward_torque_Nm));
      a->Visit(MJ_NVP(kp_scale));
      a->Visit(MJ_NVP(kd_scale));
      a->Visit(MJ_NVP(max_torque_Nm));
      a->Visit(MJ_NVP(stop_position_deg));
    }
  };

  // Only valid for kJoint mode.
  std::vector<Joint> joint;

  struct Leg {
    int leg_id = 0;
    bool power = false;
    base::Point3D position_mm;
    base::Point3D velocity_mm_s;
    base::Point3D force_N;
    std::optional<base::Point3D> kp_scale;
    std::optional<base::Point3D> kd_scale;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(leg_id));
      a->Visit(MJ_NVP(power));
      a->Visit(MJ_NVP(position_mm));
      a->Visit(MJ_NVP(velocity_mm_s));
      a->Visit(MJ_NVP(force_N));
      a->Visit(MJ_NVP(kp_scale));
      a->Visit(MJ_NVP(kd_scale));
    }
  };

  // Only valid for kLeg mode.
  std::vector<Leg> legs_B;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_ENUM(mode, ModeMapper));
    a->Visit(MJ_NVP(joint));
    a->Visit(MJ_NVP(legs_B));
  }
};

}
}
