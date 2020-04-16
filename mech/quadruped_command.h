// Copyright 2019 Josh Pieper, jjp@pobox.com.
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
#include "base/sophus.h"

namespace mjmech {
namespace mech {

struct QuadrupedCommand {
  /// Higher values of priority take precedence.  A lower value
  /// command is only used in place of a higher value if the higher
  /// value is stale.
  int priority = 0;

  enum Mode {
    // This is a transient state that should not be commanded.  The
    // quadruped uses it to perform initialization functions.
    kConfiguring = 0,

    // In this mode, all servos are powered off.
    kStopped = 1,

    // In this mode, all servos are set to zero velocity.  It is a
    // latched state.  The only valid transition from this state is to
    // kStopped.
    kFault = 2,

    // In this mode, all servos are set to zero velocity.  This is the
    // safest thing that can be done with no knowledge of the current
    // robot state.
    kZeroVelocity = 3,

    // In this mode, each joint is commanded individually with a
    // position, velocity, and torque.
    kJoint = 4,

    // In this mode, each leg is commanded individually with a
    // position, velocity, and force.
    kLeg = 5,

    // This mode can be entered only from the kStopped or
    // kZeroVelocity state.  It positions the legs in an appropriate
    // location, then stands the robot up to the given surface frame
    // (S) pose.
    kStandUp = 6,

    // This is a simple mode that just lets the body to robot frame be
    // altered with no other leg movements.  It latches whatever foot
    // positions happen to be.  It can be entered from kStandUp:kDone.
    kRest = 7,

    // Jump one or more times.
    kJump = 8,

    // Walk
    kWalk = 9,

    kBackflip = 10,

    kNumModes,
  };

  static std::map<Mode, const char*> ModeMapper() {
    return { {
        { kConfiguring, "configuring" },
        { kStopped, "stopped" },
        { kFault, "fault" },
        { kZeroVelocity, "zero_velocity" },
        { kJoint, "joint" },
        { kLeg, "leg" },
        { kStandUp, "stand_up" },
        { kRest, "rest" },
        { kJump, "jump" },
        { kWalk, "walk" },
        { kBackflip, "backflip" },
      }};
  }

  Mode mode = kStopped;

  struct Joint {
    int id = 0;
    bool power = false;
    bool zero_velocity = false;
    double angle_deg = 0.0;
    double velocity_dps = 0.0;
    double torque_Nm = 0.0;
    std::optional<double> kp_scale;
    std::optional<double> kd_scale;
    std::optional<double> max_torque_Nm;
    std::optional<double> stop_angle_deg;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(id));
      a->Visit(MJ_NVP(power));
      a->Visit(MJ_NVP(zero_velocity));
      a->Visit(MJ_NVP(angle_deg));
      a->Visit(MJ_NVP(velocity_dps));
      a->Visit(MJ_NVP(torque_Nm));
      a->Visit(MJ_NVP(kp_scale));
      a->Visit(MJ_NVP(kd_scale));
      a->Visit(MJ_NVP(max_torque_Nm));
      a->Visit(MJ_NVP(stop_angle_deg));
    }
  };

  // Only valid for kJoint mode.
  std::vector<Joint> joints;

  struct Leg {
    int leg_id = 0;
    bool power = false;
    bool zero_velocity = false;
    bool landing = false;
    double stance = 1.0;
    base::Point3D position_mm;
    base::Point3D velocity_mm_s;
    base::Point3D force_N;
    std::optional<base::Point3D> kp_scale;
    std::optional<base::Point3D> kd_scale;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(leg_id));
      a->Visit(MJ_NVP(power));
      a->Visit(MJ_NVP(zero_velocity));
      a->Visit(MJ_NVP(landing));
      a->Visit(MJ_NVP(stance));
      a->Visit(MJ_NVP(position_mm));
      a->Visit(MJ_NVP(velocity_mm_s));
      a->Visit(MJ_NVP(force_N));
      a->Visit(MJ_NVP(kp_scale));
      a->Visit(MJ_NVP(kd_scale));
    }

    friend Leg operator*(const Sophus::SE3d& pose_mm, const Leg&);
  };

  // Only valid for kLeg mode.
  std::vector<Leg> legs_B;

  // Only valid for kJump.  These are latched at the start of a jump.
  struct Jump {
    double acceleration_mm_s2 = 0;
    bool repeat = false;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(acceleration_mm_s2));
      a->Visit(MJ_NVP(repeat));
    }
  };

  std::optional<Jump> jump;

  struct Walk {
    template <typename Archive>
    void Serialize(Archive* a) {
    }
  };

  std::optional<Walk> walk;

  struct Backflip {
    template <typename Archive>
    void Serialize(Archive* a) {
    }
  };

  std::optional<Backflip> backflip;

  /////////////////////////////////////////////
  // Things which are common to multiple modes.

  // Valid for kRest, kJump, kWalk, and...
  Sophus::SE3d pose_mm_RB;

  // Valid for kJump, kWalk, etc..
  //
  // These control the movement of the robot through the L frame,
  // i.e. the world.
  base::Point3D v_mm_s_R;  // Only the X and Y velocities are used
  base::Point3D w_LR;  // Only the Z axis rate is used

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(priority));
    a->Visit(MJ_ENUM(mode, ModeMapper));
    a->Visit(MJ_NVP(joints));
    a->Visit(MJ_NVP(legs_B));
    a->Visit(MJ_NVP(jump));
    a->Visit(MJ_NVP(walk));
    a->Visit(MJ_NVP(backflip));
    a->Visit(MJ_NVP(pose_mm_RB));
    a->Visit(MJ_NVP(v_mm_s_R));
    a->Visit(MJ_NVP(w_LR));
  }
};

inline QuadrupedCommand::Leg operator*(const Sophus::SE3d& pose_mm_AB,
                                       const QuadrupedCommand::Leg& leg_B) {
  QuadrupedCommand::Leg result_A = leg_B;

  result_A.position_mm = pose_mm_AB * leg_B.position_mm;
  result_A.velocity_mm_s = pose_mm_AB.so3() * leg_B.velocity_mm_s;
  result_A.force_N = pose_mm_AB.so3() * leg_B.force_N;
  result_A.kp_scale = leg_B.kp_scale ? (pose_mm_AB.so3() * *leg_B.kp_scale) :
      std::optional<base::Point3D>();
  result_A.kd_scale = leg_B.kd_scale ? (pose_mm_AB.so3() * *leg_B.kd_scale) :
      std::optional<base::Point3D>();

  return result_A;
}

}
}
