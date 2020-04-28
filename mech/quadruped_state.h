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

    std::vector<Link> links;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(leg));
      a->Visit(MJ_NVP(position_mm));
      a->Visit(MJ_NVP(velocity_mm_s));
      a->Visit(MJ_NVP(force_N));
      a->Visit(MJ_NVP(links));
    }

    friend Leg operator*(const Sophus::SE3d&, const Leg& rhs);
  };

  std::vector<Leg> legs_B;

  // And finally, the robot level.
  struct Robot {
    Sophus::SE3d pose_mm_LR;
    Sophus::SE3d pose_mm_RB;

    base::Point3D desired_v_mm_s_R;
    base::Point3D desired_w_LR;

    base::Point3D v_mm_s_LB;  // velocity
    base::Point3D w_LB;  // angular rate

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(pose_mm_LR));
      a->Visit(MJ_NVP(pose_mm_RB));

      a->Visit(MJ_NVP(desired_v_mm_s_R));
      a->Visit(MJ_NVP(desired_w_LR));

      a->Visit(MJ_NVP(v_mm_s_LB));
      a->Visit(MJ_NVP(w_LB));
    }
  };

  Robot robot;

  // Now the state associated with various control modes.  Each mode's
  // data is only valid while that mode is active.

  struct StandUp {
    enum Mode {
      kPrepositioning,
      kStanding,
      kDone,
    };

    static inline std::map<Mode, const char*> ModeMapper() {
      return {
        { kPrepositioning, "prepositioning" },
        { kStanding, "standing" },
        { kDone, "done" },
      };
    };

    Mode mode = kPrepositioning;

    struct Leg {
      int leg = 0;
      base::Point3D pose_mm_R;
      base::Point3D target_mm_R;

      template <typename Archive>
      void Serialize(Archive* a) {
        a->Visit(MJ_NVP(leg));
        a->Visit(MJ_NVP(pose_mm_R));
        a->Visit(MJ_NVP(target_mm_R));
      }
    };

    std::vector<Leg> legs;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_ENUM(mode, ModeMapper));
      a->Visit(MJ_NVP(legs));
    }
  };

  StandUp stand_up;

  struct Rest {
    bool done = false;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(done));
    }
  };

  Rest rest;

  struct Jump {
    enum Mode {
      kLowering = 0,
      kPushing = 1,
      kRetracting = 2,
      kFalling = 3,
      kLanding = 4,
      kDone = 5,
    };

    static inline std::map<Mode, const char*> ModeMapper() {
      return {
        { kLowering, "lowering" },
        { kPushing, "pushing" },
        { kRetracting, "retracting" },
        { kFalling, "falling" },
        { kLanding, "landing" },
        { kDone, "done" },
      };
    }

    Mode mode = kLowering;
    double velocity_mm_s = 0.0;
    double acceleration_mm_s2 = 0.0;

    QuadrupedCommand::Jump command;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_ENUM(mode, ModeMapper));
      a->Visit(MJ_NVP(velocity_mm_s));
      a->Visit(MJ_NVP(acceleration_mm_s2));
      a->Visit(MJ_NVP(command));
    }
  };

  Jump jump;

  struct Walk {
    double phase = 0.0;
    double moving_target_remaining_s = 0.0;
    int idle_count = 0;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(phase));
      a->Visit(MJ_NVP(moving_target_remaining_s));
      a->Visit(MJ_NVP(idle_count));
    }
  };

  Walk walk;

  struct Backflip {
    enum Mode {
      kLowering = 0,
      kFrontPush = 1,
      kBackPush = 2,
      kFlight = 3,
      kDone = 4,
    };

    static inline std::map<Mode, const char*> ModeMapper() {
      return {
        { kLowering, "lowering" },
        { kFrontPush, "front_push" },
        { kBackPush, "back_push" },
        { kFlight, "flight" },
        { kDone, "done" },
      };
    }

    Mode mode = kLowering;
    double pitch_deg = 0.0;
    double pitch_rate_dps = 0.0;
    double pitch_accel_dps2 = 0.0;
    double velocity_mm_s = 0.0;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_ENUM(mode, ModeMapper));
      a->Visit(MJ_NVP(pitch_deg));
      a->Visit(MJ_NVP(pitch_rate_dps));
      a->Visit(MJ_NVP(pitch_accel_dps2));
      a->Visit(MJ_NVP(velocity_mm_s));
    }
  };

  Backflip backflip;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(joints));
    a->Visit(MJ_NVP(legs_B));
    a->Visit(MJ_NVP(robot));

    a->Visit(MJ_NVP(stand_up));
    a->Visit(MJ_NVP(rest));
    a->Visit(MJ_NVP(jump));
    a->Visit(MJ_NVP(walk));
    a->Visit(MJ_NVP(backflip));
  }
};

inline QuadrupedState::Leg operator*(const Sophus::SE3d& pose_AB,
                                     const QuadrupedState::Leg& rhs_B) {
  auto result_A = rhs_B;
  result_A.position_mm = pose_AB * rhs_B.position_mm;
  result_A.velocity_mm_s = pose_AB.so3() * rhs_B.velocity_mm_s;
  result_A.force_N = pose_AB.so3() * rhs_B.force_N;
  return result_A;
}

}
}
