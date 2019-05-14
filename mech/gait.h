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

#include "mjlib/base/visitor.h"

#include "base/common.h"
#include "base/tf.h"

#include "leg_ik.h"

namespace mjmech {
namespace mech {
struct Leg {
  enum class Mode {
    kStance,
      kSwing,
      kUnknown,
  };

  struct Config {
    base::Point3D mount_mm;
    base::Point3D idle_mm;
    boost::shared_ptr<IKSolver> leg_ik;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(mount_mm));
      a->Visit(MJ_NVP(idle_mm));
    }

    bool operator==(const Config&) const { return true; }
  };

  struct Result {
    base::Point3D point;
    Mode mode = Mode::kUnknown;
  };

  struct State {
    base::Point3D point;
    Mode mode = Mode::kUnknown;
    boost::shared_ptr<IKSolver> leg_ik;
    base::Frame* frame = nullptr;
  };
};

struct MechanicalConfig {
  std::vector<Leg::Config> leg_config;
  base::Point3D body_cog_mm;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(leg_config));
    a->Visit(MJ_NVP(body_cog_mm));
  }
};

struct Command {
  double translate_x_mm_s = 0;
  double translate_y_mm_s = 0;
  double rotate_deg_s = 0;
  double body_x_mm = 0;
  double body_y_mm = 0;
  double body_z_mm = 0;
  double body_pitch_deg = 0;
  double body_roll_deg = 0;
  double body_yaw_deg = 0;
  double lift_height_percent = 100;
  double time_rate_percent = 100;
  bool reset_phase = false;

  bool IsZero() const {
    return translate_x_mm_s == 0.0 &&
        translate_y_mm_s == 0.0 &&
        rotate_deg_s == 0.0;
  }

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(translate_x_mm_s));
    a->Visit(MJ_NVP(translate_y_mm_s));
    a->Visit(MJ_NVP(rotate_deg_s));
    a->Visit(MJ_NVP(body_x_mm));
    a->Visit(MJ_NVP(body_y_mm));
    a->Visit(MJ_NVP(body_z_mm));
    a->Visit(MJ_NVP(body_pitch_deg));
    a->Visit(MJ_NVP(body_roll_deg));
    a->Visit(MJ_NVP(body_yaw_deg));
    a->Visit(MJ_NVP(lift_height_percent));
    a->Visit(MJ_NVP(time_rate_percent));
    a->Visit(MJ_NVP(reset_phase));
  }
};

struct CommonState {
  base::Frame world_frame;
  base::Frame robot_frame;
  base::Frame body_frame;
  base::Frame cog_frame;

  CommonState() :
      world_frame(),
      robot_frame({}, {}, &world_frame),
      body_frame({}, {}, &robot_frame),
      cog_frame({}, {}, &body_frame) {}

  CommonState(const CommonState& rhs) : CommonState() {
    *this = rhs;
  }

  CommonState& operator=(const CommonState& rhs) {
    world_frame.transform = rhs.world_frame.transform;
    robot_frame.transform = rhs.robot_frame.transform;
    body_frame.transform = rhs.body_frame.transform;
    cog_frame.transform = rhs.cog_frame.transform;
    return *this;
  };

  void MakeShoulder(const Leg::Config& leg_config,
                    base::Frame* frame) const {
    // For now, we are assuming that shoulders face away from the y
    // axis.
    const double rotation_rad = (leg_config.mount_mm.x > 0.0) ?
        (0.5 * M_PI) : (-0.5 * M_PI);
    frame->transform = base::Transform(
        leg_config.mount_mm,
        base::Quaternion::FromEuler(0, 0, rotation_rad));
    frame->parent = &body_frame;
  }

};

struct JointCommand {
  struct Joint {
    int servo_number = 0;
    double angle_deg = 0.0;
    double torque_Nm = 0.0;
    double kp = 1.0;

    JointAngles::Joint ik_joint;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(servo_number));
      a->Visit(MJ_NVP(angle_deg));
      a->Visit(MJ_NVP(torque_Nm));
      a->Visit(MJ_NVP(kp));
      a->Visit(MJ_NVP(ik_joint));
    }

    Joint() {}
    Joint(int servo_number, double angle_deg, double torque_Nm, double kp)
        : servo_number(servo_number),
          angle_deg(angle_deg),
          torque_Nm(torque_Nm),
          kp(kp) {}

    bool operator==(const Joint& rhs) const {
      return servo_number == rhs.servo_number &&
          angle_deg == rhs.angle_deg &&
          torque_Nm == rhs.torque_Nm &&
          kp == rhs.kp;
    }
  };

  std::vector<Joint> joints;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(joints));
  }
};

class Gait : boost::noncopyable {
 public:
  virtual ~Gait() {};
  virtual JointCommand AdvancePhase(double delpha_phase) = 0;
  virtual JointCommand AdvanceTime(double delta_s) = 0;
  enum Result {
    kValid,
    kNotSupported,
  };
  virtual Result SetCommand(const Command&) = 0;
  virtual bool are_all_legs_stance() const = 0;
  virtual int zero_phase_count() const = 0;
};
}
}
