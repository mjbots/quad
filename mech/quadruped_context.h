// Copyright 2019-2020 Josh Pieper, jjp@pobox.com.
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

#include <deque>

#include <boost/noncopyable.hpp>

#include "mjlib/base/assert.h"

#include "mech/propagate_leg.h"
#include "mech/quadruped_command.h"
#include "mech/quadruped_config.h"
#include "mech/quadruped_state.h"
#include "mech/quadruped_util.h"
#include "mech/swing_trajectory.h"
#include "mech/trajectory.h"

namespace mjmech {
namespace mech {

struct QuadrupedContext : boost::noncopyable {
  using Config = QuadrupedConfig;
  using QC = QuadrupedCommand;

  struct MammalJoint {
    double shoulder_deg = 0.0;
    double femur_deg = 0.0;
    double tibia_deg = 0.0;
  };

  struct Leg {
    int leg = 0;
    Config::Leg config;
    Sophus::SE3d pose_mm_BG;
    Eigen::Vector3d pose_mm_B_femur;
    MammalIk ik;

    base::Point3D stand_up_R;
    base::Point3D idle_R;
    MammalJoint resolved_stand_up_joints;

    Leg(const Config::Leg& config_in,
        const Config::StandUp& stand_up,
        double stand_height_mm,
        double idle_x_mm,
        double idle_y_mm)
        : leg(config_in.leg),
          config(config_in),
          pose_mm_BG(config_in.pose_mm_BG),
          pose_mm_B_femur(config_in.pose_mm_BG * config_in.ik.shoulder.pose_mm),
          ik(config_in.ik) {
      // The idle and standup poses assume a null RB transform
      const Sophus::SE3d pose_mm_RB;
      const auto pose_mm_RG = pose_mm_RB * config.pose_mm_BG;

      const base::Point3D pose_mm_R = [&]() {
        auto result = stand_up.pose_mm_R;
        if (config.pose_mm_BG.translation().x() < 0.0) { result.x() *= -1; }
        if (config.pose_mm_BG.translation().y() < 0.0) { result.y() *= -1; }
        return result;
      }();

      const base::Point3D pose_mm_G = pose_mm_RG.inverse() * pose_mm_R;

      IkSolver::Effector effector_G;
      effector_G.pose_mm = pose_mm_G;
      const auto resolved = ik.Inverse(effector_G, {});
      auto get_resolved = [&](int id) {
        MJ_ASSERT(!!resolved);
        for (const auto& joint_angle : *resolved) {
          if (joint_angle.id == id) { return joint_angle.angle_deg; }
        }
        mjlib::base::AssertNotReached();
      };
      resolved_stand_up_joints.shoulder_deg = get_resolved(config.ik.shoulder.id);
      resolved_stand_up_joints.femur_deg = get_resolved(config.ik.femur.id);
      resolved_stand_up_joints.tibia_deg = get_resolved(config.ik.tibia.id);

      stand_up_R = pose_mm_R;
      base::Point3D tf = config.pose_mm_BG.translation();
      idle_R = base::Point3D(
          idle_x_mm * ((tf.x() > 0.0) ? 1.0 : -1.0),
          idle_y_mm * ((tf.y() > 0.0) ? 1.0 : -1.0),
          stand_height_mm);
    }
  };

  QuadrupedContext(const QuadrupedConfig& config_in,
                   const QuadrupedCommand* command_in,
                   QuadrupedState* state_in)
      : config(config_in),
        command(command_in),
        state(state_in) {
    for (const auto& leg : config.legs) {
      legs.emplace_back(leg, config.stand_up, config.stand_height_mm,
                        config.idle_x_mm, config.idle_y_mm);
    }
  }

  const Leg& GetLeg(int id) const {
    for (auto& leg : legs) {
      if (leg.leg == id) { return leg; }
    }
    mjlib::base::AssertNotReached();
  }

  const QuadrupedState::Leg& GetLegState_B(int id) const {
    for (const auto& leg_B : state->legs_B) {
      if (leg_B.leg == id) { return leg_B; }
    }
    mjlib::base::AssertNotReached();
  }

  void UpdateCommandedRB() {
    Sophus::SE3d command_mm_RB =
        command->pose_mm_RB * config.command_offset_mm_RB;
    const base::Point3D translation =
        command_mm_RB.translation() -
        state->robot.pose_mm_RB.translation();
    state->robot.pose_mm_RB.translation() +=
        config.rb_filter_constant_Hz * config.period_s * translation;
    state->robot.pose_mm_RB.so3() =
        Sophus::SO3d(
            state->robot.pose_mm_RB.so3().unit_quaternion().slerp(
                config.rb_filter_constant_Hz * config.period_s,
                command_mm_RB.so3().unit_quaternion()));
  }

  void UpdateCommandedLR() {
    const auto result_R = FilterCommand(
        {state->robot.desired_v_mm_s_R, state->robot.desired_w_LR},
        {command->v_mm_s_R, command->w_LR},
        config.lr_acceleration_mm_s2,
        config.lr_alpha_rad_s2,
        config.period_s);
    state->robot.desired_v_mm_s_R = result_R.v_mm_s;
    state->robot.desired_w_LR = result_R.w;
  }

  void MoveLegsForLR(std::vector<QC::Leg>* legs_R) {
    PropagateLeg propagator(state->robot.desired_v_mm_s_R,
                            state->robot.desired_w_LR,
                            config.period_s);

    for (auto& leg_R : *legs_R) {
      if (leg_R.stance == 0.0 && !leg_R.landing) { continue; }

      const auto result = propagator(leg_R.position_mm);

      leg_R.position_mm = result.position_mm;

      // We don't want to change the Z velocity, but do want to force
      // the X and Y, since the LR frame movement is the only thing
      // that should be happening for a leg in stance configuration.
      leg_R.velocity_mm_s.head<2>() = result.velocity_mm_s.head<2>();
    }
  }

  bool MoveLegsFixedSpeedZ(
      const std::vector<int>& leg_ids,
      std::vector<QC::Leg>* legs_R,
      double desired_velocity_mm_s,
      double desired_height_mm) const {
    std::vector<std::pair<int, base::Point3D>> desired_poses_mm_R;

    for (int id : leg_ids) {
      const auto& leg_R = GetLeg_R(legs_R, id);
      base::Point3D pose_mm_R = leg_R.position_mm;
      pose_mm_R.z() = desired_height_mm;
      desired_poses_mm_R.push_back(std::make_pair(leg_R.leg_id, pose_mm_R));
    }

    return MoveLegsFixedSpeed(
        legs_R,
        desired_velocity_mm_s,
        desired_poses_mm_R,
        base::Point3D(0, 0, 1),
        base::Point3D(1, 1, 0));
  }

  bool MoveLegsFixedSpeed(
      std::vector<QC::Leg>* legs_R,
      double desired_velocity_mm_s,
      const std::vector<std::pair<int, base::Point3D>>& command_pose_mm_R,
      base::Point3D velocity_mask = base::Point3D(1., 1., 1),
      base::Point3D velocity_inverse_mask = base::Point3D(0., 0., 0.)) const {

    bool done = true;

    // We do each leg independently.
    for (const auto& pair : command_pose_mm_R) {
      auto& leg_R = GetLeg_R(legs_R, pair.first);

      TrajectoryState initial{leg_R.position_mm, leg_R.velocity_mm_s};
      const auto result = CalculateAccelerationLimitedTrajectory(
          initial, pair.second, desired_velocity_mm_s,
          config.bounds.max_acceleration_mm_s2,
          config.period_s);

      leg_R.position_mm = result.pose_l;
      leg_R.velocity_mm_s =
          velocity_inverse_mask.asDiagonal() * leg_R.velocity_mm_s  +
          velocity_mask.asDiagonal() * result.velocity_l_s;

      if ((leg_R.position_mm - pair.second).norm() > 1.0) {
        done = false;
      }
    }

    return done;
  }

  void MoveLegsTargetTime(
      std::vector<QC::Leg>* legs_R,
      double remaining_s,
      const std::vector<std::pair<int, base::Point3D>>& command_pose_mm_R) const {
    for (const auto& pair : command_pose_mm_R) {
      auto& leg_R = GetLeg_R(legs_R, pair.first);

      // This only makes sense for things that are not on the ground.
      MJ_ASSERT(leg_R.stance == 0.0);

      const base::Point3D error_mm = pair.second - leg_R.position_mm;
      const double error_norm_mm = error_mm.norm();
      const double velocity_mm_s = error_norm_mm /
          std::max(config.period_s, remaining_s);

      // For now, we'll do this as just an infinite acceleration
      // profile.

      const double delta_mm =
          std::min(
              error_norm_mm,
              velocity_mm_s * config.period_s);
      leg_R.position_mm += error_mm.normalized() * delta_mm;
      leg_R.velocity_mm_s =
          error_mm.normalized() * velocity_mm_s;
      // Since we are not in stance.
      leg_R.force_N = base::Point3D(0, 0, 0);
    }
  }

  void UpdateLegsStanceForce(
      std::vector<QC::Leg>* legs_R,
      double extra_z_N) const {
    const double stance_legs = [&]() {
      double result = 0.0;
      for (const auto& leg_R : *legs_R) {
        result += leg_R.stance;
      }
      return result;
    }();

    for (auto& leg_R : *legs_R) {
      const double gravity_N =
          leg_R.stance ?
          config.mass_kg * base::kGravity :
          0.0;
      const double force_z_N =
          stance_legs == 0.0 ?
          0.0 :
          leg_R.stance * gravity_N / stance_legs + extra_z_N;
      leg_R.force_N = base::Point3D(0, 0, force_z_N);
    }
  }

  const QuadrupedConfig& config;
  const QuadrupedCommand* const command;
  QuadrupedState* const state;
  std::deque<Leg> legs;

  std::array<SwingTrajectory, 4> swing_trajectory = {};
};

}
}
