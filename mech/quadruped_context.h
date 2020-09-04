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
#include "mech/valid_leg_region.h"
#include "mech/vertical_line_frame.h"

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
    Sophus::SE3d pose_BG;
    Eigen::Vector3d pose_B_femur;
    MammalIk ik;

    base::Point3D stand_up_R;
    base::Point3D idle_R;
    MammalJoint resolved_stand_up_joints;
    double shoulder_clearance_deg = 0.0;

    Leg(const Config::Leg& config_in,
        const Config::StandUp& stand_up,
        double stand_height,
        double idle_x,
        double idle_y)
        : leg(config_in.leg),
          config(config_in),
          pose_BG(config_in.pose_BG),
          pose_B_femur(config_in.pose_BG * config_in.ik.shoulder.pose),
          ik(config_in.ik) {
      // The idle and standup poses assume a null RB transform
      const Sophus::SE3d pose_RB;
      const auto pose_RG = pose_RB * config.pose_BG;

      const base::Point3D pose_R = [&]() {
        auto result = stand_up.pose_R;
        if (config.pose_BG.translation().x() < 0.0) { result.x() *= -1; }
        if (config.pose_BG.translation().y() < 0.0) { result.y() *= -1; }
        return result;
      }();

      const base::Point3D pose_G = pose_RG.inverse() * pose_R;

      IkSolver::Effector effector_G;
      effector_G.pose = pose_G;
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
      shoulder_clearance_deg =
          ((config.pose_BG.translation().y() < 0.0) ? 1.0 : -1.0) *
          ((config.pose_BG.so3().unit_quaternion().z() == 0.0) ? 1.0 : -1.0) *
          stand_up.shoulder_clearance_deg;

      stand_up_R = pose_R;
      base::Point3D tf = config.pose_BG.translation();
      idle_R = base::Point3D(
          idle_x * ((tf.x() > 0.0) ? 1.0 : -1.0),
          idle_y * ((tf.y() > 0.0) ? 1.0 : -1.0),
          stand_height);
    }
  };

  QuadrupedContext(const QuadrupedConfig& config_in,
                   const QuadrupedCommand* command_in,
                   QuadrupedState* state_in)
      : config(config_in),
        command(command_in),
        state(state_in) {
    for (const auto& leg : config.legs) {
      legs.emplace_back(leg, config.stand_up, config.stand_height,
                        config.idle_x, config.idle_y);
    }

    // Determine a rough estimate of the valid region for each leg.

    // assume B == R .... this doesn't matter too much, we just need
    // to give a point "somewhere" in the valid G region.
    Sophus::SE3d tf_BR;
    for (size_t i = 0; i < legs.size(); i++) {
      valid_regions.emplace_back(
          legs[i].ik,
          legs[i].pose_BG.inverse() * tf_BR * legs[i].idle_R,
          config.walk.lift_height);
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

  QuadrupedState::Leg GetLegState_R(int id) const {
    return state->robot.frame_RB.pose * GetLegState_B(id);
  }

  const QuadrupedState::Joint& GetJointState(int id) const {
    for (const auto& joint : state->joints) {
      if (joint.id == id) { return joint; }
    }
    mjlib::base::AssertNotReached();
  }

  base::KinematicRelation LevelDesiredRB() const {
    const auto& robot = state->robot;

    // Project the M(0, 0, 0) point down until it intersects the T
    // plane, getting the result in the M frame.

    Sophus::SE3d tf_AM = robot.frame_AB.pose * robot.frame_MB.pose.inverse();

    const Sophus::SE3d tf_MB = robot.frame_MB.pose;
    Sophus::SE3d tf_MT = tf_AM.inverse() * robot.tf_TA.inverse();
    const Sophus::SE3d tf_BM = tf_MB.inverse();

    const Eigen::Vector3d com_on_ground_M =
        FindVerticalLinePlaneIntersect(
            tf_MT,
            Eigen::Vector3d(0, 0, 0),
            Eigen::Vector3d(0, 0, 1),
            Eigen::Vector3d(0, 0, 0));

    // Project that back into the B frame to get our offset.
    base::KinematicRelation result_RB;
    result_RB.pose.translation().head<2>() =
        -(tf_BM * com_on_ground_M).head<2>();

    return result_RB;
  }

  void UpdateCommandedR(double max_rate = 1.0) {
    const auto result_R = FilterCommand(
        {state->robot.desired_R.v, state->robot.desired_R.w},
        {command->v_R, command->w_R},
        max_rate * config.lr_acceleration,
        max_rate * config.lr_alpha_rad_s2,
        config.period_s);
    state->robot.desired_R.v = result_R.v;
    state->robot.desired_R.w = result_R.w;
  }

  void MoveLegsForR(std::vector<QC::Leg>* legs_R) {
    PropagateLeg propagator(state->robot.desired_R.v,
                            state->robot.desired_R.w,
                            config.period_s);

    for (auto& leg_R : *legs_R) {
      if (leg_R.stance == 0.0 && !leg_R.landing) { continue; }

      const auto result = propagator(leg_R.position);

      leg_R.position = result.position;

      // We don't want to change the Z velocity, but do want to force
      // the X and Y, since the R frame movement is the only thing
      // that should be happening for a leg in stance configuration.
      leg_R.velocity.head<2>() = result.velocity.head<2>();
    }
  }

  struct MoveOptions {
    std::optional<double> override_acceleration;
  };

  bool MoveLegsFixedSpeedZ(
      const std::vector<int>& leg_ids,
      std::vector<QC::Leg>* legs_R,
      double desired_velocity,
      double desired_height,
      const MoveOptions& move_options = MoveOptions()) const {
    std::vector<std::pair<int, base::Point3D>> desired_poses_R;

    for (int id : leg_ids) {
      const auto& leg_R = GetLeg_R(legs_R, id);
      base::Point3D pose_R = leg_R.position;
      pose_R.z() = desired_height;
      desired_poses_R.push_back(std::make_pair(leg_R.leg_id, pose_R));
    }

    return MoveLegsFixedSpeed(
        legs_R,
        desired_velocity,
        desired_poses_R,
        move_options,
        base::Point3D(0, 0, 1),
        base::Point3D(1, 1, 0));
  }

  bool MoveLegsFixedSpeed(
      std::vector<QC::Leg>* legs_R,
      double desired_velocity,
      const std::vector<std::pair<int, base::Point3D>>& command_pose_R,
      const MoveOptions& move_options = MoveOptions(),
      base::Point3D velocity_mask = base::Point3D(1., 1., 1),
      base::Point3D velocity_inverse_mask = base::Point3D(0., 0., 0.)) const {

    bool done = true;

    const double acceleration =
        move_options.override_acceleration.value_or(
            config.bounds.max_acceleration);

    // We do each leg independently.
    for (const auto& pair : command_pose_R) {
      auto& leg_R = GetLeg_R(legs_R, pair.first);

      TrajectoryState initial{leg_R.position, leg_R.velocity};
      const auto result = CalculateAccelerationLimitedTrajectory(
          initial, pair.second,
          desired_velocity, acceleration,
          config.period_s);

      leg_R.position = result.pose_l;
      leg_R.acceleration =
          velocity_inverse_mask.asDiagonal() * leg_R.acceleration +
          velocity_mask.asDiagonal() * result.acceleration_l_s2;
      leg_R.velocity =
          velocity_inverse_mask.asDiagonal() * leg_R.velocity  +
          velocity_mask.asDiagonal() * result.velocity_l_s;

      if ((leg_R.position - pair.second).norm() > 0.001) {
        done = false;
      }
    }

    return done;
  }

  void MoveLegsTargetTime(
      std::vector<QC::Leg>* legs_R,
      double remaining_s,
      const std::vector<std::pair<int, base::Point3D>>& command_pose_R) const {
    for (const auto& pair : command_pose_R) {
      auto& leg_R = GetLeg_R(legs_R, pair.first);

      // This only makes sense for things that are not on the ground.
      MJ_ASSERT(leg_R.stance == 0.0);

      const base::Point3D error = pair.second - leg_R.position;
      const double error_norm = error.norm();
      const double velocity = error_norm /
          std::max(config.period_s, remaining_s);

      // For now, we'll do this as just an infinite acceleration
      // profile.

      const double delta =
          std::min(
              error_norm,
              velocity * config.period_s);
      leg_R.position += error.normalized() * delta;
      leg_R.velocity = error.normalized() * velocity;
      // Since we are not in stance.
      leg_R.force_N = base::Point3D(0, 0, 0);
    }
  }

  const QuadrupedConfig& config;
  const QuadrupedCommand* const command;
  QuadrupedState* const state;
  std::deque<Leg> legs;

  std::array<SwingTrajectory, 4> swing_trajectory = {};
  std::vector<ValidLegRegion> valid_regions;
};

}
}
