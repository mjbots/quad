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

#include "mech/quadruped_2leg_walk.h"

#include "mech/quadruped_util.h"

namespace mjmech {
namespace mech {

namespace {
/// Return a good target location for each leg during the swing
/// phase.  We just aim to spend half the time reaching the idle
/// position and half the time going past it.
///
/// @p stance_time_s is how long we expect the leg to be in contact
/// with the ground.
std::vector<std::pair<int, base::Point3D>>
GetSwingTarget_R(const QuadrupedContext* context,
                 double stance_time_s) {

  auto* state = context->state;

  const auto& desired_w_LR = state->robot.desired_w_LR;
  const auto& desired_v_mm_s_R = state->robot.desired_v_mm_s_R;
  const double dt = 0.5 * stance_time_s;
  const auto& v_mm_s = desired_v_mm_s_R;

  const Sophus::SE3d pose_T2_T1(
      Sophus::SO3d(
          Eigen::AngleAxisd(dt * desired_w_LR.z(), Eigen::Vector3d::UnitZ())
          .toRotationMatrix()),
      v_mm_s * dt);

  std::vector<std::pair<int, base::Point3D>> result;

  for (const auto& leg : context->legs) {
    base::Point3D position_mm_R = pose_T2_T1 * leg.idle_R;
    result.push_back(std::make_pair(leg.leg, position_mm_R));
  }
  return result;
}
}

std::vector<QuadrupedCommand::Leg> Quadruped2LegWalk_R(
    QuadrupedContext* context,
    const std::vector<QuadrupedCommand::Leg>& old_legs_R) {
  auto* const state = context->state;
  const auto& config = context->config;

  context->UpdateCommandedRB();
  // We always have some legs on the ground, so can nominally always
  // accelerate.
  context->UpdateCommandedLR();

  auto legs_R = old_legs_R;

  // Update our phase.
  auto& ws = state->walk;
  const auto old_phase = ws.phase;
  auto& phase = ws.phase;
  phase = std::fmod(
      phase + config.period_s / config.walk.cycle_time_s, 1.0);
  if (phase < old_phase) {
    // We have wrapped around.
    if (state->robot.desired_v_mm_s_R.norm() == 0.0 &&
        state->robot.desired_w_LR.norm() == 0.0) {
      ws.idle_count++;
    } else {
      ws.idle_count = 0;
    }
  }

  // Check to see if we are in a step or not.
  const auto step_phase = [&]() -> std::optional<double> {
    if (phase > 0.0 && phase < config.walk.step_phase) {
        return phase / config.walk.step_phase;
    } else if (phase > 0.5 && phase < (0.5 + config.walk.step_phase)) {
      return (phase - 0.5) / config.walk.step_phase;
    }
    return {};
  }();

  const auto [ground_legs, step_legs] = [&]()
       -> std::pair<std::vector<int>, std::vector<int>> {
    if (!step_phase) { return {{0, 1, 2, 3}, {}}; }
    if (phase < 0.5) { return {{0, 3}, {1, 2}};}
    return {{1, 2}, {0, 3}};
  }();

  // All ground legs should be at the correct height and in stance.
  for (int ground_leg : ground_legs) {
    auto& leg_R = GetLeg_R(&legs_R, ground_leg);
    leg_R.stance = 1.0;
  }

  // Accumulate our various control times.
  const auto step_time_s =
      config.walk.cycle_time_s * config.walk.step_phase;

  const auto swing_targets_R = GetSwingTarget_R(
      context,
      (1.0 - config.walk.step_phase) * config.walk.cycle_time_s);

  // Move our legs that are in step.
  for (int step_leg_id : step_legs) {
    auto& leg_R = GetLeg_R(&legs_R, step_leg_id);
    auto& state_leg = ws.legs[step_leg_id];

    leg_R.landing = false;
    leg_R.kp_scale = {};
    leg_R.kd_scale = {};
    leg_R.stance = 0.0;

    if (!state_leg.in_flight) {
      // Configure our swing calculation.
      state_leg.target_mm_R = [&]() {
        for (const auto& pair : swing_targets_R) {
          if (pair.first == step_leg_id) { return pair.second; }
        }
        mjlib::base::AssertNotReached();
      }();

      context->swing_trajectory[step_leg_id] = SwingTrajectory(
          leg_R.position_mm, leg_R.velocity_mm_s,
          state_leg.target_mm_R,
          config.walk.lift_height_mm,
          config.walk.step.lift_lower_time,
          step_time_s);
      state_leg.in_flight = true;
    }

    const auto swing = context->swing_trajectory[step_leg_id].Advance(
        config.period_s, -state->robot.desired_v_mm_s_R -
        state->robot.desired_w_LR.cross(state_leg.target_mm_R));
    leg_R.position_mm = swing.position;
    leg_R.velocity_mm_s = swing.velocity_s;
  }

  // Reset the kp_scale of anything on the ground.
  for (int step_leg_id : ground_legs) {
    auto& leg_R = GetLeg_R(&legs_R, step_leg_id);
    leg_R.kp_scale = {};
    leg_R.kd_scale = {};

    auto& state_leg = ws.legs[step_leg_id];
    state_leg.in_flight = false;
  }

  // Advance the legs which are landing or on the ground.
  context->MoveLegsForLR(&legs_R);

  // Update the other legs.
  context->MoveLegsFixedSpeedZ(
      ground_legs, &legs_R,
      // we should already be basically at the right height,
      // so this velocity isn't that meaningful.
      config.rest.velocity_mm_s,
      config.stand_height_mm);

  context->UpdateLegsStanceForce(&legs_R, 0.0);

  return legs_R;
}

}
}
