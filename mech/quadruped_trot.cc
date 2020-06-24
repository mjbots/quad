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

#include "mech/quadruped_trot.h"

#include "mech/quadruped_util.h"
#include "mech/trajectory_line_intersect.h"

namespace mjmech {
namespace mech {

namespace {
using QC = QuadrupedCommand;
using Walk = QuadrupedState::Walk;
using VLeg = Walk::VLeg;

constexpr int kVlegMapping[][2] = {
  {0, 3}, {1, 2},
};

/// This object exists to hold a bunch of state variables and make it
/// relatively easy to split up the calculation into multiple pieces.
class WalkContext {
 public:
  WalkContext(QuadrupedContext* context)
      : context_(context),
        state_(context->state),
        config_(context->config),
        ws_(state_->walk),
        wc_(config_.walk) {}

  std::vector<QC::Leg> Run(const std::vector<QC::Leg>& old_legs_R) {
    context_->UpdateCommandedRB();

    auto legs_R = old_legs_R;

    UpdateGlobal();
    UpdateSwingTime(legs_R);
    MaybeLift(&legs_R);
    PropagateLegs(&legs_R);

    // For now, set all legs pd gains to the default.
    for (auto& leg_R : legs_R) {
      leg_R.kp_N_m = config_.default_kp_N_m;
      leg_R.kd_N_m_s = config_.default_kd_N_m_s;
    }

    return legs_R;
  }

  void UpdateGlobal() {
    if (all_stance()) {
      // We are allowed to accelerate when all legs are in stance.
      context_->UpdateCommandedR();
      ws_.stance_elapsed_s += config_.period_s;
    } else {
      ws_.stance_elapsed_s = 0.0;
    }
  }

  void UpdateSwingTime(const std::vector<QC::Leg>& legs_R) {
    for (int vleg_idx = 0; vleg_idx < 2; vleg_idx++) {
      const int leg1 = kVlegMapping[vleg_idx][0];
      const int leg2 = kVlegMapping[vleg_idx][1];

      Eigen::Vector3d p1_R = legs_R[leg1].position;
      Eigen::Vector3d p2_R = legs_R[leg2].position;
      // TODO: We actually need the leg positions in the L frame, not
      // the R frame.  This will mostly matter for walking on hills and
      // side slopes, but anytime we are tipped.  For now, we'll just
      // use R to avoid a dependence on the IMU which needs more
      // validation.
      ws_.vlegs[vleg_idx].remaining_s = TrajectoryLineIntersectTime(
          state_->robot.desired_R.v.head<2>(),
          state_->robot.desired_R.w.z(),
          p1_R.head<2>(),
          p2_R.head<2>());
    }
  }

  void MaybeLift(std::vector<QC::Leg>* legs_R) {
    if (!all_stance()) {
      return;
    }

    // Check to see if we are ready to lift.
    double biggest_remaining_s = -std::numeric_limits<double>::infinity();
    int biggest_vleg_idx = -1;
    for (int vleg_idx = 0; vleg_idx < 2; vleg_idx++) {
      const auto& vleg = ws_.vlegs[vleg_idx];
      if (vleg.remaining_s > biggest_remaining_s) {
        biggest_remaining_s = vleg.remaining_s;
        biggest_vleg_idx = vleg_idx;
      }
    }
    if (!((biggest_remaining_s < 0.5 * wc_.swing_time_s &&
           ws_.stance_elapsed_s > wc_.min_stance_s) ||
          (ws_.stance_elapsed_s > wc_.max_stance_s))) {
      return;
    }

    // If we have changed direction since the last swing, repeat the
    // same foot selection.
    if (state_->robot.desired_R.v.dot(ws_.last_swing_v_R) < 0.0) {
      ws_.next_step_vleg = (ws_.next_step_vleg + 1) % 2;
    }

    LiftVleg(legs_R, ws_.next_step_vleg);
    ws_.next_step_vleg = (ws_.next_step_vleg + 1) % 2;
  }

  void LiftVleg(std::vector<QC::Leg>* legs_R, int vleg_idx) {
    ws_.last_swing_v_R = state_->robot.desired_R.v;

    // Yes, we are ready to begin a lift.
    auto& vleg_to_lift = ws_.vlegs[vleg_idx];
    vleg_to_lift.swing_elapsed_s = 0.0;
    vleg_to_lift.mode = VLeg::Mode::kSwing;

    // Pick a target and initialize our swing calculators for each
    // leg about to lift.

    // We use our "expected next" velocity as the swing target to aim
    // for.  This assumes constant acceleration, and that the stance
    // to swing ratio works out accurately, but then gives a better
    // chance of being ready for the next cycle.
    const auto predicted_state_R = FilterCommand(
        {state_->robot.desired_R.v, state_->robot.desired_R.w},
        {context_->command->v_R, context_->command->w_R},
        config_.lr_acceleration,
        config_.lr_alpha_rad_s2,
        wc_.swing_time_s * wc_.stance_swing_ratio);

    PropagateLeg propagate(
        predicted_state_R.v,
        predicted_state_R.w,
        // We aim for half a swing period beyond the idle point
        // which would be sufficient if we had zero stance time.  We
        // then add more so that in steady state we have a non-zero
        // amount of time spent in stance (and some time to
        // accelerate/decelerate).
        -(0.5 + wc_.stance_swing_ratio * 1.0) * wc_.swing_time_s);

    for (int leg_idx : kVlegMapping[vleg_idx]) {
      auto& leg_R = GetLeg_R(legs_R, leg_idx);
      leg_R.stance = 0.0;

      const auto& config_leg = context_->GetLeg(leg_idx);

      const auto presult_R = propagate(config_leg.idle_R);
      ws_.legs[leg_idx].target_R = presult_R.position;

      context_->swing_trajectory[leg_idx] = SwingTrajectory(
          leg_R.position, leg_R.velocity,
          ws_.legs[leg_idx].target_R,
          wc_.lift_height,
          wc_.step.lift_lower_time,
          wc_.swing_time_s);
    }
  }

  void PropagateLegs(std::vector<QC::Leg>* legs_R) {
    // Update our current leg positions.
    PropagateLeg propagator(
        state_->robot.desired_R.v,
        state_->robot.desired_R.w,
        config_.period_s);
    for (int vleg_idx = 0; vleg_idx < 2; vleg_idx++) {
      auto& vleg = ws_.vlegs[vleg_idx];
      if (vleg.mode == VLeg::kSwing) {
        vleg.swing_elapsed_s += config_.period_s;
      }
      for (int leg_idx : kVlegMapping[vleg_idx]) {
        auto& leg_R = GetLeg_R(legs_R, leg_idx);
        switch (vleg.mode) {
          case VLeg::Mode::kStance: {
            const auto result_R = propagator(leg_R.position);
            leg_R.position = result_R.position;
            leg_R.velocity = result_R.velocity;
            // TODO: We should keep track of our current desired body
            // acceleration and feed it in here.
            leg_R.acceleration = base::Point3D();
            break;
          }
          case VLeg::Mode::kSwing: {
            const auto swing_R =
                context_->swing_trajectory[leg_idx].Advance(
                    config_.period_s,
                    -state_->robot.desired_R.v -
                    state_->robot.desired_R.w.cross(leg_R.position));
            leg_R.position = swing_R.position;
            leg_R.velocity = swing_R.velocity_s;
            leg_R.acceleration = swing_R.acceleration_s2;

            break;
          }
        }
      }
      if (vleg.swing_elapsed_s > wc_.swing_time_s) {
        vleg.mode = VLeg::kStance;
        vleg.swing_elapsed_s = 0.0;

        if (state_->robot.desired_R.v.norm() == 0.0 &&
            state_->robot.desired_R.w.norm() == 0.0) {
          ws_.idle_count++;
        } else {
          ws_.idle_count = 0;
        }

        for (int leg_idx : kVlegMapping[vleg_idx]) {
          auto& leg_R = GetLeg_R(legs_R, leg_idx);
          leg_R.stance = 1.0;
        }
      }
    }
  }

  bool all_stance() const {
    return std::count_if(
        std::begin(ws_.vlegs), std::end(ws_.vlegs),
        [&](const auto& vleg) {
          return vleg.mode == VLeg::Mode::kSwing;
        }) == 0;
  }

  QuadrupedContext* const context_;
  QuadrupedState* const state_;
  const QuadrupedConfig& config_;
  Walk& ws_;
  const QuadrupedConfig::Walk& wc_;
};
}

std::vector<QuadrupedCommand::Leg> QuadrupedTrot_R(
    QuadrupedContext* context,
    const std::vector<QuadrupedCommand::Leg>& old_legs_R) {
  WalkContext ctx(context);
  return ctx.Run(old_legs_R);
}

}
}
