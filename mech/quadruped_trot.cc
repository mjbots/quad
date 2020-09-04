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

QuadrupedState::Walk::Trot CalculateTrot(
    const QuadrupedConfig& config,
    double max_travel_dist, double speed) {
  MJ_ASSERT(speed >= 0.0);
  MJ_ASSERT(max_travel_dist >= 0.0);

  // From trot_timing.py
  const auto& cw = config.walk;

  QuadrupedState::Walk::Trot r;

  const double max_speed =
      max_travel_dist / (cw.min_swing_time_s - 2 * cw.max_flight_time_s);
  r.max_speed = max_speed;

  if (speed < ((0.5 * max_travel_dist) /
               (cw.max_twovleg_time_s +
                0.5 * cw.max_swing_time_s))) {
    // Phase A
    r.swing_time = r.onevleg_time = cw.max_swing_time_s;
    r.twovleg_time = cw.max_twovleg_time_s;
    r.speed = speed;
    return r;
  }

  if (speed < (max_travel_dist / cw.max_swing_time_s)) {
    // Phase B
    r.swing_time = r.onevleg_time = cw.max_swing_time_s;
    r.twovleg_time = 0.5 * max_travel_dist / speed - 0.5 * cw.max_swing_time_s;
    r.speed = speed;
    return r;
  }

  if (speed < (max_travel_dist / (cw.max_swing_time_s - 2 * cw.max_flight_time_s))) {
    // Phase C
    r.swing_time = cw.max_swing_time_s;
    r.twovleg_time = 0.0;
    r.flight_time = 0.5 * r.swing_time - max_travel_dist / (2 * speed);
    r.onevleg_time = r.swing_time - 2 * r.flight_time;
    r.speed = speed;
    return r;
  }

  if (speed < max_speed) {
    // Phase D
    r.flight_time = cw.max_flight_time_s;
    r.twovleg_time = 0.0;
    r.swing_time = max_travel_dist / speed + 2 * r.flight_time;
    r.onevleg_time = r.swing_time - 2 * r.flight_time;
    r.speed = speed;
    return r;
  }

  // Phase E: max speed
  r.swing_time = cw.min_swing_time_s;
  r.flight_time = cw.max_flight_time_s;
  r.twovleg_time = 0;
  r.onevleg_time = r.swing_time - 2 * r.flight_time;
  r.speed = max_speed;
  return r;
}

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

  TrotResult Run(const std::vector<QC::Leg>& old_legs_R) {
    auto legs_R = old_legs_R;

    UpdateGlobal();
    UpdateSwingTime(legs_R);
    UpdateInvalidTime(legs_R);
    UpdateTravelDistance();
    UpdateTrot();
    MaybeLift(&legs_R);
    PropagateLegs(&legs_R);

    // For now, set all legs pd gains to the default.
    for (auto& leg_R : legs_R) {
      leg_R.kp_N_m = config_.default_kp_N_m;
      leg_R.kd_N_m_s = config_.default_kd_N_m_s;
    }

    TrotResult result;
    result.legs_R = std::move(legs_R);
    result.desired_RB = context_->LevelDesiredRB();
    return result;
  }

  void UpdateGlobal() {
    const int stance_vlegs = count_stance();
    if (stance_vlegs >= 1) {
      // We are allowed to accelerate when at least one vleg is down.
      context_->UpdateCommandedR(
          stance_vlegs == 2 ? 1.0 : config_.walk.onevleg_accel);
      const auto max_speed = std::max(1.0, ws_.trot.max_speed);
      if (state_->robot.desired_R.v.norm() > max_speed) {
        state_->robot.desired_R.v =
            state_->robot.desired_R.v.normalized() * max_speed;
      }
    }
    if (all_stance()) {
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
      // The R frame is adjusted automatically to keep the CoM over
      // the balance point, so we can just do our math here in the R
      // frame.
      ws_.vlegs[vleg_idx].remaining_s = TrajectoryLineIntersectTime(
          state_->robot.desired_R.v.head<2>(),
          state_->robot.desired_R.w.z(),
          p1_R.head<2>(),
          p2_R.head<2>());
    }
  }

  void UpdateInvalidTime(const std::vector<QC::Leg>& legs_R) {
    for (const auto& leg_R : legs_R) {
      const auto id = leg_R.leg_id;
      const auto& leg_config = context_->GetLeg(id);
      const base::Point3D p_B =
          state_->robot.frame_RB.pose.inverse() * leg_R.position;

      // Assume a zero RB frame omega and velocity.
      MJ_ASSERT(state_->robot.frame_RB.v.norm() == 0.0);
      MJ_ASSERT(state_->robot.frame_RB.w.norm() == 0.0);
      const base::Point3D v_B =
          state_->robot.frame_RB.pose.inverse().so3() * leg_R.velocity;

      // For now, we'll ignore desired_R.  If we wanted to use it,
      // we'd need to ignore the current leg_R.velocity command and
      // instead project the desired_R velocity and rotation onto the
      // leg point.
      const base::Point3D p_G = leg_config.pose_BG.inverse() * p_B;
      const base::Point3D v_G = leg_config.pose_BG.inverse().so3() * v_B;
      const double omega_G = 0.0;

      ws_.legs[id].invalid_time_s =
          context_->valid_regions[id].TimeToLeave_G(
              p_G.head<2>(), v_G.head<2>(), omega_G);
    }

    for (int vleg_idx = 0; vleg_idx < 2; vleg_idx++) {
      const int leg1 = kVlegMapping[vleg_idx][0];
      const int leg2 = kVlegMapping[vleg_idx][1];

      ws_.vlegs[vleg_idx].invalid_time_s =
          std::min(ws_.legs[leg1].invalid_time_s,
                   ws_.legs[leg2].invalid_time_s);
    }
  }

  void UpdateTravelDistance() {
    for (int id = 0; id < 4; id++) {
      const auto& config_leg = context_->GetLeg(id);
      const base::Point3D p_R = config_leg.idle_R;
      const base::Point3D p_B = state_->robot.frame_RB.pose.inverse() * p_R;
      const base::Point3D p_G = config_leg.pose_BG.inverse() * p_B;
      auto find_time = [&](double sign) {
        const base::Point3D v =
            sign * state_->robot.desired_R.v +
            // TODO: Do I need to apply sign to the cross product too?  Or
            // to where it is used down below?
            p_R.cross(state_->robot.desired_R.w);
        constexpr double kBothSides = 2.0;
        return kBothSides * std::abs(
            v.norm() * context_->valid_regions[id].TimeToLeave_G(
                p_G.head<2>(), v.head<2>(),
                -state_->robot.desired_R.w.z()));
      };
      ws_.legs[id].travel_distance = std::min(find_time(-1.0), find_time(1.0));
    }

    double new_travel_distance = std::numeric_limits<double>::infinity();
    for (const auto& leg : ws_.legs) {
      if (leg.travel_distance < new_travel_distance) {
        new_travel_distance = leg.travel_distance;
      }
    }

    if (std::isfinite(new_travel_distance)) {
      ws_.travel_distance = new_travel_distance;
    }
  }

  void UpdateTrot() {
    ws_.trot = CalculateTrot(
        config_,
        config_.walk.travel_ratio * ws_.travel_distance,
        state_->robot.desired_R.v.norm());
  }

  void MaybeLift(std::vector<QC::Leg>* legs_R) {
    const auto num_stance = count_stance();
    if (num_stance == 0) { return; }

    if (num_stance <= 1 && ws_.trot.flight_time <= 0.0) {
      return;
    }

    // This lambda returns an unset optional if no leg should be
    // lifted.  -1 if the "next" leg should be lifted, and otherwise a
    // vleg index if that specific leg should be lifted.
    const auto vleg_to_lift = [&]() -> std::optional<int> {
      double biggest_remaining_s = -std::numeric_limits<double>::infinity();
      int biggest_vleg_idx = -1;

      double smallest_invalid_s = std::numeric_limits<double>::infinity();
      int smallest_vleg_idx = -1;
      double total_invalid_s = 0.0;

      for (int vleg_idx = 0; vleg_idx < 2; vleg_idx++) {
        const auto& vleg = ws_.vlegs[vleg_idx];
        if (vleg.mode == VLeg::Mode::kSwing) { continue; }

        if (vleg.remaining_s > biggest_remaining_s) {
          biggest_remaining_s = vleg.remaining_s;
          biggest_vleg_idx = vleg_idx;
        }

        if (vleg.invalid_time_s < smallest_invalid_s) {
          smallest_invalid_s = vleg.invalid_time_s;
          smallest_vleg_idx = vleg_idx;
        }
        total_invalid_s += vleg.invalid_time_s;
      }

      if (ws_.trot.twovleg_time >= config_.walk.min_invalid_time_s &&
          std::isfinite(smallest_invalid_s) &&
          std::isfinite(total_invalid_s)) {
        const double other_invalid_s = total_invalid_s - smallest_invalid_s;
        const bool individual_leg_invalid = (
            smallest_invalid_s < config_.walk.min_invalid_time_s);
        const bool other_leg_invalid = (
            other_invalid_s <
            (ws_.trot.swing_time + config_.walk.min_invalid_time_s));
        if (individual_leg_invalid || other_leg_invalid) {
          // We're going to run out of travel before a full swing could
          // complete.  Thus pick up a leg no matter what the balance
          // says.  If the two legs are relatively close as to when they
          // will be invalid, then just let the normal leg selection
          // process work here.
          if (std::abs(ws_.vlegs[0].invalid_time_s -
                       ws_.vlegs[1].invalid_time_s) < 0.75 * ws_.trot.swing_time) {
            return -1;
          }
          return smallest_vleg_idx;
        }
      }

      const bool max_stance_exceeded = (
          ws_.stance_elapsed_s > wc_.max_stance_s);
      if (max_stance_exceeded) {
        // Just lift the next thing.
        return -1;
      }

      if (ws_.trot.flight_time <= 0.0) {
        if (!std::isfinite(biggest_remaining_s)) {
          if (ws_.stance_elapsed_s >= ws_.trot.twovleg_time) {
            return -1;
          }
        } else if (biggest_remaining_s < 0.5 * ws_.trot.swing_time) {
          return -1;
        }
      } else {
        // Nominally, when a stance leg is 1/2 of one_leg time away
        // from the balance point, then it gets lifted.  We add a
        // corrective term to keep the two vlegs in the correct phase.
        // If they are in the correct phase, then the alternating vleg
        // should be in flight and have flight_time_s remaining in its
        // swing.
        for (int vleg_idx = 0; vleg_idx < 2; vleg_idx++) {
          const auto& vleg = ws_.vlegs[vleg_idx];
          if (vleg.mode != VLeg::Mode::kStance) { continue; }

          const auto alternate_leg_swing_remaining = [&]() {
            const int alternate_vleg_idx = (vleg_idx + 1) % 2;
            const auto& alternate_vleg = ws_.vlegs[alternate_vleg_idx];
            if (alternate_vleg.mode == VLeg::Mode::kStance) { return 0.0; }
            return ws_.trot.swing_time - alternate_vleg.swing_elapsed_s;
          }();

          const auto desired_remaining = (
              (-0.5 * ws_.trot.onevleg_time) -
              0.5 * alternate_leg_swing_remaining);

          if (vleg.remaining_s <= desired_remaining) {
            return vleg_idx;
          }
        }
      }

      // No leg to lift now.
      return {};
    }();

    if (!vleg_to_lift) { return; }

    if (*vleg_to_lift >= 0) {
      ws_.next_step_vleg = *vleg_to_lift;
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

    const double target_time_s =
        (ws_.trot.flight_time <= 0.0) ?
        (0.5 * ws_.trot.swing_time + ws_.trot.twovleg_time) :
        (0.5 * ws_.trot.onevleg_time);

    const double filter_time_s = 0.6 * ws_.trot.swing_time;

    // We use an estimate of our expected next velocity to pick the
    // target point.
    const auto predicted_state_R = FilterCommand(
        {state_->robot.desired_R.v, state_->robot.desired_R.w},
        {context_->command->v_R, context_->command->w_R},
        config_.lr_acceleration,
        config_.lr_alpha_rad_s2,
        filter_time_s);

    ws_.vlegs[vleg_idx].predicted_next_v = predicted_state_R.v;
    ws_.vlegs[vleg_idx].target_time_s = target_time_s;

    PropagateLeg propagate(
        predicted_state_R.v,
        predicted_state_R.w,
        -target_time_s);

    for (int leg_idx : kVlegMapping[vleg_idx]) {
      auto& leg_R = GetLeg_R(legs_R, leg_idx);
      const auto& config_leg = context_->GetLeg(leg_idx);

      const auto presult_R = propagate(config_leg.idle_R);
      ws_.legs[leg_idx].target_R = presult_R.position;
      // This isn't necessary, as we don't currently pay attention to
      // latched when in swing, but it is good for consistency.
      ws_.legs[leg_idx].latched = false;

      context_->swing_trajectory[leg_idx] = SwingTrajectory(
          leg_R.position, leg_R.velocity,
          ws_.legs[leg_idx].target_R,
          wc_.lift_height,
          wc_.world_blend_ratio,
          ws_.trot.swing_time);
    }
  }

  void PropagateLegs(std::vector<QC::Leg>* legs_R) {
    // Update our current leg positions.
    PropagateLeg propagator(
        state_->robot.desired_R.v,
        state_->robot.desired_R.w,
        config_.period_s);
    const double kContactDetection_N =
        wc_.contact_detect_load * config_.mass_kg * base::kGravity / 4;

    for (int vleg_idx = 0; vleg_idx < 2; vleg_idx++) {
      auto& vleg = ws_.vlegs[vleg_idx];
      if (vleg.mode == VLeg::kSwing) {
        vleg.swing_elapsed_s += config_.period_s;
      }
      for (int leg_idx : kVlegMapping[vleg_idx]) {
        auto& leg_R = GetLeg_R(legs_R, leg_idx);

        switch (vleg.mode) {
          case VLeg::Mode::kStance: {
            auto& leg_R = GetLeg_R(legs_R, leg_idx);
            const auto& status_R = context_->GetLegState_R(leg_idx);

            if (!ws_.legs[leg_idx].latched &&
                status_R.force_N.z() > kContactDetection_N) {
              // We've now made contact with the ground.  Latch this
              // position, gradually returning to the one we want to
              // be in.
              ws_.legs[leg_idx].latched = true;
              leg_R.position = status_R.position;
              const double downtime = ws_.trot.onevleg_time + ws_.trot.twovleg_time;
              const double delta = config_.stand_height - leg_R.position.z();
              const double restore_time = wc_.stance_restore_fraction * downtime;
              leg_R.velocity.z() = delta / restore_time;
            }

            leg_R.stance =
                std::min(1.0, leg_R.stance + wc_.stance_restore * config_.period_s);

            const auto result_R = propagator(leg_R.position);
            leg_R.position = result_R.position;
            const double old_z_vel = leg_R.velocity.z();
            leg_R.velocity = result_R.velocity;
            leg_R.velocity.z() = old_z_vel;

            const double downtime = ws_.trot.onevleg_time + ws_.trot.twovleg_time;

            if (leg_R.velocity.z() != 0.0) {
              const double scale = std::pow(wc_.stance_restore_scale,
                                            config_.period_s / downtime);
              leg_R.velocity.z() *= scale;
              leg_R.position.z() += leg_R.velocity.z() * config_.period_s;
              const double delta = config_.stand_height - leg_R.position.z();
              if (leg_R.velocity.z() * delta < 0.0) {
                // We're done.
                leg_R.velocity.z() = 0.0;
                leg_R.position.z() = config_.stand_height;
              }
            }

            // TODO: We should keep track of our current desired body
            // acceleration and feed it in here.
            leg_R.acceleration = base::Point3D();

            // Restore our kp and kd scales.
            if (!!leg_R.kp_scale) {
              double kp = leg_R.kp_scale->x();

              kp += wc_.swing_damp_kp_restore * config_.period_s;

              if (kp < 1.0) {
                leg_R.kp_scale = {kp, kp, kp};
              } else {
                leg_R.kp_scale = {};
              }
            }

            if (!!leg_R.kd_scale) {
              double kd = leg_R.kd_scale->x();

              kd += wc_.swing_damp_kd_restore * config_.period_s;

              if (kd < 1.0) {
                leg_R.kd_scale = {kd, kd, kd};
              } else {
                leg_R.kd_scale = {};
              }
            }
            break;
          }
          case VLeg::Mode::kSwing: {
            leg_R.stance = 0.0;
            const auto swing_R =
                context_->swing_trajectory[leg_idx].Advance(
                    config_.period_s,
                    -state_->robot.desired_R.v -
                    state_->robot.desired_R.w.cross(leg_R.position));
            leg_R.position = swing_R.position;
            leg_R.velocity = swing_R.velocity_s;
            leg_R.acceleration = swing_R.acceleration_s2;
            if (swing_R.phase > wc_.swing_damp_start_phase) {
              const double kp = wc_.swing_damp_kp;
              leg_R.kp_scale = {kp, kp, kp};
              const double kd = wc_.swing_damp_kd;
              leg_R.kd_scale = {kd, kd, kd};
            } else {
              leg_R.kp_scale = {};
              leg_R.kd_scale = {};
            }

            break;
          }
        }
      }
      if (vleg.swing_elapsed_s > ws_.trot.swing_time) {
        vleg.mode = VLeg::kStance;
        vleg.swing_elapsed_s = 0.0;

        if (state_->robot.desired_R.v.norm() == 0.0 &&
            state_->robot.desired_R.w.norm() == 0.0) {
          ws_.idle_count++;
        } else {
          ws_.idle_count = 0;
        }

        for (int leg_idx : kVlegMapping[vleg_idx]) {
          ws_.legs[leg_idx].latched = false;
          auto& leg_R = GetLeg_R(legs_R, leg_idx);
          leg_R.stance = wc_.initial_stance;
          leg_R.velocity.z() = 0.0;
        }
      }
    }
  }

  bool all_stance() const {
    return count_stance() == 2;
  }

  int count_stance() const {
    return std::count_if(
        std::begin(ws_.vlegs), std::end(ws_.vlegs),
        [&](const auto& vleg) {
          return vleg.mode == VLeg::Mode::kStance;
        });
  }

  QuadrupedContext* const context_;
  QuadrupedState* const state_;
  const QuadrupedConfig& config_;
  Walk& ws_;
  const QuadrupedConfig::Walk& wc_;
};
}

TrotResult QuadrupedTrot(
    QuadrupedContext* context,
    const std::vector<QuadrupedCommand::Leg>& old_legs_R) {
  WalkContext ctx(context);
  return ctx.Run(old_legs_R);
}

}
}
