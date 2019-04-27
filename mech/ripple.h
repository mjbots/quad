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

// TODO jpieper
//
//  * During the swing phase, this forces the end effector to travel
//    in a robot-frame straight line to the new position.  All we
//    really care is that the Z position follows a given trajectory.
//    There could be alternate, faster or more efficient paths to
//    reach the new position.
//
//  * If the command is changed during a swing, the leg target
//    position is not updated.  The current representation doesn't
//    necessarily make that super easy, but it should be doable.
//    Right now, commands are only updated at the end of a full phase,
//    which is kind of a long time.
//
//  * I'm not sure how many heap operations are required for a single
//    gait update.  Ideally, it would be 0.
//
//  * Setting a new command is a very heavyweight operation, since it
//    must do a full IK simulation to see if the new command is
//    feasible or not.

#pragma once

#include <map>
#include <optional>

#include "mjlib/base/visitor.h"

#include "gait.h"

namespace mjmech {
namespace mech {
struct RippleConfig {
  MechanicalConfig mechanical;

  double max_cycle_time_s = 4.0;
  double lift_height_mm = 80.0;
  double lift_percent = 25.0;
  double swing_percent = 80.0;
  double position_margin_percent = 80.0;
  std::vector<std::vector<int>> leg_order;
  double body_z_offset_mm = 0.0;
  double servo_speed_margin_percent = 70.0;
  bool statically_stable = false;
  double static_center_factor = 3.0;
  double static_stable_factor = 10.0;
  double static_margin_mm = 20.0;
  double servo_speed_dps = 360.0;

  double attitude_p_mm_deg = 0.0;
  double attitude_i_mm_dps = 0.0;
  double rate_p_mm_dps = 0.0;
  double rate_i_mm_dps2 = 0.0;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(mechanical));
    a->Visit(MJ_NVP(max_cycle_time_s));
    a->Visit(MJ_NVP(lift_height_mm));
    a->Visit(MJ_NVP(lift_percent));
    a->Visit(MJ_NVP(swing_percent));
    a->Visit(MJ_NVP(position_margin_percent));
    a->Visit(MJ_NVP(leg_order));
    a->Visit(MJ_NVP(body_z_offset_mm));
    a->Visit(MJ_NVP(servo_speed_margin_percent));
    a->Visit(MJ_NVP(statically_stable));
    a->Visit(MJ_NVP(static_center_factor));
    a->Visit(MJ_NVP(static_stable_factor));
    a->Visit(MJ_NVP(static_margin_mm));
    a->Visit(MJ_NVP(servo_speed_dps));
    a->Visit(MJ_NVP(attitude_p_mm_deg));
    a->Visit(MJ_NVP(attitude_i_mm_dps));
    a->Visit(MJ_NVP(rate_p_mm_dps));
    a->Visit(MJ_NVP(rate_i_mm_dps2));
  }
};

struct RippleState : public CommonState {
  double phase = 0.0;
  enum Action {
    kActionStartSwing,
    kActionStartStance,
    kActionEnd,
  };

  int action = 0;

  struct Leg : public mjmech::mech::Leg::State {
    std::optional<base::Point3D> swing_start_pos;
    std::optional<base::Point3D> swing_end_pos;

    bool operator==(const Leg&) const { return true; }
  };

  std::vector<Leg> legs;

  RippleState() {}
  RippleState(const RippleState& rhs) : CommonState(rhs) {
    *this = rhs;
  }

  RippleState& operator=(const RippleState& rhs) {
    *static_cast<CommonState*>(this) =
        *(static_cast<const CommonState*>(&rhs));
    phase = rhs.phase;
    action = rhs.action;
    legs = rhs.legs;

    auto map_frame = [this, &rhs](const base::Frame* frame) {
      if (frame == &rhs.world_frame) {
        return &world_frame;
      } else if (frame == &rhs.robot_frame) {
        return &robot_frame;
      } else if (frame == &rhs.body_frame) {
        return &body_frame;
      } else if (frame == &rhs.cog_frame) {
        return &cog_frame;
      } else if (frame == nullptr) {
        return static_cast<base::Frame*>(nullptr);
      }
      mjlib::base::AssertNotReached();
    };

    for (auto& leg: legs) {
      leg.frame = map_frame(leg.frame);
    }
    return *this;
  }
};

struct Options {
  double cycle_time_s = 1.0;
  double servo_speed_dps = 0.0;
};

class RippleGait : public Gait {
 public:
  RippleGait(const RippleConfig& config)
    : config_(config),
      idle_state_(GetIdleState()),
      actions_(GetActionList()),
      state_(idle_state_) {
  }

  virtual ~RippleGait() {}

  void SetState(const RippleState& state) {
    state_ = state;
  }

  virtual Result SetCommand(const Command& command) override {
    Command command_copy = command;
    std::optional<Options> options = SelectCommandOptions(&command_copy);

    if (!options) {
      return kNotSupported;
    }

    ReallySetCommand(command_copy, *options);
    options_ = *options;
    return kValid;
  }

  virtual JointCommand AdvancePhase(double delta_phase) override {
    if (config_.leg_order.empty()) {
      state_.phase = std::fmod(state_.phase + delta_phase, 1.0);
      return MakeJointCommand(state_);
    }

    double cur_phase = state_.phase;
    double next_phase = state_.phase + delta_phase;

    for (;;) {
      const auto& next_action = actions_.at(state_.action);

      if (next_action.phase > next_phase) { break; }

      const double advance_phase = next_action.phase;
      if (advance_phase != cur_phase) {
        const double delta_phase = advance_phase - cur_phase;
        AdvancePhaseNoaction(delta_phase, advance_phase);
      }

      cur_phase = advance_phase;
      DoAction(state_.action);

      state_.action++;

      if (state_.action >= static_cast<int>(actions_.size())) {
        state_.action = 0;
        next_phase -= 1.0;
        cur_phase -= 1.0;

        // Install our new command.
      }
    }

    // Finally, advance the remainder of the phase and update our state.
    AdvancePhaseNoaction(next_phase - cur_phase, next_phase);
    state_.phase = next_phase;

    return MakeJointCommand(state_);
  }

  virtual JointCommand AdvanceTime(double delta_s) override {
    return AdvancePhase(
        (command_.time_rate_percent / 100.0) *
        delta_s / phase_time_s());
  }

  const RippleState& state() const { return state_; }
  const Options& options() const { return options_; }
  const Command& command() const { return command_; }

  RippleState GetIdleState() const {
    RippleState result;

    result.body_frame.transform.translation.z = config_.body_z_offset_mm;

    for (const auto& leg_config: config_.mechanical.leg_config) {
      base::Point3D point;

      const double x_sign = base::GetSign(leg_config.mount_mm.x);
      point.x = leg_config.mount_mm.x + leg_config.idle_mm.x * x_sign;

      const double y_sign = base::GetSign(leg_config.mount_mm.y);
      point.y = leg_config.mount_mm.y + leg_config.idle_mm.y * y_sign;

      point.z = leg_config.mount_mm.z +
          leg_config.idle_mm.z -
          config_.body_z_offset_mm;

      result.legs.emplace_back();
      RippleState::Leg& leg_state = result.legs.back();

      leg_state.point = result.world_frame.MapFromFrame(
          &result.body_frame, point);
      leg_state.frame = &result.world_frame;
      leg_state.mode = Leg::Mode::kStance;

      leg_state.leg_ik = leg_config.leg_ik;
    }

    return result;
  }

  JointCommand MakeJointCommand(const RippleState& state) const {
    JointCommand result;
    int index = 0;
    for (const auto& leg: state.legs) {
      base::Frame shoulder_frame;
      MakeShoulderFrame(config_.mechanical.leg_config.at(index),
                        &state, &shoulder_frame);

      base::Point3D shoulder_point =
          shoulder_frame.MapFromFrame(leg.frame, leg.point);
      auto joints = leg.leg_ik->Solve(shoulder_point);
      for (const auto& joint: joints.joints) {
        result.joints.emplace_back(
            JointCommand::Joint(joint.ident, joint.angle_deg));
      }

      index += 1;
    }

    return result;
  }

 private:
  void MakeShoulderFrame(const Leg::Config& leg_config,
                         const RippleState* state,
                         base::Frame* shoulder_frame) const {
    state->MakeShoulder(leg_config, shoulder_frame);
  }

  struct Action;

  std::optional<Options> SelectCommandOptions(Command* command) const {
    if (config_.leg_order.empty()) { return std::nullopt; }
    if (command->time_rate_percent > 100.0) { return std::nullopt; }

    // First, iterate, solving IK for all legs in time until we
    // find the point at which the first leg is unsolvable.
    const double dt = 0.05;

    double time_s = 0.0;

    RippleState my_state = idle_state_;
    ApplyBodyCommand(&my_state, *command);

    std::optional<double> end_time_s = 0.0;
    end_time_s = std::nullopt; // work around maybe-uninitialized
    std::optional<double> min_observed_speed = 0.0;
    min_observed_speed = std::nullopt; // work around maybe-uninitialized

    std::map<std::pair<int, int>, double> old_joint_angle_deg;

    const double fraction_in_stance = 1.0 - swing_phase_time();
    const double margin =
        0.01 * config_.position_margin_percent * fraction_in_stance;

    const double final_time_s = 0.5 * config_.max_cycle_time_s / margin;
    while (time_s < final_time_s) {
      if (!!end_time_s) { break; }
      time_s += dt;

      for (const double direction: std::vector<double>{-1, 1}) {
        base::Frame frame;
        GetUpdateFrameCommand(direction * time_s, *command, &frame);

        if (!!end_time_s) { break; }

        int leg_num = 0;
        for (const auto& leg: my_state.legs) {
          // TODO: Need to do this for the lifted leg as well.
          auto leg_robot_frame_point = frame.MapToParent(leg.point);
          base::Frame shoulder_frame;
          MakeShoulderFrame(
              config_.mechanical.leg_config.at(leg_num),
              &my_state, &shoulder_frame);

          auto leg_shoulder_point = shoulder_frame.MapFromFrame(
              &my_state.robot_frame, leg_robot_frame_point);

          const auto& leg_config = config_.mechanical.leg_config.at(leg_num);

          auto result = leg_config.leg_ik->Solve(leg_shoulder_point);
          if (!result.Valid()) {
            // Break, so that we can take action knowing
            // how far we can go.
            end_time_s = time_s;
            break;
          }

          std::optional<double> largest_change_deg = 0.0;
          largest_change_deg = std::nullopt;

          for (const auto& joint: result.joints) {
            auto old_result_it =
                old_joint_angle_deg.find(
                    std::make_pair(direction, joint.ident));
            if (old_result_it != old_joint_angle_deg.end()) {
              const double old_angle_deg = old_result_it->second;
              const double new_angle_deg = joint.angle_deg;
              const double delta_deg = std::abs(old_angle_deg - new_angle_deg);
              if (!largest_change_deg ||
                  delta_deg > *largest_change_deg) {
                largest_change_deg = delta_deg;
              }
            }

            old_joint_angle_deg.insert(
                std::make_pair(
                    std::make_pair(direction, joint.ident),
                    joint.angle_deg));
          }

          if (largest_change_deg) {
            const double this_speed_deg_s = *largest_change_deg / dt;

            if (!min_observed_speed ||
                this_speed_deg_s < *min_observed_speed) {
              min_observed_speed = this_speed_deg_s;
            }
          }
          leg_num++;
        }
      }
    }

    if (!min_observed_speed) {
      return std::nullopt;
    }

    Options result;
    if (!end_time_s) {
      // We can achieve this at the maximum time.
      result.cycle_time_s = config_.max_cycle_time_s;
    } else {
      result.cycle_time_s = (2.0 * (*end_time_s) * margin);
    }

    // TODO jpieper: See if this cycle time is feasible.  We will
    // do this by checking to see if the swing leg has to move too
    // fast.
    const double min_swing_speed =
        (*min_observed_speed *
         (1.0 - swing_phase_time()) /
         swing_phase_time());

    result.servo_speed_dps = min_swing_speed;

    const double servo_speed_dps = config_.servo_speed_dps;

    const double speed_margin = 0.01 * config_.servo_speed_margin_percent;
    if (min_swing_speed > speed_margin * servo_speed_dps) {
      // Slow the command down.
      const double slow_down_factor =
          (min_swing_speed /
           (speed_margin * servo_speed_dps));
      command->translate_x_mm_s /= slow_down_factor;
      command->translate_y_mm_s /= slow_down_factor;
      command->rotate_deg_s /= slow_down_factor;
      result.cycle_time_s *= slow_down_factor;
      result.servo_speed_dps = speed_margin * servo_speed_dps;
    }

    return result;
  }

  void ReallySetCommand(const Command& command, const Options& options) {
    command_ = command;
    options_ = options;

    ApplyBodyCommand(&state_, command);
  }

  void ApplyBodyCommand(RippleState* state, const Command& command) const {
    // TODO jpieper: Handle statically stable gaits.
    state->body_frame.transform.translation.x = command.body_x_mm;
    state->body_frame.transform.translation.y = command.body_y_mm;
    state->body_frame.transform.translation.z = command.body_z_mm;
    state->body_frame.transform.translation.z += config_.body_z_offset_mm;
    state->body_frame.transform.rotation = base::Quaternion::FromEuler(
        base::Radians(command.body_roll_deg),
        base::Radians(command.body_pitch_deg),
        base::Radians(command.body_yaw_deg));
  }

  void AdvancePhaseNoaction(double delta_phase, double final_phase) {
    const double dt = delta_phase * phase_time_s();

    base::Frame update_frame;
    GetUpdateFrame(dt, &update_frame);

    auto new_transform = update_frame.TransformToFrame(&state_.world_frame);
    state_.robot_frame.transform = new_transform;

    // Update the legs which are in swing.
    for (auto& leg: state_.legs) {
      if (leg.mode == Leg::Mode::kSwing) {
        double leg_phase =
            (std::fmod(final_phase, (1.0 / config_.leg_order.size())) /
             swing_phase_time());
        // Don't allow the phase to wrap-around on the final update.
        if (delta_phase > 0.0 && leg_phase == 0.0) {
          leg_phase = 1.0;
        }

        BOOST_ASSERT(leg.frame == &state_.robot_frame);
        const auto delta = *leg.swing_end_pos - *leg.swing_start_pos;
        const auto current = *leg.swing_start_pos + delta.scaled(leg_phase);
        leg.point = current;

        const double lift_fraction = 0.01 * config_.lift_percent;
        const double height_mm =
            config_.lift_height_mm *
            command_.lift_height_percent / 100.0;
        if (leg_phase < lift_fraction) {
          leg.point.z = (leg_phase / lift_fraction) * height_mm;
        } else if (leg_phase < (1.0 - lift_fraction)) {
          leg.point.z = height_mm;
        } else {
          leg.point.z = ((1.0 - leg_phase) / lift_fraction) * height_mm;
        }
      }
    }

    // TODO jpieper: Statically stable gait.
  }

  void DoAction(int action_index) {
    const auto& action = actions_[action_index];

    if (action.action == RippleState::kActionEnd) { return; }

    for (int leg_num: config_.leg_order.at(action.leg_group)) {
      auto& leg = state_.legs.at(leg_num);

      if (action.action == RippleState::kActionStartStance) {
        leg.mode = Leg::Mode::kStance;
        leg.point = state_.world_frame.MapFromFrame(
            leg.frame, leg.point);
        leg.point.z = 0;
        leg.frame = &state_.world_frame;
      } else if (action.action == RippleState::kActionStartSwing) {
        leg.mode = Leg::Mode::kSwing;
        leg.point = state_.robot_frame.MapFromFrame(
            leg.frame, leg.point);
        leg.frame = &state_.robot_frame;
        leg.swing_start_pos = leg.point;
        leg.swing_end_pos = GetSwingEndPos(leg_num);
      }
    }
  }

  void GetUpdateFrame(double dt, base::Frame* frame) const {
    return GetUpdateFrameCommand(dt, command_, frame);
  }

  void GetUpdateFrameCommand(
      double dt, const Command& command, base::Frame* result) const {
    result->parent = &state_.robot_frame;

    const double vx = command.translate_x_mm_s;
    const double vy = command.translate_y_mm_s;
    double dx = 0.0;
    double dy = 0.0;
    if (command.rotate_deg_s == 0.0) {
      dx = vx * dt;
      dy = vy * dt;
    } else {
      const double vyaw = base::Radians(command.rotate_deg_s);
      dx = ((std::cos(dt * vyaw) - 1) * vy +
            std::sin(dt * vyaw) * vx) / vyaw;
      dy = ((std::cos(dt * vyaw) - 1) * vx +
             std::sin(dt * vyaw) * vy) / vyaw;
      result->transform.rotation =
          base::Quaternion::FromEuler(0, 0, dt * vyaw);
    }

    result->transform.translation.x = dx;
    result->transform.translation.y = dy;
  }

  base::Point3D GetSwingEndPos(int leg_num) const {
    // Target swing end positions such that during stance, the leg
    // will spend half its travel time reaching the idle position, and
    // half its travel time going beyond the idle position.

    const double stance_phase_time = 1.0 - swing_phase_time();
    const double dt = 0.5 * stance_phase_time * phase_time_s();

    base::Frame end_frame;
    GetUpdateFrame(dt, &end_frame);

    // TODO jpieper: This should map from whatever frame the idle
    // state leg was actually in.
    return end_frame.MapToParent(idle_state_.legs.at(leg_num).point);
  }

  std::vector<Action> GetActionList() const {
    std::vector<Action> result;

    if (config_.leg_order.empty()) { return result; }

    const double swing_time = swing_phase_time();

    for (int i = 0; i < static_cast<int>(config_.leg_order.size()); i++) {
      const double fraction =
          static_cast<double>(i) / config_.leg_order.size();
      Action action = { fraction, i, RippleState::kActionStartSwing };
      result.push_back(action);
      Action action2 = { fraction + swing_time, i,
                         RippleState::kActionStartStance };
      result.push_back(action2);
    }

    Action action3 = { 1.0, -1, RippleState::kActionEnd };
    result.push_back(action3);
    return result;
  }

  double swing_phase_time() const {
    return (1.0 / config_.leg_order.size()) * 0.01 * config_.swing_percent;
  }

  double phase_time_s() const { return options_.cycle_time_s; }

  const RippleConfig config_;
  Options options_;
  const RippleState idle_state_;

  struct Action {
    double phase = 0.0;
    int leg_group = -1;
    RippleState::Action action;

    Action() {}
    Action(double phase, int leg_group, RippleState::Action action)
        : phase(phase), leg_group(leg_group), action(action) {}
  };

  const std::vector<Action> actions_;

  // Mutable state.

  RippleState state_;
  Command command_;
};
}
}
