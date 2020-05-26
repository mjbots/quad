// Copyright 2020 Josh Pieper, jjp@pobox.com.
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

#include "mech/swing_trajectory.h"

#include "mjlib/base/assert.h"

namespace mjmech {
namespace mech {

SwingTrajectory::SwingTrajectory(const Eigen::Vector3d& start,
                                 const Eigen::Vector3d& start_velocity,
                                 const Eigen::Vector3d& end,
                                 double height,
                                 double world_blend,
                                 double swing_time_s)
    : start_(start),
      start_velocity_(start_velocity),
      end_(end),
      world_blend_(world_blend),
      swing_time_s_(swing_time_s),
      zinterp_(start.z(), start.z() - height),
      xymove_((start.head<2>() +
               0.5 * world_blend * swing_time_s * start_velocity.head<2>()),
              end.head<2>()) {
  MJ_ASSERT(start.z() == end.z());
}

SwingTrajectory::Result SwingTrajectory::Advance(
    double delta_s, const Eigen::Vector3d& world_velocity_s) {
  const double old_phase = phase_;
  phase_ += delta_s / swing_time_s_;
  phase_ = std::min(1.0, phase_);

  Result result;
  result.phase = phase_;

  // Do the Z first.  It only depends upon the global phase.
  if (phase_ < 0.5) {
    result.position.z() = zinterp_.position(2 * phase_);
    result.velocity_s.z() = zinterp_.velocity(2 * phase_) /
                            (0.5 * swing_time_s_);
    result.acceleration_s2.z() = zinterp_.acceleration(2 * phase_) /
                                 (0.5 * swing_time_s_);
  } else {
    result.position.z() = zinterp_.position(2.0 - 2 * phase_);
    result.velocity_s.z() = -zinterp_.velocity(2.0 - 2 * phase_) /
                            (0.5 * swing_time_s_);
    result.acceleration_s2.z() = zinterp_.acceleration(2.0 - 2 * phase_) /
                                 (0.5 * swing_time_s_);
  }

  // Now do the X/Y.  We break it up into 3 phases, lift, move, and lower.
  if (phase_ < world_blend_) {
    // Lift.
    result.acceleration_s2.head<2>() =
        (-start_velocity_ * (1.0 / (swing_time_s_ * world_blend_))).head<2>();
    result.velocity_s.head<2>() =
        (start_velocity_ * (1.0 - phase_ / world_blend_)).head<2>();
    const double time_s = phase_ * swing_time_s_;
    result.position.head<2>() =
        (start_ + time_s * start_velocity_ +
         0.5 * result.acceleration_s2 * time_s * time_s).head<2>();
  } else if (phase_ < 1.0 - world_blend_) {
    const double xyphase = (phase_ - world_blend_) / (1.0 - 2.0 * world_blend_);

    result.position.head<2>() = xymove_.position(xyphase);
    result.velocity_s.head<2>() = xymove_.velocity(xyphase);
    result.acceleration_s2.head<2>() = xymove_.acceleration(xyphase);
  } else {
    // Lower.
    const double blend_s = swing_time_s_ * world_blend_;
    const double phase_ratio = (phase_ - (1.0 - world_blend_)) / world_blend_;
    result.acceleration_s2.head<2>() =
        (world_velocity_s / (1.0 / blend_s)).head<2>();
    result.velocity_s.head<2>() =
        (phase_ratio * world_velocity_s).head<2>();

    const double old_phase_ratio =
        (old_phase - (1.0 - world_blend_)) / world_blend_;
    if (old_phase_ratio <= 0.0) {
      // If this is our first lowering time, lock in the position to track.
      current_.head<2>() = end_.head<2>();
      current_.head<2>() += result.velocity_s.head<2>() * (phase_ratio * blend_s);
    } else {
      // Otherwise, we integrate from where we were last time.
      current_.head<2>() += delta_s * result.velocity_s.head<2>();
    }
    result.position.head<2>() = current_.head<2>();
  }

  return result;
}

}
}
