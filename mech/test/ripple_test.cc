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

#include "mech/ripple.h"

#include <boost/test/auto_unit_test.hpp>

#include <fmt/format.h>

namespace {
using namespace mjmech::base;
using namespace mjmech::mech;

void CheckPoints(const Point3D& lhs, const Point3D& rhs) {
  BOOST_CHECK_SMALL(lhs.x() - rhs.x(), 1e-2);
  BOOST_CHECK_SMALL(lhs.y() - rhs.y(), 1e-2);
  BOOST_CHECK_SMALL(lhs.z() - rhs.z(), 1e-2);
}

template <typename T>
void SanityCheckState(const T& state) {
  BOOST_CHECK_EQUAL(state.robot_frame.parent, &state.world_frame);
  BOOST_CHECK_EQUAL(state.body_frame.parent, &state.robot_frame);

  std::set<const Frame*> valid_frames{
    &state.world_frame, &state.robot_frame, &state.body_frame
        };

  int stance_legs = 0;
  for (const auto& leg: state.legs) {
    BOOST_CHECK_EQUAL(valid_frames.count(leg.frame), 1);

    if (leg.mode == Leg::Mode::kStance) {
      stance_legs += 1;
      BOOST_CHECK_EQUAL(leg.frame, &state.world_frame);
    } else if (leg.mode == Leg::Mode::kSwing) {
    } else {
      BOOST_CHECK_MESSAGE(false,
                          fmt::format("unknown leg mode {}",
                                      static_cast<int>(leg.mode)));
    }
  }

  // Require at least 2 legs in stance at all times.
  BOOST_CHECK_GE(stance_legs, 2);
}

boost::shared_ptr<IKSolver> MakeIKSolver(int start_servo_num) {
  LizardIK::Config config;
  config.coxa.min_deg = -120.0;
  config.coxa.idle_deg = 0.0;
  config.coxa.max_deg = 120.0;
  config.coxa.length_mm = 60.0;
  config.coxa.ident = start_servo_num + 0;

  config.femur.min_deg = -120.0;
  config.femur.idle_deg = 0.0;
  config.femur.max_deg = 120.0;
  config.femur.length_mm = 60.0;
  config.femur.ident = start_servo_num + 1;

  config.tibia.min_deg = -120.0;
  config.tibia.idle_deg = 0.0;
  config.tibia.max_deg = 120.0;
  config.tibia.length_mm = 60.0;
  config.tibia.ident = start_servo_num + 2;

  config.servo_speed_dps = 360.0;

  return boost::shared_ptr<IKSolver>(new LizardIK(config));
}

RippleConfig MakeConfig() {
  RippleConfig result;

  double mounts[][2]= {
    { 90, 90 },
    { 90, -90 },
    { -90, -90 },
    { -90, 90 },
  };

  for (int i = 0; i < 4; i++) {
    Leg::Config leg_config;
    leg_config.mount_mm.x() = mounts[i][0];
    leg_config.mount_mm.y() = mounts[i][1];
    leg_config.mount_mm.z() = 0;

    leg_config.idle_mm.x() = 100.0;
    leg_config.idle_mm.y() = 0;
    leg_config.idle_mm.z() = 0;

    leg_config.leg_ik = MakeIKSolver(i * 3);

    result.mechanical.leg_config.push_back(leg_config);
  }

  result.mechanical.body_cog_mm.x() = 0;
  result.mechanical.body_cog_mm.y() = 0;
  result.mechanical.body_cog_mm.z() = 0;

  result.max_cycle_time_s = 4.0;
  result.lift_height_mm = 20.0;
  result.swing_percent = 50.0;
  result.leg_order = { {0}, {2}, {1}, {3} };
  result.body_z_offset_mm = 60.0;

  return result;
}

template <typename Gait, typename Command>
void RunCycle(Gait& gait, const Command& command,
              double total_time_s, double time_step_s) {
  gait.SetCommand(command);
  auto old_state = gait.state();

  double cur_time_s = 0.0;
  while (cur_time_s < total_time_s) {
    const auto& joints = gait.AdvanceTime(time_step_s);
    cur_time_s += time_step_s;

    BOOST_CHECK_GE(joints.joints.size(), 1);

    auto this_state = gait.state();
    SanityCheckState(this_state);

    for (size_t leg_num = 0; leg_num < this_state.legs.size(); leg_num++) {
      const auto& this_leg = this_state.legs.at(leg_num);
      const auto& old_leg = old_state.legs.at(leg_num);

      Point3D current_world_point = this_state.world_frame.MapFromFrame(
          this_leg.frame, this_leg.point);
      Point3D old_world_point = old_state.world_frame.MapFromFrame(
          old_leg.frame, old_leg.point);

      // Lets identified as in stance do not move in the world frame.
      if (this_leg.mode == Leg::Mode::kStance &&
          old_leg.mode == Leg::Mode::kStance) {
        CheckPoints(current_world_point, old_world_point);
      }

      // No leg moves faster than X in the world frame.
      double world_speed_mm_s =
          (current_world_point - old_world_point).norm() / time_step_s;
      BOOST_CHECK_LT(world_speed_mm_s, 1100);

      // No leg moves faster than Y in the body frame.
      Point3D current_body_point = this_state.body_frame.MapFromFrame(
          this_leg.frame, this_leg.point);
      Point3D old_body_point = old_state.body_frame.MapFromFrame(
          old_leg.frame, old_leg.point);

      double body_speed_mm_s =
          (current_body_point - old_body_point).norm() / time_step_s;
      BOOST_CHECK_LT(body_speed_mm_s, 1100);

      old_state = this_state;
    }
  }
}
}

BOOST_AUTO_TEST_CASE(TestRippleBasic) {
  RippleConfig config = MakeConfig();
  RippleGait gait(config);

  // We want to:
  //  1. get the idle state
  //  2. initialize ourselves to that
  //
  // then,
  //  a) command no motion... verify stepping in place
  //  b) command forward motion
  //  c) command rotation
  //  d) do all of the above with each of the other 6 degrees of
  //     freedom altered

  // Also, we will want to test changes in command, and coming to a
  // stop.
  const auto state = gait.state();

  CheckPoints(state.legs.at(0).point, Point3D(190, 90, 0));
  CheckPoints(state.legs.at(1).point, Point3D(190, -90, 0));
  CheckPoints(state.legs.at(2).point, Point3D(-190, -90, 0));
  CheckPoints(state.legs.at(3).point, Point3D(-190, 90, 0));

  SanityCheckState(state);

  Command command;
  RunCycle(gait, command, 10.0, 0.01);
  RunCycle(gait, command, 10.0, 0.0073);
}

BOOST_AUTO_TEST_CASE(TestRippleAdvanced) {
  RippleConfig config = MakeConfig();
  Command command;

  config.max_cycle_time_s = 1.7;
  {
    RippleGait gait(config);
    RunCycle(gait, command, 5.0, 0.01);
  }

  command.translate_y_mm_s = 50.0;
  {
    RippleGait gait(config);
    RunCycle(gait, command, 7.0, 0.005);

    RunCycle(gait, command, 7.0, 0.03);
    RunCycle(gait, command, 7.0, 0.04);
  }
}

namespace {
boost::shared_ptr<IKSolver> MakeMammalIKSolver(int start_servo_num) {
  MammalIK::Config r;

  r.femur_attachment_mm.x() = 0;
  r.femur_attachment_mm.y() = 30;
  r.femur_attachment_mm.z() = 0;

  r.shoulder.min_deg = -90.0;
  r.shoulder.idle_deg = 0.0;
  r.shoulder.max_deg = 90.0;
  r.shoulder.ident = start_servo_num + 0;

  r.femur.min_deg = -170.0;
  r.femur.idle_deg = 0.0;
  r.femur.max_deg = 170.0;
  r.femur.length_mm = 140.0;
  r.femur.ident = start_servo_num + 1;

  r.tibia.min_deg = -170.0;
  r.tibia.idle_deg = 0.0;
  r.tibia.max_deg = 170.0;
  r.tibia.length_mm = 135.0;
  r.tibia.ident = start_servo_num + 2;

  r.servo_speed_dps = 360.0;

  r.invert = (start_servo_num == 3 || start_servo_num == 9);

  return boost::shared_ptr<MammalIK>(new MammalIK(r));
}

RippleConfig MakeMammalConfig() {
  RippleConfig result;

  double mounts[][2]= {
    { 175, 65 },
    { 175, -65 },
    { -175, -65 },
    { -175, 65 },
  };

  for (int i = 0; i < 4; i++) {
    Leg::Config leg_config;
    leg_config.mount_mm.x() = mounts[i][0];
    leg_config.mount_mm.y() = mounts[i][1];
    leg_config.mount_mm.z() = 0;

    leg_config.idle_mm.x() = 65.0;
    leg_config.idle_mm.y() = 0;
    leg_config.idle_mm.z() = 0;

    leg_config.leg_ik = MakeMammalIKSolver(i * 3);

    result.mechanical.leg_config.push_back(leg_config);
  }

  result.max_cycle_time_s = 4.0;
  result.lift_height_mm = 20.0;
  result.swing_percent = 50.0;
  result.leg_order = { {0}, {2}, {1}, {3} };
  result.body_z_offset_mm = 257.0;

  return result;
}
}

BOOST_AUTO_TEST_CASE(TestRippleResting) {
  auto config = MakeMammalConfig();
  RippleGait dut(config);

  {
    const auto state = dut.GetPrepositioningState(0.0);
    const auto cmd = dut.MakeJointCommand(state);
    BOOST_TEST(cmd.joints.size() == 12);
  }

  {
    const auto state = dut.GetPrepositioningState(1.0);
    const auto cmd = dut.MakeJointCommand(state);
    BOOST_TEST(cmd.joints.size() == 12);
  }
}
