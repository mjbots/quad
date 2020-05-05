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

#include <vector>

#include "mjlib/base/visitor.h"

#include "base/point3d.h"
#include "base/sophus.h"
#include "mech/mammal_ik.h"

namespace mjmech {
namespace mech {

// This represents the JSON used to configure the geometry of the
// robot.
struct QuadrupedConfig {
  struct Joint {
    int id = 0;
    double sign = 1.0;
    double min_deg = -360.0;
    double max_deg = 360.0;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(id));
      a->Visit(MJ_NVP(sign));
      a->Visit(MJ_NVP(min_deg));
      a->Visit(MJ_NVP(max_deg));
    }
  };

  std::vector<Joint> joints;

  struct Leg {
    int leg = 0;
    Sophus::SE3d pose_mm_BG;
    MammalIk::Config ik;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(leg));
      a->Visit(MJ_NVP(pose_mm_BG));
      a->Visit(MJ_NVP(ik));
    }
  };

  std::vector<Leg> legs;

  struct Bounds {
    double min_z_B = 0.0;
    double max_z_B = 300.0;
    double max_acceleration_mm_s2 = 100000;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(min_z_B));
      a->Visit(MJ_NVP(max_z_B));
      a->Visit(MJ_NVP(max_acceleration_mm_s2));
    }
  };

  Bounds bounds;

  double mass_kg = 10.0;

  struct StandUp {
    // This pose is referenced to the leg in the front right and the
    // x/y positions should all be positive.  All other positions will
    // be symmetric about the x/y axes.
    base::Point3D pose_mm_R = {151, 219, 49};
    double velocity_dps = 60.0;
    double velocity_mm_s = 150.0;
    double max_preposition_torque_Nm = 3.0;
    double timeout_s = 10.0;
    double tolerance_deg = 1.0;
    double tolerance_mm = 1;
    double force_scale_window_mm = 100;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(pose_mm_R));
      a->Visit(MJ_NVP(velocity_dps));
      a->Visit(MJ_NVP(velocity_mm_s));
      a->Visit(MJ_NVP(max_preposition_torque_Nm));
      a->Visit(MJ_NVP(timeout_s));
      a->Visit(MJ_NVP(tolerance_deg));
      a->Visit(MJ_NVP(tolerance_mm));
      a->Visit(MJ_NVP(force_scale_window_mm));
    }
  };

  StandUp stand_up;

  struct Rest {
    double velocity_mm_s = 100.0;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(velocity_mm_s));
    }
  };

  Rest rest;

  double stand_height_mm = 210.0;

  double idle_x_mm = 190.0;
  double idle_y_mm = 140.0;

  double rb_filter_constant_Hz = 2.0;
  double lr_acceleration_mm_s2 = 1000.0;
  double lr_alpha_rad_s2 = 0.5;

  struct Jump {
    double lower_velocity_mm_s = 100.0;
    double retract_velocity_mm_s = 1000.0;
    double land_threshold_mm = 15.0;
    double land_kp = 0.1;
    double land_kd = 0.1;
    double lower_height_mm = 100.0;
    double upper_height_mm = 220.0;
    double retract_height_mm = 190.0;
    double landing_force_scale = 1.0;
    double land_gain_increase = 100.0;
    double min_acceleration_mm_s2 = 500.0;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(lower_velocity_mm_s));
      a->Visit(MJ_NVP(retract_velocity_mm_s));
      a->Visit(MJ_NVP(land_threshold_mm));
      a->Visit(MJ_NVP(land_kp));
      a->Visit(MJ_NVP(land_kd));
      a->Visit(MJ_NVP(lower_height_mm));
      a->Visit(MJ_NVP(upper_height_mm));
      a->Visit(MJ_NVP(retract_height_mm));
      a->Visit(MJ_NVP(landing_force_scale));
      a->Visit(MJ_NVP(land_gain_increase));
      a->Visit(MJ_NVP(min_acceleration_mm_s2));
    }
  };

  Jump jump;

  struct Walk {
    // The length of a single cycle.
    double cycle_time_s = 0.75;

    double lift_height_mm = 25.0;

    // The length of time allocated for a leg to step forward,
    // measured in fraction of a phase.
    double step_phase = 0.30;

    // Use a different kp gain when lowering, to hopefully cushion our
    // landing.
    double lower_kp = 0.05;
    double lower_kd = 0.4;

    // The elements of this structure are all measured in fraction of
    // a step.
    struct Step {
      // The length of time spent taking weight off the leg before
      // lifting it.
      double release_time = 0.05;

      // The length of time spent lifting the leg to the step height.
      double lift_time = 0.20;

      // The length of time spent lowering the leg to the walk height.
      double lower_time = 0.35;

      // The length of time spent putting weight back on the leg.
      // Measured in fraction of a step.  Note: This time can start
      // sooner or later than scheduled due to when the leg actually
      // makes contact with the ground.
      double load_time = 0.05;

      template <typename Archive>
      void Serialize(Archive* a) {
        a->Visit(MJ_NVP(release_time));
        a->Visit(MJ_NVP(lift_time));
        a->Visit(MJ_NVP(lower_time));
        a->Visit(MJ_NVP(load_time));
      }
    };

    Step step;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(cycle_time_s));
      a->Visit(MJ_NVP(lift_height_mm));
      a->Visit(MJ_NVP(step_phase));
      a->Visit(MJ_NVP(lower_kp));
      a->Visit(MJ_NVP(lower_kd));
      a->Visit(MJ_NVP(step));
    }
  };

  Walk walk;

  struct Backflip {
    double lower_height_mm = 45.0;
    double pitch_accel_dps2 = 5000.0;
    double max_pitch_deg = 35.0;
    double push_pitch_deg = 55.0;

    double acceleration_mm_s2 = 30000.0;
    double push_height_mm = 180.0;
    double flight_velocity_mm_s = 500.0;
    double flight_kp = 0.5;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(lower_height_mm));
      a->Visit(MJ_NVP(pitch_accel_dps2));
      a->Visit(MJ_NVP(max_pitch_deg));
      a->Visit(MJ_NVP(push_pitch_deg));
      a->Visit(MJ_NVP(acceleration_mm_s2));
      a->Visit(MJ_NVP(push_height_mm));
      a->Visit(MJ_NVP(flight_velocity_mm_s));
      a->Visit(MJ_NVP(flight_kp));
    }
  };

  Backflip backflip;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(joints));
    a->Visit(MJ_NVP(legs));
    a->Visit(MJ_NVP(bounds));
    a->Visit(MJ_NVP(mass_kg));
    a->Visit(MJ_NVP(stand_up));
    a->Visit(MJ_NVP(rest));
    a->Visit(MJ_NVP(stand_height_mm));
    a->Visit(MJ_NVP(idle_x_mm));
    a->Visit(MJ_NVP(idle_y_mm));
    a->Visit(MJ_NVP(rb_filter_constant_Hz));
    a->Visit(MJ_NVP(lr_acceleration_mm_s2));
    a->Visit(MJ_NVP(lr_alpha_rad_s2));
    a->Visit(MJ_NVP(jump));
    a->Visit(MJ_NVP(walk));
    a->Visit(MJ_NVP(backflip));
  }
};

}
}
