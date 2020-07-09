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
  double period_s = 0.0025;

  struct Joint {
    int id = 0;
    double sign = 1.0;
    double rezero_pos_deg = 0.0;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(id));
      a->Visit(MJ_NVP(sign));
      a->Visit(MJ_NVP(rezero_pos_deg));
    }
  };

  std::vector<Joint> joints;

  // The maximum error is about +-30 deg, so this will at least catch
  // some fraction of failures to position properly at startup.
  double rezero_threshold_deg = 15.0;

  struct Leg {
    int leg = 0;
    Sophus::SE3d pose_BG;
    MammalIk::Config ik;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(leg));
      a->Visit(MJ_NVP(pose_BG));
      a->Visit(MJ_NVP(ik));
    }
  };

  std::vector<Leg> legs;

  struct Bounds {
    double min_z_B = 0.0;
    double max_z_B = 0.3;
    double max_acceleration = 100;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(min_z_B));
      a->Visit(MJ_NVP(max_z_B));
      a->Visit(MJ_NVP(max_acceleration));
    }
  };

  Bounds bounds;

  double mass_kg = 10.0;
  double leg_mass_kg = 0.5;
  base::Point3D center_of_mass_B;

  struct StandUp {
    // This pose is referenced to the leg in the front right and the
    // x/y positions should all be positive.  All other positions will
    // be symmetric about the x/y axes.
    base::Point3D pose_R = {0.151, 0.219, 0.049};
    double velocity_dps = 60.0;
    double velocity = 0.150;
    double max_preposition_torque_Nm = 3.0;
    double preposition_kp_scale = 5.0;
    double timeout_s = 10.0;
    double tolerance_deg = 1.0;
    double tolerance_m = 0.001;
    double acceleration = 2.000;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(pose_R));
      a->Visit(MJ_NVP(velocity_dps));
      a->Visit(MJ_NVP(velocity));
      a->Visit(MJ_NVP(max_preposition_torque_Nm));
      a->Visit(MJ_NVP(preposition_kp_scale));
      a->Visit(MJ_NVP(timeout_s));
      a->Visit(MJ_NVP(tolerance_deg));
      a->Visit(MJ_NVP(tolerance_m));
      a->Visit(MJ_NVP(acceleration));
    }
  };

  StandUp stand_up;

  struct Rest {
    double velocity = 0.100;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(velocity));
    }
  };

  Rest rest;

  double stand_height = 0.210;

  double idle_x = 0.190;
  double idle_y = 0.140;

  base::Point3D default_kp_N_m = {1000.0, 1000.0, 200.0};
  base::Point3D default_kd_N_m_s = {20.0, 20.0, 20.0};

  double rb_filter_constant_Hz = 2.0;
  double lr_acceleration = 2.000;
  double lr_alpha_rad_s2 = 1.0;
  double terrain_filter_s = 0.5;
  double voltage_filter_s = 1.0;

  struct Jump {
    double lower_velocity = 0.100;
    double retract_velocity = 1.000;
    double retract_acceleration = 100.000;
    double land_threshold = 0.015;
    double kp_scale = 10.0;
    double kd_scale = 1.0;
    double land_kp = 1.0;
    double land_kd = 0.1;
    double lower_height = 0.100;
    double upper_height = 0.220;
    double retract_height = 0.190;
    double landing_force_scale = 1.0;
    double land_gain_increase = 100.0;
    double min_acceleration = 0.500;
    double land_velocity = 0.01;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(lower_velocity));
      a->Visit(MJ_NVP(retract_velocity));
      a->Visit(MJ_NVP(retract_acceleration));
      a->Visit(MJ_NVP(land_threshold));
      a->Visit(MJ_NVP(kp_scale));
      a->Visit(MJ_NVP(kd_scale));
      a->Visit(MJ_NVP(land_kp));
      a->Visit(MJ_NVP(land_kd));
      a->Visit(MJ_NVP(lower_height));
      a->Visit(MJ_NVP(upper_height));
      a->Visit(MJ_NVP(retract_height));
      a->Visit(MJ_NVP(landing_force_scale));
      a->Visit(MJ_NVP(land_gain_increase));
      a->Visit(MJ_NVP(min_acceleration));
      a->Visit(MJ_NVP(land_velocity));
    }
  };

  Jump jump;

  struct Walk {
    // The length of a single leg swing.
    double swing_time_s = 0.25;

    double lift_height = 0.025;

    // The minimum amount of time to spend in stance.
    double min_stance_s = 0.15;

    // The maximum amount of time to spend in stance.
    double max_stance_s = 0.40;

    // Ratio of time spent in stance vs swing.  Bigger numbers mean
    // more time spent in stance.
    double stance_swing_ratio = 0.75;

    // When a leg would become invalid in this many swing times, force
    // a step.
    double invalid_ratio = 1.5;

    // The fraction of a swing spent ramping in and out of the world
    // velocity.
    double world_blend_ratio = 0.15;


    // New trot gait parameters.
    double max_swing_time_s = 0.20;
    double min_swing_time_s = 0.15;
    double max_twovleg_time_s = 0.35;
    double max_flight_time_s = 0.02;
    double travel_ratio = 0.8;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(swing_time_s));
      a->Visit(MJ_NVP(lift_height));
      a->Visit(MJ_NVP(min_stance_s));
      a->Visit(MJ_NVP(max_stance_s));
      a->Visit(MJ_NVP(stance_swing_ratio));
      a->Visit(MJ_NVP(invalid_ratio));
      a->Visit(MJ_NVP(world_blend_ratio));

      a->Visit(MJ_NVP(max_swing_time_s));
      a->Visit(MJ_NVP(min_swing_time_s));
      a->Visit(MJ_NVP(max_twovleg_time_s));
      a->Visit(MJ_NVP(max_flight_time_s));
      a->Visit(MJ_NVP(travel_ratio));
    }
  };

  Walk walk;

  struct Backflip {
    double lower_height = 0.045;
    double pitch_accel_dps2 = 5000.0;
    double max_pitch_deg = 35.0;
    double push_pitch_deg = 55.0;

    double acceleration = 30.000;
    double push_height = 0.180;
    double flight_velocity = 0.500;
    double flight_kp = 0.5;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(lower_height));
      a->Visit(MJ_NVP(pitch_accel_dps2));
      a->Visit(MJ_NVP(max_pitch_deg));
      a->Visit(MJ_NVP(push_pitch_deg));
      a->Visit(MJ_NVP(acceleration));
      a->Visit(MJ_NVP(push_height));
      a->Visit(MJ_NVP(flight_velocity));
      a->Visit(MJ_NVP(flight_kp));
    }
  };

  Backflip backflip;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(period_s));
    a->Visit(MJ_NVP(joints));
    a->Visit(MJ_NVP(rezero_threshold_deg));
    a->Visit(MJ_NVP(legs));
    a->Visit(MJ_NVP(bounds));
    a->Visit(MJ_NVP(mass_kg));
    a->Visit(MJ_NVP(leg_mass_kg));
    a->Visit(MJ_NVP(center_of_mass_B));
    a->Visit(MJ_NVP(stand_up));
    a->Visit(MJ_NVP(rest));
    a->Visit(MJ_NVP(stand_height));
    a->Visit(MJ_NVP(idle_x));
    a->Visit(MJ_NVP(idle_y));
    a->Visit(MJ_NVP(default_kp_N_m));
    a->Visit(MJ_NVP(default_kd_N_m_s));
    a->Visit(MJ_NVP(rb_filter_constant_Hz));
    a->Visit(MJ_NVP(lr_acceleration));
    a->Visit(MJ_NVP(lr_alpha_rad_s2));
    a->Visit(MJ_NVP(terrain_filter_s));
    a->Visit(MJ_NVP(jump));
    a->Visit(MJ_NVP(walk));
    a->Visit(MJ_NVP(backflip));
  }
};

}
}
