// Copyright 2014-2019 Josh Pieper, jjp@pobox.com.
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

#include "mjlib/base/visitor.h"

namespace mjmech {
namespace mech {

struct Command {
  double translate_x_mm_s = 0;
  double translate_y_mm_s = 0;
  double rotate_deg_s = 0;
  double body_x_mm = 0;
  double body_y_mm = 0;
  double body_z_mm = 0;
  double body_pitch_deg = 0;
  double body_roll_deg = 0;
  double body_yaw_deg = 0;
  double lift_height_percent = 100;
  double time_rate_percent = 100;
  bool reset_phase = false;

  struct RobotFrameLeg {
    int leg_num = 0;
    base::Point3D point;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(leg_num));
      a->Visit(MJ_NVP(point));
    }
  };

  /// If set, then the given legs will move to this robot frame
  /// position during each swing phase.
  std::vector<RobotFrameLeg> override_foot_placement;

  bool IsZero() const {
    return translate_x_mm_s == 0.0 &&
        translate_y_mm_s == 0.0 &&
        rotate_deg_s == 0.0;
  }

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(translate_x_mm_s));
    a->Visit(MJ_NVP(translate_y_mm_s));
    a->Visit(MJ_NVP(rotate_deg_s));
    a->Visit(MJ_NVP(body_x_mm));
    a->Visit(MJ_NVP(body_y_mm));
    a->Visit(MJ_NVP(body_z_mm));
    a->Visit(MJ_NVP(body_pitch_deg));
    a->Visit(MJ_NVP(body_roll_deg));
    a->Visit(MJ_NVP(body_yaw_deg));
    a->Visit(MJ_NVP(lift_height_percent));
    a->Visit(MJ_NVP(time_rate_percent));
    a->Visit(MJ_NVP(reset_phase));
    a->Visit(MJ_NVP(override_foot_placement));
  }
};

}
}
