// Copyright 2015-2016 Josh Pieper, jjp@pobox.com.  All rights reserved.
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

#include "base/euler.h"
#include "base/point3d.h"
#include "turret_command.h"

namespace mjmech {
namespace mech {

struct DriveCommand {
  /// Body coordinates relative to camera field of view.
  base::Point3D drive_mm_s;
  base::Euler turret_rate_dps;

  base::Point3D body_offset_mm;
  base::Euler body_attitude_deg;

  TurretCommand::FireControl fire_control;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(drive_mm_s));
    a->Visit(MJ_NVP(turret_rate_dps));
    a->Visit(MJ_NVP(body_offset_mm));
    a->Visit(MJ_NVP(body_attitude_deg));
    a->Visit(MJ_NVP(fire_control));
  }
};

}
}
