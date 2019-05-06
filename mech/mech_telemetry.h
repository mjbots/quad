// Copyright 2014-2016 Josh Pieper, jjp@pobox.com.  All rights reserved.
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

#include "target_tracker_data.h"

namespace mjmech {
namespace mech {

/// This structure is sent from the primary mech application over the
/// mcast link to the controller.  It should be small, because it will
/// be sent at a high rate and will add to overall video load.
struct MechTelemetry {
  boost::posix_time::ptime timestamp;

  float servo_min_voltage_V = 0.0;
  float servo_max_voltage_V = 0.0;
  float servo_min_temp_C = 0.0;
  float servo_max_temp_C = 0.0;

  float turret_absolute_deg = 0.0;

  float total_fire_time_s = 0.0;

  // An enum from MechWarfare::Impl::Data::Mode
  uint8_t mech_mode = 0;

  TargetTrackerData target_data;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(timestamp));
    a->Visit(MJ_NVP(servo_min_voltage_V));
    a->Visit(MJ_NVP(servo_max_voltage_V));
    a->Visit(MJ_NVP(servo_min_temp_C));
    a->Visit(MJ_NVP(servo_max_temp_C));
    a->Visit(MJ_NVP(turret_absolute_deg));
    a->Visit(MJ_NVP(total_fire_time_s));
    a->Visit(MJ_NVP(mech_mode));
    a->Visit(MJ_NVP(target_data));
  }
};

}
}
