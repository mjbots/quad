// Copyright 2015 Josh Pieper, jjp@pobox.com.  All rights reserved.
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

#include "herkulex.h"

namespace legtool {
HerkuleXConstants::HerkuleXConstants()
    : ram_registers{
  { "temperature_c", temperature_c() },
  { "voltage", voltage() },
  { "position", position() },
  { "pwm", pwm() },
  { "acc_ratio", Register{8, 1} },
  { "acc_max_time", Register{9, 1} },
  { "dead_zone", Register{10, 1} }, // when force is not applied
  { "sat_offset", Register{11, 1} },
  { "sat_slope", Register{12, 2} },
  { "pwm_offset", Register{14, 1} }, // manual 'I' term
  { "pwm_min", Register{15, 1} }, // overcome friction
  { "pwm_max", Register{15, 2} }, // save battery
  { "overload_pwm", Register{18, 2} }, // overload error setup
  { "overload_time", Register{42, 1} },
  { "position_kp", Register{24, 2} },
  { "position_kd", Register{26, 2} },
  { "position_ki", Register{28, 2} },
  { "position_ff_kd", Register{30, 2} },
  { "position_ff_kdd", Register{32, 2} },
  { "stop_threshold", Register{43, 1} },
  { "stop_time", Register{41, 1} },
  { "inpos_margin", Register{44, 1} },
  { "cal_diff", cal_diff() },
  { "torque_control", torque_control() },
      }
{
}
}
