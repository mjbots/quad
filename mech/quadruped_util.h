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

#include "mjlib/base/fail.h"

#include "mech/quadruped_command.h"

namespace mjmech {
namespace mech {

template <typename T>
QuadrupedCommand::Leg& GetLeg_R(T* legs_R, int id) {
  for (auto& leg_R : *legs_R) {
    if (leg_R.leg_id == id) { return leg_R; }
  }
  mjlib::base::AssertNotReached();
}

}
}
