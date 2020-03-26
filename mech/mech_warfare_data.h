// Copyright 2014-2016 Josh Pieper, jjp@pobox.com.
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

#include <boost/date_time/posix_time/posix_time_types.hpp>

#include "drive_command.h"

namespace mjmech {
namespace mech {

struct MechWarfareData {
  boost::posix_time::ptime timestamp;

  enum class Mode {
    kIdle,
    kTurretBias,
    kPrepositioning,
    kStanding,
    kManual,
    kDrive,
    kPreparingToSit,
    kSitting,
    kFault,
  };

  Mode mode = Mode::kIdle;

  DriveCommand current_drive;
  boost::posix_time::ptime last_command_timestamp;
  boost::posix_time::ptime turret_bias_start_timestamp;
  boost::posix_time::ptime sitting_start_timestamp;
  boost::posix_time::ptime prepositioning_start_timestamp;
  boost::posix_time::ptime standing_start_timestamp;
  bool active = false;
  uint64_t skipped_updates = 0;

  static std::map<Mode, const char*> ModeMapper() {
    return std::map<Mode, const char*>{
      {Mode::kIdle, "kIdle"},
      {Mode::kTurretBias, "kTurretBias"},
      {Mode::kPrepositioning, "kPrepositioning"},
      {Mode::kStanding, "kStanding"},
      {Mode::kManual, "kManual"},
      {Mode::kDrive, "kDrive"},
      {Mode::kPreparingToSit, "kPreparingToSit"},
      {Mode::kSitting, "kSitting"},
      {Mode::kFault, "kFault"},
    };
  }

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(timestamp));
    a->Visit(MJ_ENUM(mode, ModeMapper));
    a->Visit(MJ_NVP(current_drive));
    a->Visit(MJ_NVP(last_command_timestamp));
    a->Visit(MJ_NVP(turret_bias_start_timestamp));
    a->Visit(MJ_NVP(sitting_start_timestamp));
    a->Visit(MJ_NVP(prepositioning_start_timestamp));
    a->Visit(MJ_NVP(standing_start_timestamp));
    a->Visit(MJ_NVP(active));
    a->Visit(MJ_NVP(skipped_updates));
  }
};


}
}
