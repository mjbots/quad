// Copyright 2015-2019 Josh Pieper, jjp@pobox.com.  All rights reserved.
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

#include <map>
#include <optional>

#include <boost/asio.hpp>
#include <boost/program_options.hpp>
#include <boost/signals2/signal.hpp>

#include "mjlib/base/visitor.h"

#include "base/linux_input.h"

#include "drive_command.h"
#include "gait.h"
#include "turret_command.h"

namespace mjmech {
namespace mech {
namespace mw_command {

struct MechMessage {
  Command gait;
  TurretCommand turret;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(gait));
    a->Visit(MJ_NVP(turret));
  }
};

struct OptOptions {
  double period_s = 0.1;
  double deadband = 0.20;
  double max_translate_x_mm_s = 200.0;
  double max_translate_y_mm_s = 200.0;
  double max_rotate_deg_s = 100.0;
  double max_body_z_mm = 30.0;
  double min_body_z_mm = -60.0;
  double idle_body_y_mm = 0.0;
  double forward_body_y_mm = 0.0;
  double reverse_body_y_mm = 0.0;
  double max_body_x_mm = 30;
  double max_body_y_mm = 30;
  double max_body_pitch_deg = 20;
  double max_body_roll_deg = 20;
  double max_turret_rate_deg_s = 100;
  double turret_linear_transition_point = 0.5;
  double turret_linear_fine_percent = 0.2;
  double target_max_rate = 100.0;
  bool manual_agitator = false;
  bool verbose = false;
  bool manual_drive = false;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(period_s));
    a->Visit(MJ_NVP(deadband));
    a->Visit(MJ_NVP(max_translate_x_mm_s));
    a->Visit(MJ_NVP(max_translate_y_mm_s));
    a->Visit(MJ_NVP(max_rotate_deg_s));
    a->Visit(MJ_NVP(max_body_z_mm));
    a->Visit(MJ_NVP(min_body_z_mm));
    a->Visit(MJ_NVP(idle_body_y_mm));
    a->Visit(MJ_NVP(forward_body_y_mm));
    a->Visit(MJ_NVP(reverse_body_y_mm));
    a->Visit(MJ_NVP(max_body_x_mm));
    a->Visit(MJ_NVP(max_body_y_mm));
    a->Visit(MJ_NVP(max_body_pitch_deg));
    a->Visit(MJ_NVP(max_body_roll_deg));
    a->Visit(MJ_NVP(max_turret_rate_deg_s));
    a->Visit(MJ_NVP(turret_linear_transition_point));
    a->Visit(MJ_NVP(turret_linear_fine_percent));
    a->Visit(MJ_NVP(target_max_rate));
    a->Visit(MJ_NVP(manual_agitator));
    a->Visit(MJ_NVP(verbose));
    a->Visit(MJ_NVP(manual_drive));
  }
};

class Commander {
 public:
  Commander(const boost::asio::executor&);
  virtual ~Commander();

  void AsyncStart(mjlib::io::ErrorCallback);
  void SendMechMessage(const MechMessage&);

  struct Parameters {
    std::string target = "192.168.0.123";
    // send live commands from joystick at this device
    std::string joystick = "";
    Command cmd;
    OptOptions opt;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(target));
      a->Visit(MJ_NVP(joystick));
      a->Visit(MJ_NVP(cmd));
      a->Visit(MJ_NVP(opt));
    }
  };

  Parameters* parameters() { return &parameters_; }
  boost::program_options::options_description* options();

  typedef boost::signals2::signal<void (int x, int y)> TargetOffsetSignal;
  TargetOffsetSignal* target_offset_signal();

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
  Parameters parameters_;
};


}
}
}
