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

#include <memory>

#include <boost/noncopyable.hpp>

#include "mjlib/base/error_code.h"

#include "base/component_archives.h"
#include "base/context.h"

#include "mech/multiplex_client.h"
#include "mech/moteus_servo.h"

namespace mjmech {
namespace mech {

class JumpTest : boost::noncopyable {
 public:
  JumpTest(base::Context& context);
  ~JumpTest();

  void AsyncStart(mjlib::io::ErrorCallback);

  struct Members {
    std::unique_ptr<MultiplexClient> multiplex_client;
    std::unique_ptr<MoteusServo> moteus_servo;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(multiplex_client));
      a->Visit(MJ_NVP(moteus_servo));
    }
  };

  struct JointSetup {
    double shoulder = 0.0;
    double femur = 0.0;
    double tibia = 0.0;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(shoulder));
      a->Visit(MJ_NVP(femur));
      a->Visit(MJ_NVP(tibia));
    }
  };

  struct Parameters {
    bool skip_powered = false;
    double period_s = 0.05;

    JointSetup preposition_deg = { 0, 60, 120 };
    double preposition_max_torque_Nm = 2;
    double preposition_speed_dps = 30.0;
    double preposition_time_s = 4.0;

    JointSetup standing_deg = { 0, 5, 10 };
    double standing_speed_dps = 20.0;
    double standing_time_s = 4.0;

    JointSetup squat_deg = { 0, 40, 80 };
    double squat_dps = 20.0;
    double squat_time_s = 4.0;

    JointSetup jump_deg = { 0, 0, 0 };
    double jump_max_torque_Nm = 50;
    double jump_dps = 600.0;

    JointSetup landing_prepare_deg = { 0, 20, 40 };
    double landing_prepare_dps = 800.0;
    double landing_prepare_time_s = 2.0;

    double landing_kp = 0.6;
    double landing_threshold_dps = 20.0;
    double landing_torque_Nm = 1.0;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(skip_powered));
      a->Visit(MJ_NVP(period_s));
      a->Visit(MJ_NVP(preposition_deg));
      a->Visit(MJ_NVP(preposition_max_torque_Nm));
      a->Visit(MJ_NVP(preposition_speed_dps));
      a->Visit(MJ_NVP(preposition_time_s));
      a->Visit(MJ_NVP(standing_deg));
      a->Visit(MJ_NVP(standing_speed_dps));
      a->Visit(MJ_NVP(standing_time_s));
      a->Visit(MJ_NVP(squat_deg));
      a->Visit(MJ_NVP(squat_dps));
      a->Visit(MJ_NVP(squat_time_s));

      a->Visit(MJ_NVP(jump_deg));
      a->Visit(MJ_NVP(jump_max_torque_Nm));
      a->Visit(MJ_NVP(jump_dps));

      a->Visit(MJ_NVP(landing_prepare_deg));
      a->Visit(MJ_NVP(landing_prepare_dps));
      a->Visit(MJ_NVP(landing_prepare_time_s));

      a->Visit(MJ_NVP(landing_kp));
      a->Visit(MJ_NVP(landing_threshold_dps));
      a->Visit(MJ_NVP(landing_torque_Nm));
    }
  };

  Parameters* parameters() { return &parameters_; }
  clipp::group program_options();

 private:
  Members m_;
  Parameters parameters_;
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}
}
