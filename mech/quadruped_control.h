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

#include <clipp/clipp.h>

#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <boost/noncopyable.hpp>

#include "mjlib/base/visitor.h"

#include "base/context.h"

#include "mech/imu_client.h"
#include "mech/moteus_servo.h"
#include "mech/multiplex_client.h"
#include "mech/quadruped_command.h"
#include "mech/quadruped_state.h"

namespace mjmech {
namespace mech {

/// This sequences the primary control modes of the quadruped.
class QuadrupedControl : boost::noncopyable {
 public:
  /// @param client_getter will be called at AsyncStart time
  using ClientGetter = std::function<mjlib::multiplex::AsioClient*()>;
  using ImuGetter = std::function<ImuClient*()>;
  QuadrupedControl(base::Context&,
                   ClientGetter client_getter, ImuGetter imu_getter);
  ~QuadrupedControl();

  void AsyncStart(mjlib::io::ErrorCallback handler);

  struct Parameters {
    double period_s = 0.01;
    double max_torque_Nm = -1.0;
    std::string config;

    bool enable_imu = true;
    std::string imu_device = "/dev/spidev0.0";
    int imu_speed = 10000000;
    int imu_cpu_affinity = -1;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(period_s));
      a->Visit(MJ_NVP(max_torque_Nm));
      a->Visit(MJ_NVP(config));
      a->Visit(MJ_NVP(enable_imu));
      a->Visit(MJ_NVP(imu_device));
      a->Visit(MJ_NVP(imu_speed));
      a->Visit(MJ_NVP(imu_cpu_affinity));
    }
  };

  struct Status {
    boost::posix_time::ptime timestamp;

    QuadrupedCommand::Mode mode = QuadrupedCommand::Mode::kConfiguring;
    boost::posix_time::ptime mode_start;
    std::string fault;

    QuadrupedState state;

    int missing_replies = 0;
    double time_status_s = 0.0;
    double time_control_s = 0.0;
    double time_command_s = 0.0;
    double time_cycle_s = 0.0;
    double time_delta_s = 0.0;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(timestamp));
      a->Visit(MJ_ENUM(mode, QuadrupedCommand::ModeMapper));
      a->Visit(MJ_NVP(mode_start));
      a->Visit(MJ_NVP(fault));
      a->Visit(MJ_NVP(state));
      a->Visit(MJ_NVP(missing_replies));
      a->Visit(MJ_NVP(time_status_s));
      a->Visit(MJ_NVP(time_control_s));
      a->Visit(MJ_NVP(time_command_s));
      a->Visit(MJ_NVP(time_cycle_s));
      a->Visit(MJ_NVP(time_delta_s));
    }
  };

  void Command(const QuadrupedCommand&);
  const Status& status() const;

  clipp::group program_options();

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}
}
