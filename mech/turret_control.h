// Copyright 2020 Josh Pieper, jjp@pobox.com.
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

#include <functional>
#include <memory>
#include <string>

#include <clipp/clipp.h>

#include <boost/noncopyable.hpp>

#include "mjlib/base/visitor.h"

#include "base/context.h"

#include "mech/control_timing.h"
#include "mech/imu_client.h"
#include "mech/multiplex_client.h"

namespace mjmech {
namespace mech {

class TurretControl : boost::noncopyable {
 public:
  /// @param client_getter will be called at AsyncStart time
  using ClientGetter = std::function<mjlib::multiplex::AsioClient*()>;
  using ImuGetter = std::function<ImuClient*()>;
  TurretControl(base::Context&,
                ClientGetter client_getter, ImuGetter imu_getter);
  ~TurretControl();

  void AsyncStart(mjlib::io::ErrorCallback handler);

  struct Parameters {
    double period_s = 0.01;
    double max_torque_Nm = -1.0;
    std::string config;

    double command_timeout_s = 1.0;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(period_s));
      a->Visit(MJ_NVP(max_torque_Nm));
      a->Visit(MJ_NVP(config));
      a->Visit(MJ_NVP(command_timeout_s));
    }
  };

  enum class Mode {
    kStop = 0,
    kActive = 1,
    kFault = 2,
  };

  static inline std::map<Mode, const char*> ModeMapper() {
    return {
      { Mode::kStop, "stop" },
      { Mode::kActive, "active" },
      { Mode::kFault, "fault" },
          };
  };

  struct Status {
    boost::posix_time::ptime timestamp;
    Mode mode = Mode::kStop;

    std::string fault;

    int missing_replies = 0;
    ControlTiming::Status timing;

    struct GimbalServo {
      int id = 0;
      double angle_deg = 0.0;
      double velocity_dps = 0.0;
      double torque_Nm = 0.0;

      double temperature_C = 0.0;
      double voltage = 0.0;
      int32_t mode = 0;
      int32_t fault = 0;

      template <typename Archive>
      void Serialize(Archive* a) {
        a->Visit(MJ_NVP(id));
        a->Visit(MJ_NVP(angle_deg));
        a->Visit(MJ_NVP(velocity_dps));
        a->Visit(MJ_NVP(torque_Nm));
        a->Visit(MJ_NVP(temperature_C));
        a->Visit(MJ_NVP(voltage));
        a->Visit(MJ_NVP(mode));
        a->Visit(MJ_NVP(fault));
      }
    };

    GimbalServo pitch_servo;
    GimbalServo yaw_servo;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(timestamp));
      a->Visit(MJ_ENUM(mode, ModeMapper));
      a->Visit(MJ_NVP(fault));
      a->Visit(MJ_NVP(missing_replies));
      a->Visit(MJ_NVP(timing));
      a->Visit(MJ_NVP(pitch_servo));
      a->Visit(MJ_NVP(yaw_servo));
    }
  };

  struct CommandData {
    int priority = 0;

    Mode mode = Mode::kStop;

    double pitch_rate_dps = 0.0;
    double yaw_rate_dps = 0.0;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(priority));
      a->Visit(MJ_ENUM(mode, ModeMapper));
      a->Visit(MJ_NVP(pitch_rate_dps));
      a->Visit(MJ_NVP(yaw_rate_dps));
    }
  };

  const Status& status() const;
  void Command(const CommandData&);

  clipp::group program_options();

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};


}
}
