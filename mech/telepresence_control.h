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

#include "mjlib/base/pid.h"
#include "mjlib/base/visitor.h"
#include "mjlib/multiplex/asio_client.h"

#include "base/context.h"

#include "mech/camera_driver.h"
#include "mech/control_timing.h"
#include "mech/imu_client.h"
#include "mech/target_tracker.h"

namespace mjmech {
namespace mech {

class TelepresenceControl : boost::noncopyable {
 public:
  /// @param client_getter will be called at AsyncStart time
  using ClientGetter = std::function<mjlib::multiplex::AsioClient*()>;
  TelepresenceControl(base::Context&, ClientGetter client_getter);
  ~TelepresenceControl();

  void AsyncStart(mjlib::io::ErrorCallback handler);

  struct Parameters {
    double period_s = 0.01;
    double max_torque_Nm = -1.0;
    double command_timeout_s = 1.0;
    mjlib::base::PID::Config pid;

    int id1 = 0;
    int id2 = 0;
    double id1_scale = 1.0;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(period_s));
      a->Visit(MJ_NVP(max_torque_Nm));
      a->Visit(MJ_NVP(command_timeout_s));
      a->Visit(MJ_NVP(pid));
      a->Visit(MJ_NVP(id1));
      a->Visit(MJ_NVP(id2));
      a->Visit(MJ_NVP(id1_scale));
    }

    Parameters() {
      pid.kp = 0.0;
      pid.kd = 0.0;
      pid.ki = 0.0;
    }
  };

  enum class Mode {
    kStop = 0,
    kActive = 1,
    kFault = 2,
  };

  struct Status {
    boost::posix_time::ptime timestamp;
    Mode mode = Mode::kStop;

    std::string fault;

    int missing_replies = 0;
    ControlTiming::Status timing;

    struct Servo {
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

    Servo servo1;
    Servo servo2;

    mjlib::base::PID::State pid;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(timestamp));
      a->Visit(MJ_NVP(mode));
      a->Visit(MJ_NVP(fault));
      a->Visit(MJ_NVP(missing_replies));
      a->Visit(MJ_NVP(timing));
      a->Visit(MJ_NVP(servo1));
      a->Visit(MJ_NVP(servo2));
      a->Visit(MJ_NVP(pid));
    }
  };

  struct CommandData {
    int priority = 0;

    Mode mode = Mode::kStop;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(priority));
      a->Visit(MJ_NVP(mode));
    }
  };

  struct ControlData {
    bool power = false;

    struct Servo {
      double torque_Nm = 0.0;
      double velocity_dps = 0.0;

      template <typename Archive>
      void Serialize(Archive* a) {
        a->Visit(MJ_NVP(torque_Nm));
        a->Visit(MJ_NVP(velocity_dps));
      }
    };

    Servo servo1;
    Servo servo2;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(power));
      a->Visit(MJ_NVP(servo1));
      a->Visit(MJ_NVP(servo2));
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

namespace mjlib {
namespace base {

template <>
struct IsEnum<mjmech::mech::TelepresenceControl::Mode> {
  static constexpr bool value = true;

  using M = mjmech::mech::TelepresenceControl::Mode;
  static inline std::map<M, const char*> map() {
    return {
      { M::kStop, "stop" },
      { M::kActive, "active" },
      { M::kFault, "fault" },
    };
  }
};

}
}
