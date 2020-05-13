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

#include "base/context.h"

#include "mech/camera_driver.h"
#include "mech/control_timing.h"
#include "mech/imu_client.h"
#include "mech/multiplex_client.h"
#include "mech/target_tracker.h"

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
    mjlib::base::PID::Config pitch;
    mjlib::base::PID::Config yaw;

    double servo_pitch_min_deg = -15.0;
    double servo_pitch_max_deg = 10.0;

    double imu_servo_filter_s = 2.0;
    double command_timeout_s = 1.0;

    CameraDriver::Options camera;
    TargetTracker::Options tracker;

    double max_rate_accel_dps2 = 1000.0;

    double target_gain_dps = 10.0;
    double target_timeout_s = 0.3;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(period_s));
      a->Visit(MJ_NVP(max_torque_Nm));
      a->Visit(MJ_NVP(config));
      a->Visit(MJ_NVP(pitch));
      a->Visit(MJ_NVP(yaw));
      a->Visit(MJ_NVP(servo_pitch_min_deg));
      a->Visit(MJ_NVP(servo_pitch_max_deg));
      a->Visit(MJ_NVP(imu_servo_filter_s));
      a->Visit(MJ_NVP(command_timeout_s));
      a->Visit(MJ_NVP(camera));
      a->Visit(MJ_NVP(tracker));
      a->Visit(MJ_NVP(max_rate_accel_dps2));
      a->Visit(MJ_NVP(target_gain_dps));
      a->Visit(MJ_NVP(target_timeout_s));
    }

    Parameters() {
      pitch.kp = 0.05;
      pitch.kd = 0.005;
      pitch.ki = 0.01;
      pitch.ilimit = 0.1;
      pitch.sign = -1.0;
      pitch.max_desired_rate = 100.0;

      yaw.kp = 0.05;
      yaw.kd = 0.005;
      yaw.ki = 0.0;
      yaw.ilimit = 0.0;
      yaw.sign = -1.0;
      yaw.max_desired_rate = 0.0;  // disable this
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

    struct ServoControl {
      double angle_deg = 0.0;
      double rate_dps = 0.0;
      mjlib::base::PID::State pid;

      template <typename Archive>
      void Serialize(Archive* a) {
        a->Visit(MJ_NVP(angle_deg));
        a->Visit(MJ_NVP(rate_dps));
        a->Visit(MJ_NVP(pid));
      }
    };

    struct Control {
      ServoControl pitch;
      ServoControl yaw;

      template <typename Archive>
      void Serialize(Archive* a) {
        a->Visit(MJ_NVP(pitch));
        a->Visit(MJ_NVP(yaw));
      }
    };

    Control control;

    struct Imu {
      double pitch_deg = 0.0;
      double pitch_rate_dps = 0.0;
      double yaw_deg = 0.0;
      double yaw_rate_dps = 0.0;

      template <typename Archive>
      void Serialize(Archive* a) {
        a->Visit(MJ_NVP(pitch_deg));
        a->Visit(MJ_NVP(pitch_rate_dps));
        a->Visit(MJ_NVP(yaw_deg));
        a->Visit(MJ_NVP(yaw_rate_dps));
      }
    };

    Imu imu;

    double imu_servo_pitch_deg = 0.0;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(timestamp));
      a->Visit(MJ_NVP(mode));
      a->Visit(MJ_NVP(fault));
      a->Visit(MJ_NVP(missing_replies));
      a->Visit(MJ_NVP(timing));
      a->Visit(MJ_NVP(pitch_servo));
      a->Visit(MJ_NVP(yaw_servo));
      a->Visit(MJ_NVP(control));
      a->Visit(MJ_NVP(imu));
      a->Visit(MJ_NVP(imu_servo_pitch_deg));
    }
  };

  struct CommandData {
    int priority = 0;

    Mode mode = Mode::kStop;

    double pitch_rate_dps = 0.0;
    double yaw_rate_dps = 0.0;
    bool track_target = false;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(priority));
      a->Visit(MJ_NVP(mode));
      a->Visit(MJ_NVP(pitch_rate_dps));
      a->Visit(MJ_NVP(yaw_rate_dps));
      a->Visit(MJ_NVP(track_target));
    }
  };

  struct ControlData {
    struct Servo {
      bool power = false;
      double torque_Nm = 0.0;

      template <typename Archive>
      void Serialize(Archive* a) {
        a->Visit(MJ_NVP(power));
        a->Visit(MJ_NVP(torque_Nm));
      }
    };

    Servo pitch;
    Servo yaw;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(pitch));
      a->Visit(MJ_NVP(yaw));
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
struct IsEnum<mjmech::mech::TurretControl::Mode> {
  static constexpr bool value = true;

  using M = mjmech::mech::TurretControl::Mode;
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
