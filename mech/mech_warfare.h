// Copyright 2015-2016 Josh Pieper, jjp@pobox.com.  All rights reserved.
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

#include <boost/asio/ip/udp.hpp>
#include <boost/property_tree/ptree.hpp>

#include "base/component_archives.h"

#include "ahrs.h"
#include "gait_driver.h"
#include "mech_defines.h"
#include "mjmech_imu_driver.h"
#include "ripple.h"
#include "servo_monitor.h"
#include "turret.h"
#include "video_sender_app.h"

namespace mjmech {
namespace base {
struct Context;
}
namespace mech {
/// Accepts json formatted commands over the network and uses that to
/// sequence gaits and firing actions.
class MechWarfare : boost::noncopyable {
 public:
  MechWarfare(base::Context& context);
  ~MechWarfare();

  void AsyncStart(base::ErrorHandler handler);

  struct Members {
    std::unique_ptr<Mech::ServoBase> servo_base;
    std::unique_ptr<Mech::Servo> servo;
    std::unique_ptr<MjmechImuDriver> imu;
    std::unique_ptr<Ahrs> ahrs;
    std::unique_ptr<GaitDriver> gait_driver;
    std::unique_ptr<ServoMonitor::HerkuleXServo> servo_iface;
    std::unique_ptr<ServoMonitor> servo_monitor;
    std::unique_ptr<Turret> turret;
    std::unique_ptr<VideoSenderApp> video;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(servo_base));
      a->Visit(MJ_NVP(servo));
      a->Visit(MJ_NVP(imu));
      a->Visit(MJ_NVP(ahrs));
      a->Visit(MJ_NVP(gait_driver));
      a->Visit(MJ_NVP(servo_monitor));
      a->Visit(MJ_NVP(turret));
      a->Visit(MJ_NVP(video));
    }
  };

  struct Parameters {
    int port = 13356;
    std::string gait_config;
    double period_s = 0.05;
    double idle_timeout_s = 1.0;
    double drive_rotate_factor = 0.5;
    double drive_max_rotate_dps = 40.0;

    base::ComponentParameters<Members> children;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(port));
      a->Visit(MJ_NVP(gait_config));
      a->Visit(MJ_NVP(period_s));
      a->Visit(MJ_NVP(idle_timeout_s));
      a->Visit(MJ_NVP(drive_rotate_factor));
      a->Visit(MJ_NVP(drive_max_rotate_dps));
      children.Serialize(a);
    }

    Parameters(Members* m) : children(m) {}
  };

  Parameters* parameters() { return &parameters_; }

 private:
  boost::asio::io_service& service_;

  Members m_;
  Parameters parameters_{&m_};
  class Impl;
  std::unique_ptr<Impl> impl_;

};
}
}
