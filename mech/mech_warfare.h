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

namespace mjmech {
namespace mech {
/// Accepts json formatted commands over the network and uses that to
/// sequence gaits and firing actions.
///
/// NOTE jpieper: This could also manage video if we had a way of
/// managing it from C++.
class MechWarfare : boost::noncopyable {
 public:
  template <typename Context>
  MechWarfare(Context& context) : MechWarfare(context.service) {
    m_.servo_base.reset(new Mech::ServoBase(service_, factory_));
    m_.servo.reset(new Mech::Servo(m_.servo_base.get()));
    m_.imu.reset(new MjmechImuDriver(context));
    m_.ahrs.reset(new Ahrs(context, m_.imu->imu_data_signal()));
    m_.gait_driver.reset(new GaitDriver(service_,
                                        &context.telemetry_registry,
                                        m_.servo.get(),
                                        m_.ahrs->ahrs_data_signal()));
    m_.servo_monitor.reset(new ServoMonitor(context, m_.servo.get()));
    m_.turret.reset(new Turret(context, m_.servo_base.get()));
  }

  MechWarfare(boost::asio::io_service&);
  ~MechWarfare() {}

  void AsyncStart(base::ErrorHandler handler);

  struct Members {
    std::unique_ptr<Mech::ServoBase> servo_base;
    std::unique_ptr<Mech::Servo> servo;
    std::unique_ptr<MjmechImuDriver> imu;
    std::unique_ptr<Ahrs> ahrs;
    std::unique_ptr<GaitDriver> gait_driver;
    std::unique_ptr<ServoMonitor> servo_monitor;
    std::unique_ptr<Turret> turret;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(servo_base));
      a->Visit(MJ_NVP(servo));
      a->Visit(MJ_NVP(imu));
      a->Visit(MJ_NVP(ahrs));
      a->Visit(MJ_NVP(gait_driver));
      a->Visit(MJ_NVP(servo_monitor));
      a->Visit(MJ_NVP(turret));
    }
  };

  struct Parameters {
    int port = 13356;
    std::string gait_config;

    base::ComponentParameters<Members> children;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(port));
      a->Visit(MJ_NVP(gait_config));
      children.Serialize(a);
    }

    Parameters(Members* m) : children(m) {}
  };

  Parameters* parameters() { return &parameters_; }

 private:
  RippleConfig LoadRippleConfig();
  void NetworkListen();
  void StartRead();
  void HandleRead(base::ErrorCode, std::size_t);
  void HandleMessage(const boost::property_tree::ptree&);
  void HandleMessageGait(const boost::property_tree::ptree&);
  void HandleMessageTurret(const boost::property_tree::ptree&);

  boost::asio::io_service& service_;

  Mech::Factory factory_;

  Members m_;
  Parameters parameters_{&m_};
  boost::asio::ip::udp::socket server_;
  char receive_buffer_[3000] = {};
  boost::asio::ip::udp::endpoint receive_endpoint_;
};
}
}
