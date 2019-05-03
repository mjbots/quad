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

#include <boost/asio/io_service.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <boost/signals2/signal.hpp>

#include "mjlib/base/visitor.h"
#include "mjlib/io/async_types.h"

#include "mech/servo_interface.h"

namespace mjmech {
namespace mech {
class ServoMonitor : boost::noncopyable {
 public:
  template <typename Context>
  ServoMonitor(Context& context,
               ServoInterface* servo) :
      ServoMonitor(context.service, context.telemetry_registry.get(), servo) {}

  template <typename TelemetryRegistry>
  ServoMonitor(boost::asio::io_service& service,
               TelemetryRegistry* telemetry_registry,
               ServoInterface* servo)
      : ServoMonitor(service, servo) {
    telemetry_registry->Register("servo", &servo_data_signal_);
  }

  ~ServoMonitor();

  void AsyncStart(mjlib::io::ErrorCallback handler);

  struct Parameters {
    /// Query individual servos at this rate.
    double period_s = 0.5;

    /// If a servo times out a response, then exclude it from queries.
    /// The time to exclude starts at parole_min_time_s, then doubles
    /// with each timeout until it reaches parole_max_time_s.
    double parole_min_time_s = 60.0;
    double parole_max_time_s = 600.0;

    /// A list of servos to monitor.  Comma separated with optional -
    /// ranges, like: "1,2-5,9-12"
    std::string servos;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(period_s));
      a->Visit(MJ_NVP(parole_min_time_s));
      a->Visit(MJ_NVP(parole_max_time_s));
      a->Visit(MJ_NVP(servos));
    }
  };

  Parameters* parameters();

  void ExpectTorqueOn();
  void ExpectTorqueOff();

  /// Given a string in the format of Parameters::servos, return a
  /// list of integer servo ids that it represents.
  static std::vector<int> SplitServoIds(const std::string&);

  struct ServoData {
    boost::posix_time::ptime timestamp;

    struct Servo {
      boost::posix_time::ptime last_update;
      int address = -1;
      double voltage_V = 0.0;
      double temperature_C = 0.0;
      bool torque_on = false;

      template <typename Archive>
      void Serialize(Archive* a) {
        a->Visit(MJ_NVP(last_update));
        a->Visit(MJ_NVP(address));
        a->Visit(MJ_NVP(voltage_V));
        a->Visit(MJ_NVP(temperature_C));
        a->Visit(MJ_NVP(torque_on));
      }
    };

    std::vector<Servo> servos;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(timestamp));
      a->Visit(MJ_NVP(servos));
    }
  };

  typedef boost::signals2::signal<void (const ServoData*)> ServoDataSignal;

  ServoDataSignal* data_signal() { return &servo_data_signal_; }

 private:
  ServoMonitor(boost::asio::io_service& service,
               ServoInterface* servo);


  ServoDataSignal servo_data_signal_;

  class Impl;
  std::unique_ptr<Impl> impl_;
};
}
}
