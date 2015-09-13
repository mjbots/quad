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

#include <boost/asio/io_service.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <boost/signals2/signal.hpp>

#include "base/comm.h"
#include "base/visitor.h"

namespace mjmech {
namespace mech {
class ServoInterface;

class ServoMonitor : boost::noncopyable {
 public:
  template <typename Context>
  ServoMonitor(Context& context,
               ServoInterface* servo) :
      ServoMonitor(context.service, &context.telemetry_registry, servo) {}

  template <typename TelemetryRegistry>
  ServoMonitor(boost::asio::io_service& service,
               TelemetryRegistry* telemetry_registry,
               ServoInterface* servo)
      : ServoMonitor(service, servo) {
    telemetry_registry->Register("servo", &servo_data_signal_);
  }

  ~ServoMonitor();

  void AsyncStart(base::ErrorHandler handler);

  struct Parameters {
    /// Query individual servos at this rate.
    double period_s = 0.5;

    /// If a servo times out a response, then exclude it from queries.
    /// The time to exclude starts at parole_min_time_s, then doubles
    /// with each timeout until it reaches parole_max_time_s.
    double parole_min_time_s = 60.0;
    double parole_max_time_s = 600.0;

    /// ServoMonitor will watch any servo that another process
    /// communicates with, but you can seed it with additional ones
    /// using this comma separated optionally ranged list (0,3-5,9).
    std::string servos;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(LT_NVP(period_s));
      a->Visit(LT_NVP(parole_min_time_s));
      a->Visit(LT_NVP(parole_max_time_s));
      a->Visit(LT_NVP(servos));
    }
  };

  Parameters* parameters();

 private:
  ServoMonitor(boost::asio::io_service& service,
               ServoInterface* servo);

  struct ServoData {
    boost::posix_time::ptime timestamp;

    struct Servo {
      boost::posix_time::ptime last_update;
      int address = -1;
      double voltage_V = 0.0;
      double temperature_C = 0.0;

      template <typename Archive>
      void Serialize(Archive* a) {
        a->Visit(LT_NVP(last_update));
        a->Visit(LT_NVP(address));
        a->Visit(LT_NVP(voltage_V));
        a->Visit(LT_NVP(temperature_C));
      }
    };

    std::vector<Servo> servos;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(LT_NVP(timestamp));
      a->Visit(LT_NVP(servos));
    }
  };

  boost::signals2::signal<void (const ServoData*)> servo_data_signal_;

  class Impl;
  std::unique_ptr<Impl> impl_;
};
}
}
