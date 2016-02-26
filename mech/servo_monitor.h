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

#include "herkulex.h"

namespace mjmech {
namespace mech {
class ServoMonitor : boost::noncopyable {
 public:
  class HerkuleXServo {
   public:
    virtual ~HerkuleXServo() {}

    typedef boost::function<
      void (base::ErrorCode, HerkuleXBase::MemReadResponse)> MemReadHandler;

    virtual void RamRead(
        uint8_t servo, uint8_t reg, uint8_t length, MemReadHandler) = 0;

    virtual void ClearStatus(uint8_t servo, base::ErrorHandler) = 0;
  };

  template <typename T>
  class HerkuleXServoConcrete : public HerkuleXServo {
   public:
    HerkuleXServoConcrete(T* base) : base_(base) {}
    virtual ~HerkuleXServoConcrete() {}

    virtual void RamRead(
        uint8_t servo, uint8_t reg, uint8_t length, MemReadHandler handler) {
      base_->MemRead(HerkuleXBase::RAM_READ,
                     servo, reg, length, handler);
    }

    virtual void ClearStatus(uint8_t servo, base::ErrorHandler handler) {
      base_->MemWrite(HerkuleXBase::RAM_WRITE,
                      servo, HerkuleXConstants::status_error().position,
                      std::string(2, '\x00'), handler);
    }

    T* const base_;
  };

  template <typename Context>
  ServoMonitor(Context& context,
               HerkuleXServo* servo) :
      ServoMonitor(context.service, context.telemetry_registry.get(), servo) {}

  template <typename TelemetryRegistry>
  ServoMonitor(boost::asio::io_service& service,
               TelemetryRegistry* telemetry_registry,
               HerkuleXServo* servo)
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
               HerkuleXServo* servo);


  ServoDataSignal servo_data_signal_;

  class Impl;
  std::unique_ptr<Impl> impl_;
};
}
}
