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

#include "turret.h"

#include <boost/asio/deadline_timer.hpp>

#include "base/common.h"
#include "base/fail.h"

namespace mjmech {
namespace mech {

namespace {

class Parser {
 public:
  Parser(const Mech::ServoBase::MemReadResponse& response)
      : response_(response) {}

  uint8_t get(int index) const {
    return static_cast<uint8_t>(response_.register_data.at(index));
  };

  int32_t get_int32(int index) {
    uint32_t unextended = (get(index) |
                           (get(index + 1) << 7) |
                           (get(index + 2) << 14) |
                           (get(index + 3) << 21));
    if (unextended >= 0x8000000) {
      return unextended - (0x80 << 21);
    } else {
      return unextended;
    }
  };

 private:
  const Mech::ServoBase::MemReadResponse& response_;
};
}

class Turret::Impl : boost::noncopyable {
 public:
  Impl(Turret* parent,
       boost::asio::io_service& service,
       Mech::ServoBase* servo)
      : parent_(parent),
        service_(service),
        servo_(servo),
        timer_(service_) {}

  void StartTimer() {
    timer_.expires_from_now(
        base::ConvertSecondsToDuration(parameters_.period_s));
    timer_.async_wait(std::bind(&Impl::HandleTimer, this,
                                std::placeholders::_1));
  }

  void HandleTimer(const base::ErrorCode& ec) {
    if (ec == boost::asio::error::operation_aborted) { return; }
    FailIf(ec);
    StartTimer();

    DoPoll();
  }

  void DoPoll() {
    // If we don't currently know it, ask for the current command.
    if (!data_.imu_command) {
      const int kAddressPitchCommand = 0x50;
      servo_->MemRead(
          servo_->RAM_READ, parameters_.gimbal_address,
          kAddressPitchCommand, 8,
          [this](base::ErrorCode ec, Mech::ServoBase::MemReadResponse response) {
            HandleCommand(ec, response);
          });
    }

    // If we are doing a rate motion, and know the current command,
    // send our updated command.

    // TODO jpieper: Do this.


    // Then, ask for IMU and absolute coordinates every time.
    const int kAddressImuPitch = 0x58;
    // const int kAddressImuYaw = 0x5c;
    // const int kAddressAbsYaw = 0x60;
    const int kSizeImuPitch = 4;
    const int kSizeImuYaw = 4;
    const int kSizeAbsYaw = 2;
    servo_->MemRead(
        servo_->RAM_READ, parameters_.gimbal_address,
        kAddressImuPitch,
        kSizeImuPitch + kSizeImuYaw + kSizeAbsYaw,
        [this](base::ErrorCode ec, Mech::ServoBase::MemReadResponse response) {
          HandleCurrent(ec, response);
        });
  }

  void HandleCommand(base::ErrorCode ec,
                     Mech::ServoBase::MemReadResponse response) {
    FailIf(ec);

    Parser parser = response;

    TurretData::Position command;
    command.y_deg = parser.get_int32(0) / 1000.0;
    command.x_deg = parser.get_int32(4) / 1000.0;
    data_.imu_command = command;
    Emit();
  }

  void HandleCurrent(base::ErrorCode ec,
                     Mech::ServoBase::MemReadResponse response) {
    FailIf(ec);

    Parser parser = response;

    data_.imu.y_deg = parser.get_int32(0) / 1000.0;
    data_.imu.x_deg = parser.get_int32(4) / 1000.0;
    data_.absolute.y_deg = data_.imu.y_deg;

    const uint16_t absolute_int = parser.get(8) | parser.get(9) << 7;
    data_.absolute.x_deg = (absolute_int - 0x3fff) / (0x7fff * 360.0);

    // Now read from the fire control board.
    const int kAddressFirePwm = 81;
    servo_->MemRead(
        servo_->RAM_READ, parameters_.fire_control_address,
        kAddressFirePwm, 2,
        [this](base::ErrorCode ec, Mech::ServoBase::MemReadResponse response) {
          HandleFireControl(ec, response);
        });
  }

  void HandleFireControl(base::ErrorCode ec,
                         Mech::ServoBase::MemReadResponse response) {
    data_.fire_enabled = response.register_data.at(0) != 0;
    data_.agitator_enabled = response.register_data.at(1) != 0;

    Emit();
  }

  void Emit() {
    data_.timestamp = boost::posix_time::microsec_clock::universal_time();
    parent_->turret_data_signal_(&data_);
  }

  Turret* const parent_;
  boost::asio::io_service& service_;
  Mech::ServoBase* const servo_;
  boost::asio::deadline_timer timer_;
  Parameters parameters_;

  TurretData data_;
};

Turret::Turret(boost::asio::io_service& service,
               Mech::ServoBase* servo)
    : impl_(new Impl(this, service, servo)) {}

Turret::~Turret() {}

void Turret::AsyncStart(base::ErrorHandler handler) {
  impl_->StartTimer();

  impl_->service_.post(std::bind(handler, base::ErrorCode()));
}

void Turret::SetCommand(const TurretCommand& command) {
  CommandLog log;
  log.timestamp = boost::posix_time::microsec_clock::universal_time();
  log.command = command;

  turret_command_signal_(&log);

  // TODO jpieper
}

Turret::Parameters* Turret::parameters() { return &impl_->parameters_; }

}
}
