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

struct PitchCommand {
  const int position = 0x50;
  const int length = 4;
};

struct YawCommand {
  const int position = 0x54;
  const int length = 4;
};

struct AbsoluteYawCommand {
  const int position = 0x68;
  const int length = 2;
};

struct ImuPitch {
  const int position = 0x58;
  const int length = 4;
};

struct ImuYaw {
  const int position = 0x5c;
  const int length = 4;
};

struct AbsoluteYaw {
  const int position = 0x60;
  const int length = 2;
};

struct LedControl {
  const int position = 0x35;
  const int length = 1;
};

struct FireTime {
  const int position = 80;
  const int length = 1;
};

struct FirePwm {
  const int position = 81;
  const int length = 1;
};

struct AgitatorPwm {
  const int position = 82;
  const int length = 1;
};


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
    base::FailIf(ec);
    StartTimer();

    if (is_disabled()) { return; }

    DoPoll();
  }

  void DoPoll() {
    poll_count_++;

    // If we are doing a rate motion, and know the current command,
    // send our updated command.

    if (data_.imu_command &&
        (data_.rate.x_deg_s != 0.0 || data_.rate.y_deg_s != 0.0)) {
      TurretCommand::Imu next = *data_.imu_command;
      next.x_deg += data_.rate.x_deg_s * parameters_.period_s;
      next.y_deg += data_.rate.y_deg_s * parameters_.period_s;
      next.y_deg = std::max(parameters_.min_y_deg,
                            std::min(parameters_.max_y_deg,
                                     next.y_deg));

      data_.imu_command = next;

      SendImuCommand(next);
    }

    // If we don't currently know it, ask for the current command.
    if (!data_.imu_command ||
        (poll_count_ % parameters_.command_update_decimate) == 0) {
      const int kAddressPitchCommand = 0x50;
      servo_->MemRead(
          servo_->RAM_READ, parameters_.gimbal_address,
          kAddressPitchCommand, 8,
          [this](base::ErrorCode ec,
                 Mech::ServoBase::MemReadResponse response) {
            HandleCommand(ec, response);
          });
    }

    // Then, ask for IMU and absolute coordinates every time.
    servo_->MemRead(
        servo_->RAM_READ, parameters_.gimbal_address,
        ImuPitch().position,
        ImuPitch().length + ImuYaw().length + AbsoluteYaw().length,
        [this](base::ErrorCode ec, Mech::ServoBase::MemReadResponse response) {
          HandleCurrent(ec, response);
        });
  }

  void HandleCommand(base::ErrorCode ec,
                     Mech::ServoBase::MemReadResponse response) {
    if (ec == boost::asio::error::operation_aborted ||
        ec == herkulex_error::synchronization_error) {
      TemporarilyDisableTurret(ec);
      return;
    } else {
      MarkCommunicationsActive();
    }
    FailIf(ec);

    Parser parser = response;

    TurretCommand::Imu command;
    command.y_deg = parser.get_int32(0) / 1000.0;
    command.x_deg = parser.get_int32(4) / 1000.0;
    data_.imu_command = command;
    Emit();
  }

  void HandleCurrent(base::ErrorCode ec,
                     Mech::ServoBase::MemReadResponse response) {
    if (ec == boost::asio::error::operation_aborted ||
        ec == herkulex_error::synchronization_error) {
      TemporarilyDisableTurret(ec);
      return;
    } else {
      MarkCommunicationsActive();
    }
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
    FailIf(ec);

    data_.fire_enabled = response.register_data.at(0) != 0;
    data_.agitator_enabled = response.register_data.at(1) != 0;

    Emit();
  }

  void Emit() {
    data_.timestamp = boost::posix_time::microsec_clock::universal_time();
    parent_->turret_data_signal_(&data_);
  }

  void HandleWrite(base::ErrorCode ec) {
    FailIf(ec);
  }

  static std::string MakeCommand(const TurretCommand::Imu& command) {
    const int pitch_command =
        static_cast<int>(command.y_deg * 1000.0);
    const int yaw_command =
        static_cast<int>(command.x_deg * 1000.0);
    const auto u8 = [](int val) { return static_cast<uint8_t>(val); };

    const uint8_t data[8] = {
      u8(pitch_command & 0x7f),
      u8((pitch_command >> 7) & 0x7f),
      u8((pitch_command >> 14) & 0x7f),
      u8((pitch_command >> 21) & 0x7f),
      u8(yaw_command & 0x7f),
      u8((yaw_command >> 7) & 0x7f),
      u8((yaw_command >> 14) & 0x7f),
      u8((yaw_command >> 21) & 0x7f),
    };
    return std::string(reinterpret_cast<const char*>(data), sizeof(data));
  }

  void SendImuCommand(const TurretCommand::Imu& command) {
    const std::string data = MakeCommand(command);
    servo_->MemWrite(
        servo_->RAM_WRITE, parameters_.gimbal_address,
        PitchCommand().position,
        data,
        std::bind(&Impl::HandleWrite, this,
                  std::placeholders::_1));
  }

  void TemporarilyDisableTurret(const base::ErrorCode& ec) {
    std::cerr << "error reading turret: " << ec.message() << "\n";
    error_count_++;

    if (error_count_ >= parameters_.error_disable_count) {
      std::cerr << boost::format(
          "Turret: Device unresponsive, disabling for %d seconds.\n") %
          parameters_.disable_period_s;
      disable_until_ = boost::posix_time::microsec_clock::universal_time() +
          base::ConvertSecondsToDuration(parameters_.disable_period_s);
    }
  }

  void MarkCommunicationsActive() {
    error_count_ = 0;
  }

  bool is_disabled() const {
    if (disable_until_.is_not_a_date_time()) { return false; }
    const auto now = boost::posix_time::microsec_clock::universal_time();
    return now < disable_until_;
  }

  Turret* const parent_;
  boost::asio::io_service& service_;
  Mech::ServoBase* const servo_;
  boost::asio::deadline_timer timer_;
  Parameters parameters_;
  boost::posix_time::ptime disable_until_;
  int poll_count_ = 0;
  int error_count_ = 0;

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

  if (impl_->is_disabled()) { return; }

  if (command.absolute) {
    // Absolute takes precedence.

    // When we send a command, the device will translate that into
    // some new IMU relative command, thus we no longer know what the
    // relative command is and will have to re-request it.
    impl_->data_.imu_command = boost::none;

    // Also, we will by default stop moving after such a command.
    impl_->data_.rate = TurretCommand::Rate();

    const double limited_pitch_deg =
        std::max(impl_->parameters_.min_y_deg,
                 std::min(impl_->parameters_.max_y_deg,
                          command.absolute->y_deg));
    const int pitch_command =
        static_cast<int>(limited_pitch_deg * 1000.0);
    impl_->servo_->RamWrite(
        impl_->parameters_.gimbal_address, PitchCommand(), pitch_command,
        std::bind(&Impl::HandleWrite, impl_.get(), std::placeholders::_1));


    const double limited_yaw_deg =
        std::max(impl_->parameters_.min_x_deg,
                 std::min(impl_->parameters_.max_x_deg,
                          command.absolute->x_deg));
    const int yaw_command =
        std::max(0, std::min(0x3fff,
                             static_cast<int>(limited_yaw_deg /
                                              0x3fff * 360.0 + 0x1fff)));
    impl_->servo_->RamWrite(
        impl_->parameters_.gimbal_address, AbsoluteYawCommand(), yaw_command,
        std::bind(&Impl::HandleWrite, impl_.get(), std::placeholders::_1));
  } else if (command.imu) {
    // Then IMU relative.

    impl_->data_.imu_command = *command.imu;
    impl_->data_.imu_command->y_deg =
        std::max(impl_->parameters_.min_y_deg,
                 std::min(impl_->parameters_.max_y_deg,
                          impl_->data_.imu_command->y_deg));
    impl_->data_.rate = TurretCommand::Rate();

    impl_->SendImuCommand(*command.imu);
  } else if (command.rate) {
    // Finally, rate if we have one.

    // All we do here is update our rate for the polling loop to take
    // care of.
    impl_->data_.rate = *command.rate;
  }

  // Update the laser status.
  uint8_t leds = (command.laser_on ? 1 : 0) << 2;
  impl_->servo_->RamWrite(
      impl_->parameters_.fire_control_address, LedControl(), leds,
      std::bind(&Impl::HandleWrite, impl_.get(), std::placeholders::_1));

  const auto u8 = [](int val) {
    return static_cast<uint8_t>(std::max(0, std::min(255, val)));
  };

  // Update the agitator status.
  using AM = TurretCommand::AgitatorMode;
  const uint8_t agitator_pwm = u8([&]() {
      switch (command.agitator) {
        case AM::kOff: {
          return 0.0;
        }
        case AM::kOn: {
          return impl_->parameters_.agitator_pwm;
        }
        case AM::kAuto: {
          base::Fail("Unsupported agitator mode");
      }
      }
      base::AssertNotReached();
    }() * 255);

  impl_->servo_->RamWrite(
      impl_->parameters_.fire_control_address, AgitatorPwm(), agitator_pwm,
      std::bind(&Impl::HandleWrite, impl_.get(), std::placeholders::_1));


  // Now do the fire control, only accept things where the sequence
  // number has advanced.
  if (command.fire.sequence != impl_->data_.last_sequence) {
    using FM = TurretCommand::Fire::Mode;

    impl_->data_.last_sequence = command.fire.sequence;

    const double fire_time_s = [&]() {
      switch (command.fire.command) {
        case FM::kOff: { return 0.0; }
        case FM::kNow1: { return impl_->parameters_.fire_duration_s; }
        case FM::kCont: { return 0.5; }
        case FM::kInPos1:
        case FM::kInPos2:
        case FM::kInPos3:
        case FM::kInPos5: {
          // TODO jpieper: Once we do support this (if we do at all),
          // we should probably capture the command around so that we
          // can keep trying to execute it as the gimbal moves into
          // position.
          base::Fail("Unsupported fire control command");
          break;
        }
      }
      base::AssertNotReached();
    }();

    const double pwm =
        (fire_time_s == 0.0) ? 0.0 : impl_->parameters_.fire_motor_pwm;
    const uint8_t fire_data[] = {
      u8(fire_time_s / 0.01),
      u8(pwm * 255),
    };
    std::string fire_data_str(reinterpret_cast<const char*>(fire_data),
                              sizeof(fire_data));
    impl_->servo_->MemWrite(
        impl_->servo_->RAM_WRITE,
        impl_->parameters_.fire_control_address,
        FireTime().position,
        fire_data_str,
        std::bind(&Impl::HandleWrite, impl_.get(),
                  std::placeholders::_1));
  }
}

Turret::Parameters* Turret::parameters() { return &impl_->parameters_; }

}
}
