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

#include "turret.h"

#include <fmt/format.h>

#include "mjlib/base/fail.h"
#include "mjlib/io/deadline_timer.h"

#include "base/common.h"
#include "base/logging.h"
#include "base/now.h"

namespace pl = std::placeholders;
namespace mp = mjlib::multiplex;
using Value = mp::Format::Value;

namespace mjmech {
namespace mech {

namespace {

const auto u8 = [](int val) {
  return static_cast<uint8_t>(std::max(0, std::min(255, val)));
};

const HerkuleXConstants::Register PitchCommand  = { 0x50, 4, 7, true };
const HerkuleXConstants::Register YawCommand = { 0x54, 4, 7, true };
const HerkuleXConstants::Register AbsoluteYawCommand = { 0x68, 4, 7, true };
const HerkuleXConstants::Register PitchRateCommand = {0x6c, 4, 7, true};
const HerkuleXConstants::Register YawRateCommand = {0x70, 4, 7, true};
const HerkuleXConstants::Register ImuPitch = { 0x58, 4, 7, true};
const HerkuleXConstants::Register ImuYaw = { 0x5c, 4, 7, true};
const HerkuleXConstants::Register AbsoluteYaw = { 0x60, 4, 7, true};
const HerkuleXConstants::Register LedControl = { 0x7f, 1 };
const HerkuleXConstants::Register FireTime = { 0x7c, 1};
const HerkuleXConstants::Register FirePwm = { 0x7d, 1};
const HerkuleXConstants::Register AgitatorPwm  = {0x7e, 1};
const HerkuleXConstants::Register BiasCommand = { 0x7b, 1};

enum MoteusRegister : uint32_t {
  kMode = 0x000,

  kImuPitch = 0x010,
  kImuYaw = 0x011,
  kAbsolutePitch = 0x12,
  kAbsoluteYaw = 0x13,
  kPitchRate = 0x14,
  kYawRate = 0x15,

  kPitchCommand = 0x020,
  kYawCommand = 0x021,
  kAbsolutePitchCommand = 0x022,
  kAbsoluteYawCommand = 0x023,

  kPitchRateCommand = 0x030,
  kYawRateCommand = 0x031,

  kFireTime = 0x040,
  kFirePwm = 0x041,
  kAgitatorPwm = 0x042,
  kLaser = 0x043,
  kBiasCommand = 0x044,
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
    timer_.async_wait(std::bind(&Impl::HandleTimer, this, pl::_1));
  }

  void HandleTimer(const mjlib::base::error_code& ec) {
    if (ec == boost::asio::error::operation_aborted) { return; }
    mjlib::base::FailIf(ec);
    StartTimer();

    if (is_disabled()) { return; }

    DoPoll();
  }

  void DoPoll() {
    poll_count_++;

    if (parameters_.use_moteus_turret) {
      read_request_.requests.resize(1);
      auto& request_pair = read_request_.requests.front();
      request_pair.id = parameters_.gimbal_address;
      auto& request = request_pair.request;
      request = {};
      request.ReadMultiple(MoteusRegister::kImuPitch, 4, 3);
      request.ReadMultiple(MoteusRegister::kFirePwm, 2, 0);

      read_reply_.replies.clear();

      mp_client_->AsyncRegister(
          &read_request_,
          &read_reply_,
          std::bind(&Impl::HandleMoteusCurrent, this, pl::_1));
    } else {
      // Then, ask for IMU and absolute coordinates every time.
      servo_->MemRead(
          servo_->RAM_READ, parameters_.gimbal_address,
          ImuPitch.position,
          ImuPitch.length + ImuYaw.length + AbsoluteYaw.length,
          [this](mjlib::base::error_code ec, Mech::ServoBase::MemReadResponse response) {
            HandleCurrent(ec, response);
          });
    }
  }

  void HandleCommand(mjlib::base::error_code ec,
                     Mech::ServoBase::MemReadResponse response) {
    if (CheckTemporaryError(ec)) { return; }
    mjlib::base::FailIf(ec);

    Parser parser = response;

    TurretCommand::Imu command;
    command.y_deg = parser.get_int32(0) / 1000.0;
    command.x_deg = parser.get_int32(4) / 1000.0;
    Emit();
  }

  bool CheckTemporaryError(const mjlib::base::error_code& ec,
                           bool value = false) {
    if (value ||
        ec == boost::asio::error::operation_aborted ||
        ec == herkulex_error::synchronization_error) {
      TemporarilyDisableTurret(ec);
      return true;
    } else {
      MarkCommunicationsActive();
    }
    return false;
  }

  void HandleMoteusCurrent(const mjlib::base::error_code& ec) {
    if (CheckTemporaryError(ec, read_reply_.replies.empty())) { return; }
    mjlib::base::FailIf(ec);

    auto& reply = read_reply_.replies.front().reply;

    auto read_float = [](auto read_result) -> double {
      auto* maybe_value = std::get_if<Value>(&read_result);
      if (!maybe_value) { return 0.0; }
      auto* maybe_float = std::get_if<float>(&*maybe_value);
      if (!maybe_float) { return 0.0; }
      return *maybe_float;
    };

    data_.imu.y_deg = read_float(reply[MoteusRegister::kImuPitch]);
    data_.imu.x_deg = read_float(reply[MoteusRegister::kImuYaw]);
    data_.absolute.y_deg = data_.imu.y_deg;
    data_.absolute.x_deg = read_float(reply[MoteusRegister::kAbsoluteYaw]);

    auto read_int8 = [](auto read_result) -> int8_t {
      auto* maybe_value = std::get_if<Value>(&read_result);
      if (!maybe_value) { return 0.0; }
      auto* maybe_float = std::get_if<int8_t>(&*maybe_value);
      if (!maybe_float) { return 0.0; }
      return *maybe_float;
    };

    data_.fire_enabled = read_int8(reply[MoteusRegister::kFirePwm]) != 0;
    data_.agitator_enabled = read_int8(reply[MoteusRegister::kAgitatorPwm]) != 0;
    if (data_.fire_enabled) {
      data_.total_fire_time_s += parameters_.period_s;
    }

    Emit();
  }

  void HandleCurrent(mjlib::base::error_code ec,
                     Mech::ServoBase::MemReadResponse response) {
    if (CheckTemporaryError(ec)) { return; }
    mjlib::base::FailIf(ec);

    Parser parser = response;

    data_.imu.y_deg = parser.get_int32(0) / 1000.0;
    data_.imu.x_deg = parser.get_int32(4) / 1000.0;
    data_.absolute.y_deg = data_.imu.y_deg;

    data_.absolute.x_deg = parser.get_int32(8) / 1000.0;

    // Now read from the fire control board.
    servo_->MemRead(
        servo_->RAM_READ, parameters_.gimbal_address,
        FirePwm.position, 2,
        [this](mjlib::base::error_code ec, Mech::ServoBase::MemReadResponse response) {
          HandleFireControl(ec, response);
        });
  }

  void HandleFireControl(mjlib::base::error_code ec,
                         Mech::ServoBase::MemReadResponse response) {
    if (CheckTemporaryError(ec)) { return; }
    mjlib::base::FailIf(ec);

    data_.fire_enabled = response.register_data.at(0) != 0;
    data_.agitator_enabled = response.register_data.at(1) != 0;
    if (data_.fire_enabled) {
      data_.total_fire_time_s += parameters_.period_s;
    }

    Emit();
  }

  void Emit() {
    data_.timestamp = base::Now(service_);
    parent_->turret_data_signal_(&data_);
  }

  void HandleWrite(mjlib::base::error_code ec) {
    mjlib::base::FailIf(ec);
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
    if (parameters_.use_moteus_turret) {
      mp::RegisterRequest request;
      request.WriteMultiple(
          MoteusRegister::kPitchCommand,
          {Value(static_cast<float>(command.y_deg)),
           Value(static_cast<float>(command.x_deg))});
      SendRegister(request);
    } else {
      const std::string data = MakeCommand(command);
      servo_->MemWrite(
          servo_->RAM_WRITE, parameters_.gimbal_address,
          PitchCommand.position,
          data,
          std::bind(&Impl::HandleWrite, this, pl::_1));
    }
  }

  void TemporarilyDisableTurret(const mjlib::base::error_code& ec) {
    log_.warn("error reading turret: " + ec.message());
    error_count_++;

    if (error_count_ >= parameters_.error_disable_count) {
      if (data_.disable_period_s == 0.0) {
        data_.disable_period_s = parameters_.initial_disable_period_s;
      } else {
        data_.disable_period_s = std::min(parameters_.max_disable_period_s,
                                          data_.disable_period_s * 2);
      }
      log_.warn(
          fmt::format(
              "device unresponsive, disabling for {} seconds",
              data_.disable_period_s));
      disable_until_ = base::Now(service_) +
          base::ConvertSecondsToDuration(data_.disable_period_s);
    }
  }

  void MarkCommunicationsActive() {
    if (error_count_ >= parameters_.error_disable_count) {
      log_.warn("connection re-established");
    }
    error_count_ = 0;
    data_.disable_period_s = 0.0;
  }

  bool is_disabled() const {
    if (disable_until_.is_not_a_date_time()) { return false; }
    const auto now = base::Now(service_);
    return now < disable_until_;
  }

  void SetFireControl(const TurretCommand::FireControl& command) {
    const auto now = base::Now(service_);

    // Update the laser status.
    if (parameters_.use_moteus_turret) {
      mp::RegisterRequest request;
      request.WriteSingle(
          MoteusRegister::kLaser, Value(u8(command.laser_on ? 1 : 0)));
      SendRegister(request);
    } else {
      uint8_t leds = (command.laser_on ? 1 : 0) << 2;
      servo_->RamWrite(
          parameters_.gimbal_address, LedControl, leds,
          std::bind(&Impl::HandleWrite, this, pl::_1));
    }

    // Now do the fire control, only accept things where the sequence
    // number has advanced.
    if (command.fire.sequence != data_.last_sequence) {
      using FM = TurretCommand::Fire::Mode;

      data_.last_sequence = command.fire.sequence;

      const double fire_time_s = [&]() {
        switch (command.fire.command) {
          case FM::kOff: { return 0.0; }
          case FM::kNow1: { return parameters_.fire_duration_s; }
          case FM::kCont: { return 0.5; }
          case FM::kInPos1:
          case FM::kInPos2:
          case FM::kInPos3:
          case FM::kInPos5: {
            // TODO jpieper: Once we do support this (if we do at all),
            // we should probably capture the command around so that we
            // can keep trying to execute it as the gimbal moves into
            // position.
            mjlib::base::Fail("Unsupported fire control command");
            break;
          }
        }
        mjlib::base::AssertNotReached();
      }();

      if ((command.fire.command == FM::kNow1 ||
           command.fire.command == FM::kCont) &&
          data_.last_fire_command != command.fire.command) {
        data_.auto_agitator_count++;
        if (data_.auto_agitator_count == 2) {
          data_.auto_agitator_count = 0;
          data_.auto_agitator_end =
              now + base::ConvertSecondsToDuration(
                  parameters_.auto_agitator_time_s);
        }
      }

      data_.last_fire_command = command.fire.command;

      const double pwm =
          (fire_time_s == 0.0) ? 0.0 : parameters_.fire_motor_pwm;

      if (parameters_.use_moteus_turret) {
        mp::RegisterRequest request;
        request.WriteMultiple(
            MoteusRegister::kFireTime,
            {Value(u8(fire_time_s / 0.01)),
                  Value(u8(pwm * 255))});
        SendRegister(request);
      } else {
        const uint8_t fire_data[] = {
          u8(fire_time_s / 0.01),
          u8(pwm * 255),
        };
        std::string fire_data_str(reinterpret_cast<const char*>(fire_data),
                                  sizeof(fire_data));
        servo_->MemWrite(
            servo_->RAM_WRITE,
            parameters_.gimbal_address,
            FireTime.position,
            fire_data_str,
            std::bind(&Impl::HandleWrite, this, pl::_1));
      }
    }

    // Update the agitator status.
    using AM = TurretCommand::AgitatorMode;
    const uint8_t agitator_pwm = u8([&]() {
        switch (command.agitator) {
          case AM::kOff: {
            return 0.0;
          }
          case AM::kOn: {
            return parameters_.agitator_pwm;
          }
          case AM::kAuto: {
            if (data_.auto_agitator_end.is_not_a_date_time() ||
                data_.auto_agitator_end < now) {
              return 0.0;
            } else {
              return parameters_.agitator_pwm;
            }
          }
        }
        mjlib::base::AssertNotReached();
      }() * 255);

    if (parameters_.use_moteus_turret) {
      mp::RegisterRequest request;
      request.WriteSingle(MoteusRegister::kAgitatorPwm, Value(u8(agitator_pwm)));
      SendRegister(request);
    } else {
      servo_->RamWrite(
          parameters_.gimbal_address, AgitatorPwm, agitator_pwm,
          std::bind(&Impl::HandleWrite, this, pl::_1));
    }
  }

  void WriteRate(double x_deg_s, double y_deg_s) {
    const float pitch_dps = y_deg_s;
    const float yaw_dps = x_deg_s;

    if (parameters_.use_moteus_turret) {
      mp::RegisterRequest request;
      request.WriteMultiple(MoteusRegister::kPitchRateCommand,
                            {Value(pitch_dps), Value(yaw_dps)});
      SendRegister(request);
    } else {
      const int pitch_command = static_cast<int>(pitch_dps * 1000.0);
      servo_->RamWrite(
          parameters_.gimbal_address, PitchRateCommand, pitch_command,
          std::bind(&Impl::HandleWrite, this, pl::_1));

      const int yaw_command = static_cast<int>(yaw_dps * 1000.0);
      servo_->RamWrite(
          parameters_.gimbal_address, YawRateCommand, yaw_command,
          std::bind(&Impl::HandleWrite, this, pl::_1));
    }
  }

  void UpdateTrackedTarget(const std::optional<base::Point3D>& target) {
    log_.debug(fmt::format("UpdateTrackedTarget vision={}, target={}",
                           !!target, !!data_.target_relative));

    // If we're not in target relative mode, then nothing to do here.
    if (!data_.target_relative) { return; }

    const auto now = base::Now(service_);

    if (!target) {
      // We don't currently have a target.  Just scale our rates by
      // our time constant.
      const double delta_s = base::ConvertDurationToDouble(now - data_.target_relative_last_time);
      const double factor = std::pow(0.5, (delta_s / parameters_.target_time_constant_s));
      data_.target_relative_rate.x_deg_s *= factor;
      data_.target_relative_rate.y_deg_s *= factor;
    } else {
      // Update the rates with the new target position.
      const double delta_x = target->x() - data_.target_relative->x;
      const double delta_y = target->y() - data_.target_relative->y;
      data_.target_relative_rate.x_deg_s =
          (delta_x / parameters_.pixels_per_degree) /
          parameters_.target_time_constant_s;
      data_.target_relative_rate.y_deg_s =
          -(delta_y / parameters_.pixels_per_degree) /
          parameters_.target_time_constant_s;
    }

    WriteRate(data_.target_relative_rate.x_deg_s,
              data_.target_relative_rate.y_deg_s);

    data_.target_relative_last_time = now;
  }

  void SendRegister(const mp::RegisterRequest& request_in) {
    outstanding_requests_.push_back({});
    auto& request = outstanding_requests_.back();
    request.requests.resize(1);
    request.requests[0].id = parameters_.gimbal_address;
    request.requests[0].request = request_in;

    auto it = --outstanding_requests_.end();

    mp_client_->AsyncRegister(
        &*it,
        nullptr,
        [this, it](const mjlib::base::error_code& ec) {
          if (CheckTemporaryError(ec)) { return; }
          this->outstanding_requests_.erase(it);
        });
  }

  base::LogRef log_ = base::GetLogInstance("turret");
  Turret* const parent_;
  boost::asio::io_service& service_;
  Mech::ServoBase* const servo_;
  mp::ThreadedClient* mp_client_ = nullptr;
  mjlib::io::DeadlineTimer timer_;
  Parameters parameters_;
  boost::posix_time::ptime disable_until_;
  int poll_count_ = 0;
  int error_count_ = 0;

  Data data_;

  using TC = mp::ThreadedClient;
  TC::Request read_request_;
  TC::Reply read_reply_;
  std::list<TC::Request> outstanding_requests_;
};

Turret::Turret(boost::asio::io_service& service,
               Mech::ServoBase* servo)
    : impl_(new Impl(this, service, servo)) {}

Turret::~Turret() {}

void Turret::AsyncStart(mjlib::io::ErrorCallback handler) {
  impl_->StartTimer();

  impl_->service_.post(std::bind(handler, mjlib::base::error_code()));
}

void Turret::SetMultiplexClient(mp::ThreadedClient* client) {
  impl_->mp_client_ = client;
}

void Turret::SetCommand(const TurretCommand& command) {
  CommandLog log;
  log.timestamp = base::Now(impl_->service_);
  log.command = command;

  turret_command_signal_(&log);

  if (impl_->is_disabled()) { return; }

  const bool this_rate = command.rate || command.target_relative;

  if (impl_->data_.last_rate && !this_rate) {
    // Since we no longer want to be commanding a rate, ensure that
    // the gimbal is not advancing on its own.
    impl_->data_.last_rate = false;
    if (impl_->parameters_.use_moteus_turret) {
      mp::RegisterRequest request;
      request.WriteMultiple(MoteusRegister::kPitchRateCommand,
                            {Value(0.0f), Value(0.0f)});
      impl_->SendRegister(request);
    } else {
      impl_->servo_->RamWrite(
          impl_->parameters_.gimbal_address, PitchRateCommand, 0,
          std::bind(&Impl::HandleWrite, impl_.get(), pl::_1));
      impl_->servo_->RamWrite(
          impl_->parameters_.gimbal_address, YawRateCommand, 0,
          std::bind(&Impl::HandleWrite, impl_.get(), pl::_1));
    }
  }

  if (!command.target_relative) {
    impl_->data_.target_relative = std::nullopt;
  }

  if (command.target_relative) {
    impl_->data_.target_relative = command.target_relative;
    impl_->data_.target_relative_rate = {};
    impl_->data_.target_relative_last_time = base::Now(impl_->service_);
  } else if (command.absolute) {
    const double limited_pitch_deg =
        std::max(impl_->parameters_.min_y_deg,
                 std::min(impl_->parameters_.max_y_deg,
                          command.absolute->y_deg));
    const double limited_yaw_deg =
        std::max(impl_->parameters_.min_x_deg,
                 std::min(impl_->parameters_.max_x_deg,
                          command.absolute->x_deg));

    if (impl_->parameters_.use_moteus_turret) {
      mp::RegisterRequest request;
      request.WriteSingle(MoteusRegister::kPitchCommand,
                          static_cast<float>(limited_pitch_deg));
      request.WriteSingle(MoteusRegister::kAbsoluteYawCommand,
                          static_cast<float>(limited_yaw_deg));
      impl_->SendRegister(request);
    } else {
      const int pitch_command =
          static_cast<int>(limited_pitch_deg * 1000.0);
      impl_->servo_->RamWrite(
          impl_->parameters_.gimbal_address, PitchCommand, pitch_command,
          std::bind(&Impl::HandleWrite, impl_.get(), pl::_1));

      const int yaw_command =
          static_cast<int>(limited_yaw_deg * 1000.0);
      impl_->servo_->RamWrite(
          impl_->parameters_.gimbal_address, AbsoluteYawCommand, yaw_command,
          std::bind(&Impl::HandleWrite, impl_.get(), pl::_1));
    }
  } else if (command.imu) {
    // Then IMU relative.

    const double pitch_deg = command.imu->y_deg;
    const double yaw_deg = command.imu->x_deg;

    if (impl_->parameters_.use_moteus_turret) {
      mp::RegisterRequest request;
      request.WriteMultiple(MoteusRegister::kPitchCommand,
                            {Value(static_cast<float>(pitch_deg)),
                                  Value(static_cast<float>(yaw_deg))});
      impl_->SendRegister(request);
    } else {
      const int pitch_command = static_cast<int>(pitch_deg * 1000.0);
      impl_->servo_->RamWrite(
          impl_->parameters_.gimbal_address, PitchCommand, pitch_command,
          std::bind(&Impl::HandleWrite, impl_.get(), pl::_1));

      const int yaw_command = static_cast<int>(yaw_deg * 1000.0);
      impl_->servo_->RamWrite(
          impl_->parameters_.gimbal_address, YawCommand, yaw_command,
          std::bind(&Impl::HandleWrite, impl_.get(), pl::_1));
    }
  } else if (command.rate) {
    // Finally, rate if we have one.

    impl_->WriteRate(command.rate->x_deg_s, command.rate->y_deg_s);
    impl_->data_.last_rate = true;
  }

  impl_->SetFireControl(command.fire_control);
}

void Turret::SetFireControl(const TurretCommand::FireControl& command) {
  impl_->SetFireControl(command);
}

void Turret::UpdateTrackedTarget(
    const std::optional<base::Point3D>& target) {
  impl_->UpdateTrackedTarget(target);
}

void Turret::StartBias() {
  if (impl_->parameters_.use_moteus_turret) {
    mp::RegisterRequest request;
    request.WriteSingle(MoteusRegister::kBiasCommand, u8(1));
    impl_->SendRegister(request);
  } else {
    impl_->servo_->RamWrite(
        impl_->parameters_.gimbal_address, BiasCommand, 1,
        std::bind(&Impl::HandleWrite, impl_.get(), pl::_1));
  }
}

const Turret::Data& Turret::data() const { return impl_->data_; }

Turret::Parameters* Turret::parameters() { return &impl_->parameters_; }

}
}
