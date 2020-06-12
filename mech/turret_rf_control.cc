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

/// @file
///
/// # Received Commands #
///
/// The following slot protocol is used to receive commands.
///
/// ## Slot 0 - Primary Command ##
///   byte 0: Desired mode, from TurretControl::Mode, remaining
///           bytes are mode-specific
///
///    Stop - None
///    Active - None
///   byte 1: messages received in last second
///
/// ## Slot 1 - rate ##
///
///  * int16 pitch_deg_s (scaled to -400/400)
///  * int16 yaw_deg_s (scaled to -400/400)
///
/// ## Slot 2 - weapon ##
///
///  # int8 laser
///  # int16 trigger_sequence
///
/// # Transmitted telemetry #
///
/// The following protocol is used to transmit telemetry.
///
/// ## Slot 0 - Primary State ##
///
///  * int8_t mode (from TurretControl::Mode)
///  * int8_t received packets in last second
///
/// ## Slot 1 - IMU ##
///
///  * int16_t pitch_deg (-180/180)
///  * int16_t yaw_deg (-180/180)
///  * int16_t pitch_rate_dps (-400/400)
///  * int16_t yaw_rate_dps (-400/400)
///
/// ## Slot 2 - Servo ##
///
///  * int16_t pitch_deg (-180/180)
///  * int16_t yaw_deg (-180/180)
///
/// ## Slot 3 - Weapon ##
///
///  * int8_t armed
///  * int8_t laser status
///  * int16_t shot count
///
/// ## Slot 8 - Servo Summary ##
///
///  * int8_t min_voltage
///  * int8_t max_voltage
///  * int8_t min_temp_C
///  * int8_t max_temp_C
///  * int8_t fault
///
/// ## Slot 14 - Fault ##
///
///  * char[15] - truncated text string


#include "mech/turret_rf_control.h"

#include <boost/asio/post.hpp>

#include "mjlib/base/buffer_stream.h"
#include "mjlib/io/now.h"

#include "base/telemetry_registry.h"
#include "base/saturate.h"

namespace pl = std::placeholders;

namespace mjmech {
namespace mech {

namespace {
auto vmin = [](const auto& container, auto getter, auto incase) {
  if (container.empty()) { return incase; }
  return getter(*std::min_element(
      container.begin(), container.end(),
      [&](const auto& lhs, const auto& rhs) {
        return getter(lhs) < getter(rhs);
      }));
};

auto vmax = [](const auto& container, auto getter, auto incase) {
  if (container.empty()) { return incase; }
  return getter(*std::max_element(
      container.begin(), container.end(),
      [&](const auto& lhs, const auto& rhs) {
        return getter(lhs) < getter(rhs);
      }));
};

constexpr int kOnlyRemote = 0;
constexpr double kVoltageScale = 4.0;
}

struct SlotData {
  boost::posix_time::ptime timestamp;

  struct Slot {
    boost::posix_time::ptime timestamp;
    uint8_t size = 0;
    std::array<uint8_t, 15> data = {};

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(timestamp));
      a->Visit(MJ_NVP(size));
      a->Visit(MJ_NVP(data));
    }
  };

  std::array<Slot, 15> rx = {};

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(timestamp));
    a->Visit(MJ_NVP(rx));
  }
};

class TurretRfControl::Impl {
 public:
  Impl(const base::Context& context,
       TurretControl* turret_control,
       RfGetter rf_getter)
      : executor_(context.executor),
        turret_control_(turret_control),
        rf_getter_(rf_getter) {
    context.telemetry_registry->Register("slotrf", &slotrf_signal_);
  }

  void AsyncStart(mjlib::io::ErrorCallback callback) {
    rf_ = rf_getter_();
    BOOST_ASSERT(!rf_);

    StartRead();

    boost::asio::post(
        executor_,
        std::bind(std::move(callback), mjlib::base::error_code()));
  }

 private:
  void StartRead() {
    rf_->AsyncWaitForSlot(
        &remote_, &bitfield_, std::bind(&Impl::HandleSlot, this, pl::_1));
  }

  void HandleSlot(const mjlib::base::error_code& ec) {
    mjlib::base::FailIf(ec);

    const auto now = mjlib::io::Now(executor_.context());
    slot_data_.timestamp = now;
    receive_times_.push_back(now);
    while (mjlib::base::ConvertDurationToSeconds(
               now - receive_times_.front()) > 1.0) {
      receive_times_.pop_front();
    }

    for (int i = 0; i < 15; i++) {
      if (!(bitfield_ & (1 << i))) { continue; }

      const auto slot = rf_->rx_slot(kOnlyRemote, i);
      slot_data_.rx[i].timestamp = slot.timestamp;
      slot_data_.rx[i].size = slot.size;
      std::memcpy(&slot_data_.rx[i].data[0], &slot.data[0], slot.size);
      std::memset(&slot_data_.rx[i].data[slot.size], 0, 15 - slot.size);
    }

    slotrf_signal_(&slot_data_);

    // We only command when we receive a mode slot.
    if (bitfield_ & 0x01) {
      SendCommand();
    }

    ReportTelemetry();

    StartRead();
  }

  void SendCommand() {
    // We just got a new command.  Formulate our command structure and
    // send it on.
    TurretControl::CommandData command;

    command.mode = [&]() {
      switch (slot_data_.rx[0].data[0]) {
        case 0: {
          return TurretControl::Mode::kStop;
        }
        case 1: {
          return TurretControl::Mode::kActive;
        }
        default: {
          return TurretControl::Mode::kStop;
        }

        mjlib::base::AssertNotReached();
      }
    }();

    auto read_int16 = [&](int slot, int pos) {
      int16_t value = 0;
      std::memcpy(&value, &slot_data_.rx[slot].data[pos], 2);
      return value;
    };

    command.pitch_rate_dps = 400.0 * read_int16(1, 0) / 32767.0;
    command.yaw_rate_dps = 400.0 * read_int16(1, 2) / 32767.0;
    command.track_target = slot_data_.rx[1].data[4] != 0;

    command.laser_enable = slot_data_.rx[2].data[0] ? true : false;
    command.trigger_sequence = read_int16(2, 1);

    // RF control takes precendence over everything if we're seeing
    // it.
    command.priority = 99;

    turret_control_->Command(command);
  }

  void ReportTelemetry() {
    const auto now = mjlib::io::Now(executor_.context());
    if (!last_telemetry_.is_not_a_date_time() &&
        mjlib::base::ConvertDurationToSeconds(now - last_telemetry_) < 0.01) {
      return;
    }
    last_telemetry_ = now;

    const auto& s = turret_control_->status();
    const auto& w = turret_control_->weapon();

    const bool fault = s.mode == TurretControl::Mode::kFault;

    using Slot = RfClient::Slot;
    {
      Slot slot0;
      slot0.size = 2;
      slot0.priority = 0xffffffff;
      slot0.data[0] = static_cast<uint8_t>(s.mode);
      slot0.data[1] = base::Saturate<uint8_t>(receive_times_.size());
      rf_->tx_slot(kOnlyRemote, 0, slot0);
    }
    {
      Slot slot1;
      slot1.size = 8;
      slot1.priority = 0xffffffff;
      mjlib::base::BufferWriteStream bs({slot1.data, slot1.size});
      mjlib::telemetry::WriteStream ts{bs};
      ts.Write(base::Saturate<int16_t>(32767.0 * s.imu.pitch_deg / 180.0));
      ts.Write(base::Saturate<int16_t>(32767.0 * s.imu.yaw_deg / 180.0));
      ts.Write(base::Saturate<int16_t>(32767.0 * s.imu.pitch_rate_dps /  400.0));
      ts.Write(base::Saturate<int16_t>(32767.0 * s.imu.yaw_rate_dps / 400.0));
      rf_->tx_slot(kOnlyRemote, 1, slot1);
    }
    {
      Slot slot2;
      slot2.size = 4;
      slot2.priority = fault ? 0x01010101 : 0xffffffff;
      mjlib::base::BufferWriteStream bs({slot2.data, slot2.size});
      mjlib::telemetry::WriteStream ts{bs};
      ts.Write(base::Saturate<int16_t>(s.pitch_servo.angle_deg / 180.0 * 32767.0));
      ts.Write(base::Saturate<int16_t>(base::WrapNegPiToPi(base::Radians(s.yaw_servo.angle_deg)) / M_PI * 32767.0));
      rf_->tx_slot(kOnlyRemote, 2, slot2);
    }
    {
      Slot slot3;
      slot3.size = 4;
      slot3.priority = fault ? 0x02020202 : 0xffffffff;
      mjlib::base::BufferWriteStream bs({slot3.data, slot3.size});
      mjlib::telemetry::WriteStream ts{bs};
      ts.Write(static_cast<int8_t>(w.armed ? 1 : 0));
      ts.Write(static_cast<int8_t>(w.laser_time_10ms ? 1 : 0));
      ts.Write(static_cast<int16_t>(w.shot_count));
      rf_->tx_slot(kOnlyRemote, 3, slot3);
    }

    {
      Slot slot8;
      slot8.size = 5;
      slot8.priority = 0x55555555;
      mjlib::base::BufferWriteStream bs({slot8.data, 5});
      mjlib::telemetry::WriteStream ts{bs};
      std::array<const TurretControl::Status::GimbalServo*, 2> servos{{
          &s.pitch_servo, &s.yaw_servo}};
      ts.Write(base::Saturate<int8_t>(kVoltageScale *
                   vmin(servos, [](const auto& j) { return j->voltage; }, 0.0)));
      ts.Write(base::Saturate<int8_t>(kVoltageScale *
                   vmax(servos, [](const auto& j) { return j->voltage; }, 0.0)));
      ts.Write(
          base::Saturate<int8_t>(
              vmin(servos, [](const auto& j) { return j->temperature_C; }, 0.0)));
      ts.Write(
          base::Saturate<int8_t>(
              vmax(servos, [](const auto& j) { return j->temperature_C; }, 0.0)));
      ts.Write(
          base::Saturate<int8_t>(
              vmax(servos, [](const auto& j) { return j->fault; }, 0)));
      rf_->tx_slot(kOnlyRemote, 8, slot8);
    }
    {
      Slot slot14;
      if (!fault) {
        slot14.priority = 0x01010101;
        slot14.size = 0;
        rf_->tx_slot(kOnlyRemote, 14, slot14);
      } else {
        slot14.priority = 0xaaaaaaaa;
        auto size = std::min<size_t>(13, s.fault.size());
        slot14.size = size;
        std::memcpy(slot14.data, s.fault.data(), size);
        rf_->tx_slot(kOnlyRemote, 14, slot14);
      }
    }
  }

  boost::asio::executor executor_;
  TurretControl* const turret_control_;
  RfGetter rf_getter_;

  RfClient* rf_ = nullptr;

  int remote_ = 0;
  uint16_t bitfield_ = 0;

  SlotData slot_data_;
  boost::signals2::signal<void (const SlotData*)> slotrf_signal_;

  boost::posix_time::ptime last_telemetry_;
  std::deque<boost::posix_time::ptime> receive_times_;
};

TurretRfControl::TurretRfControl(
    const base::Context& context,
    TurretControl* turret_control,
    RfGetter rf_getter)
    : impl_(std::make_unique<Impl>(context, turret_control, rf_getter)) {}

TurretRfControl::~TurretRfControl() {}

void TurretRfControl::AsyncStart(mjlib::io::ErrorCallback callback) {
  impl_->AsyncStart(std::move(callback));
}

clipp::group TurretRfControl::program_options() {
  return {};
}

}
}
