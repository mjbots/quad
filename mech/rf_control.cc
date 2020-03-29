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

#include "mech/rf_control.h"

#include "mjlib/io/now.h"

namespace pl = std::placeholders;

namespace mjmech {
namespace mech {

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

class RfControl::Impl {
 public:
  Impl(const base::Context& context,
       QuadrupedControl* quadruped_control,
       RfGetter rf_getter)
      : executor_(context.executor),
        quadruped_control_(quadruped_control),
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
        &bitfield_, std::bind(&Impl::HandleSlot, this, pl::_1));
  }

  void HandleSlot(const mjlib::base::error_code& ec) {
    mjlib::base::FailIf(ec);

    slot_data_.timestamp = mjlib::io::Now(executor_.context());

    for (int i = 0; i < 15; i++) {
      if (!(bitfield_ & (1 << i))) { continue; }

      const auto slot = rf_->rx_slot(i);
      slot_data_.rx[i].timestamp = slot.timestamp;
      slot_data_.rx[i].size = slot.size;
      std::memcpy(&slot_data_.rx[i].data[0], &slot.data[0], slot.size);
      std::memset(&slot_data_.rx[i].data[slot.size], 0, 15 - slot.size);
    }

    slotrf_signal_(&slot_data_);
    StartRead();
  }

  boost::asio::executor executor_;
  QuadrupedControl* const quadruped_control_;
  RfGetter rf_getter_;

  RfClient* rf_ = nullptr;

  uint16_t bitfield_ = 0;

  SlotData slot_data_;
  boost::signals2::signal<void (const SlotData*)> slotrf_signal_;
};

RfControl::RfControl(const base::Context& context,
                     QuadrupedControl* quadruped_control,
                     RfGetter rf_getter)
    : impl_(std::make_unique<Impl>(context, quadruped_control, rf_getter)) {}

RfControl::~RfControl() {}

void RfControl::AsyncStart(mjlib::io::ErrorCallback callback) {
  impl_->AsyncStart(std::move(callback));
}

clipp::group RfControl::program_options() {
  return {};
}

}
}
