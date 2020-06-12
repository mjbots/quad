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

#include "mech/pi3hat_wrapper.h"

#include <thread>

#include <fmt/format.h>

#include <boost/asio/io_context.hpp>
#include <boost/asio/post.hpp>

#include "mjlib/base/fail.h"
#include "mjlib/base/system_error.h"
#include "mjlib/io/now.h"

#include "mjbots/pi3hat/pi3hat.h"

namespace mjmech {
namespace mech {

class Pi3hatWrapper::Impl {
 public:
  Impl(const boost::asio::executor& executor, const Options& options)
      : executor_(executor),
        options_(options) {
    thread_ = std::thread(std::bind(&Impl::CHILD_Run, this));
  }

  ~Impl() {
    child_context_.stop();
    thread_.join();
  }

  void AsyncStart(mjlib::io::ErrorCallback callback) {
    boost::asio::post(
        executor_,
          std::bind(std::move(callback), mjlib::base::error_code()));
  }

  void ReadImu(AttitudeData* data,
               mjlib::io::ErrorCallback callback) {
    attitude_ = data;
    attitude_callback_ = std::move(callback);
  }

  void AsyncWaitForSlot(
      int* remote,
      uint16_t* bitfield,
      mjlib::io::ErrorCallback callback) {
    rf_remote_ = remote;
    rf_bitfield_ = bitfield;
    rf_callback_ = std::move(callback);
  }

  Slot rx_slot(int remote, int slot_idx) {
    return rf_rx_slots_[slot_idx];
  }

  void tx_slot(int remote, int slot_idx, const Slot& slot) {
    rf_tx_slots_[slot_idx] = slot;
    rf_to_send_ |= (1 << slot_idx);
  }

  Slot tx_slot(int remote, int slot_idx) {
    // We only support one remote.
    return rf_tx_slots_[slot_idx];
  }

  void AsyncRegister(const IdRequest& request,
                     SingleReply* reply,
                     mjlib::io::ErrorCallback callback) {
    single_request_ = { request };
    single_reply_ = {};

    AsyncRegisterMultiple(
        single_request_, &single_reply_,
        [this, reply, callback=std::move(callback)](const auto& ec) mutable {
          if (!this->single_reply_.replies.empty()) {
            *reply = this->single_reply_.replies.front();
          }
          callback(ec);
        });
  }

  void AsyncRegisterMultiple(
      const std::vector<IdRequest>& request,
      Reply* reply,
      mjlib::io::ErrorCallback callback) {
    if (rf_to_send_) {
      // Copy all the RF data to the child.
      pi3data_.rf_tx_slots = rf_tx_slots_;
      pi3data_.rf_to_send = rf_to_send_;
      rf_to_send_ = 0;
    }
    boost::asio::post(
        child_context_,
        [this, callback=std::move(callback), &request, reply,
         request_attitude=(attitude_ != nullptr),
         request_rf=rf_remote_ != nullptr]() mutable {
          this->CHILD_Register(
              &request, reply,
              request_attitude, request_rf,
              std::move(callback));
        });
  }

  mjlib::io::SharedStream MakeTunnel(
      uint8_t id,
      uint32_t channel,
      const TunnelOptions& options) {
    mjlib::base::AssertNotReached();
  }

 private:
  void CHILD_Run() {
    if (options_.cpu_affinity >= 0) {
      cpu_set_t cpuset = {};
      CPU_ZERO(&cpuset);
      CPU_SET(options_.cpu_affinity, &cpuset);

      mjlib::base::system_error::throw_if(
          ::sched_setaffinity(0, sizeof(cpu_set_t), &cpuset) < 0,
          "error setting affinity");

      std::cout << fmt::format(
          "Rpi3HatAuxStm32 cpu affinity set to {}\n", options_.cpu_affinity);
    }

    pi3hat_.emplace([&]() {
        mjbots::pi3hat::Pi3Hat::Configuration c;
        c.spi_speed_hz = options_.spi_speed_hz;
        c.mounting_deg.yaw = options_.mounting.yaw_deg;
        c.mounting_deg.pitch = options_.mounting.pitch_deg;
        c.mounting_deg.roll = options_.mounting.roll_deg;
        c.attitude_rate_hz = options_.imu_rate_hz;
        c.rf_id = options_.rf_id;
        return c;
      }());

    boost::asio::io_context::work work{child_context_};
    child_context_.run();

    // Destroy before we finish.
    pi3hat_.reset();
  }

  void CHILD_Register(const std::vector<IdRequest>* requests,
                      Reply* reply,
                      bool request_attitude,
                      bool request_rf,
                      mjlib::io::ErrorCallback callback) {
    mjbots::pi3hat::Pi3Hat::Input input;

    auto& d = pi3data_;
    d.tx_can.clear();
    d.tx_rf.clear();

    if (d.rf_to_send) {
      for (int i = 0; i < 15; i++) {
        if ((d.rf_to_send & (1 << i)) == 0) { continue; }

        const auto& src = d.rf_tx_slots[i];

        mjbots::pi3hat::RfSlot rf_slot;
        rf_slot.slot = i;
        rf_slot.priority = src.priority;
        rf_slot.size = src.size;
        std::memcpy(&rf_slot.data[0], &src.data[0], src.size);

        d.tx_rf.push_back(rf_slot);
      }

      input.tx_rf = {&d.tx_rf[0], d.tx_rf.size()};
    }

    for (const auto& request : *requests) {
      d.tx_can.push_back({});
      auto& dst = d.tx_can.back();
      dst.id = request.id | (request.request.request_reply() ? 0x8000 : 0x00);
      dst.size = request.request.buffer().size();
      std::memcpy(&dst.data[0], request.request.buffer().data(), dst.size);
      dst.bus =
          (request.id >= 1 && request.id <= 3) ? 1 :
          (request.id >= 4 && request.id <= 6) ? 2 :
          (request.id >= 7 && request.id <= 9) ? 3 :
          (request.id >= 10 && request.id <= 12) ? 4 :
          1; // just send everything else out to 1 by default
      dst.expect_reply = request.request.request_reply();
    }

    if (d.tx_can.size()) {
      input.tx_can = {&d.tx_can[0], d.tx_can.size()};
    }

    d.rx_rf.resize(16);
    input.rx_rf = {&d.rx_rf[0], d.rx_rf.size()};

    d.rx_can.resize(std::max<size_t>(d.tx_can.size() * 2, 24));
    input.rx_can = {&d.rx_can[0], d.rx_can.size()};

    input.attitude = &pi3data_.attitude;
    input.request_attitude = request_attitude;
    input.wait_for_attitude = request_attitude;
    input.request_attitude_detail = true;
    input.request_rf = request_rf;
    input.timeout_ns = options_.query_timeout_s * 1e9;

    pi3data_.result = pi3hat_->Cycle(input);

    // Now come back to the main thread.
    boost::asio::post(
        executor_,
        [this, callback=std::move(callback), reply]() mutable {
          this->FinishRegister(reply, std::move(callback));
        });
  }

  void FinishRegister(Reply* reply, mjlib::io::ErrorCallback callback) {
    const auto now = mjlib::io::Now(executor_.context());

    // Now we unpack the results and fire off the callbacks we can.

    // First CAN.
    for (size_t i = 0; i < pi3data_.result.rx_can_size; i++) {
      reply->replies.push_back({});
      const auto& src = pi3data_.rx_can[i];
      auto& this_reply = reply->replies.back();
      mjlib::base::BufferReadStream payload_stream{
        {reinterpret_cast<const char*>(&src.data[0]),
              pi3data_.rx_can[i].size}};
      this_reply.id = (src.id >> 8) & 0xff;
      this_reply.reply = mjlib::multiplex::ParseRegisterReply(payload_stream);
    }

    // Then IMU
    if (pi3data_.result.attitude_present && attitude_) {
      auto make_point = [](const auto& p) {
        return base::Point3D(p.x, p.y, p.z);
      };
      auto make_quat = [](const auto& q) {
        return base::Quaternion(q.w, q.x, q.y, q.z);
      };
      attitude_->timestamp = now;
      attitude_->attitude = make_quat(pi3data_.attitude.attitude);
      attitude_->rate_dps = make_point(pi3data_.attitude.rate_dps);
      attitude_->euler_deg = (180.0 / M_PI) * attitude_->attitude.euler_rad();
      attitude_->accel_mps2 = make_point(pi3data_.attitude.accel_mps2);
      attitude_->bias_dps = make_point(pi3data_.attitude.bias_dps);
      attitude_->attitude_uncertainty =
          make_quat(pi3data_.attitude.attitude_uncertainty);
      attitude_->bias_uncertainty_dps =
          make_point(pi3data_.attitude.bias_uncertainty_dps);

      boost::asio::post(
          executor_,
          std::bind(std::move(attitude_callback_), mjlib::base::error_code()));
      attitude_ = nullptr;
      attitude_callback_ = {};
    }

    // Then RF.
    if (rf_remote_) {
      *rf_remote_ = 0;
      *rf_bitfield_ = [&]() {
        uint16_t result = 0;
        for (size_t i = 0; i < pi3data_.result.rx_rf_size; i++) {
          const auto& src = pi3data_.rx_rf[i];
          result |= (1 << src.slot);
          auto& dst = rf_rx_slots_[src.slot];
          dst.size = src.size;
          std::memcpy(&dst.data[0], &src.data[0], src.size);
          dst.timestamp = now;
        }
        return result;
      }();

      boost::asio::post(
          executor_,
          std::bind(std::move(rf_callback_), mjlib::base::error_code()));

      rf_remote_ = nullptr;
      rf_bitfield_ = nullptr;
      rf_callback_ = {};
    }

    // Finally, post our CAN response.
    boost::asio::post(
        executor_,
        std::bind(std::move(callback), mjlib::base::error_code()));
  }

  boost::asio::executor executor_;
  const Options options_;

  std::thread thread_;

  std::vector<IdRequest> single_request_;
  Reply single_reply_;

  AttitudeData* attitude_ = nullptr;
  mjlib::io::ErrorCallback attitude_callback_;

  int* rf_remote_ = nullptr;
  uint16_t* rf_bitfield_ = nullptr;
  mjlib::io::ErrorCallback rf_callback_;

  std::array<Slot, 16> rf_rx_slots_ = {};
  std::array<Slot, 16> rf_tx_slots_ = {};
  uint16_t rf_to_send_ = 0;

  // Only accessed from the thread.
  std::optional<mjbots::pi3hat::Pi3Hat> pi3hat_;
  boost::asio::io_context child_context_;

  // The following are accessed by both threads, but never at the same
  // time.  They can either be accessed inside CHILD_Register, or in
  // the parent until the callback is invoked.
  struct Pi3Data {
    std::vector<mjbots::pi3hat::CanFrame> tx_can;
    std::vector<mjbots::pi3hat::CanFrame> rx_can;
    std::vector<mjbots::pi3hat::RfSlot> tx_rf;
    std::vector<mjbots::pi3hat::RfSlot> rx_rf;
    mjbots::pi3hat::Attitude attitude;
    std::array<Slot, 16> rf_tx_slots = {};
    uint16_t rf_to_send = 0;

    mjbots::pi3hat::Pi3Hat::Output result;
  };
  Pi3Data pi3data_;
};

Pi3hatWrapper::Pi3hatWrapper(const boost::asio::executor& executor,
                                 const Options& options)
    : impl_(std::make_unique<Impl>(executor, options)) {}

Pi3hatWrapper::~Pi3hatWrapper() {}

void Pi3hatWrapper::AsyncStart(mjlib::io::ErrorCallback callback) {
  impl_->AsyncStart(std::move(callback));
}

void Pi3hatWrapper::ReadImu(AttitudeData* data,
                              mjlib::io::ErrorCallback callback) {
  impl_->ReadImu(data, std::move(callback));
}

void Pi3hatWrapper::AsyncWaitForSlot(
    int* remote,
    uint16_t* bitfield,
    mjlib::io::ErrorCallback callback) {
  impl_->AsyncWaitForSlot(remote, bitfield, std::move(callback));
}

Pi3hatWrapper::Slot Pi3hatWrapper::rx_slot(int remote, int slot_idx) {
  return impl_->rx_slot(remote, slot_idx);
}

void Pi3hatWrapper::tx_slot(int remote, int slot_idx, const Slot& slot) {
  impl_->tx_slot(remote, slot_idx, slot);
}

Pi3hatWrapper::Slot Pi3hatWrapper::tx_slot(int remote, int slot_idx) {
  return impl_->tx_slot(remote, slot_idx);
}

void Pi3hatWrapper::AsyncRegister(const IdRequest& request,
                                    SingleReply* reply,
                                    mjlib::io::ErrorCallback callback) {
  impl_->AsyncRegister(request, reply, std::move(callback));
}

void Pi3hatWrapper::AsyncRegisterMultiple(
    const std::vector<IdRequest>& request,
    Reply* reply,
    mjlib::io::ErrorCallback callback) {
  impl_->AsyncRegisterMultiple(request, reply, std::move(callback));
}

mjlib::io::SharedStream Pi3hatWrapper::MakeTunnel(
    uint8_t id,
    uint32_t channel,
    const TunnelOptions& options) {
  return impl_->MakeTunnel(id, channel, options);
}

Pi3hatWrapper::Stats Pi3hatWrapper::stats() const {
  return Stats();
}

}
}
