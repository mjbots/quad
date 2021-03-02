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

#include <functional>
#include <thread>

#include <fmt/format.h>

#include <boost/asio/io_context.hpp>
#include <boost/asio/post.hpp>

#include "mjlib/base/fail.h"
#include "mjlib/base/system_error.h"
#include "mjlib/io/deadline_timer.h"
#include "mjlib/io/now.h"
#include "mjlib/io/repeating_timer.h"

#ifdef COM_GITHUB_MJBOTS_RASPBERRYPI
#include "mjbots/pi3hat/pi3hat.h"
#endif

#include "base/logging.h"
#include "base/saturate.h"

#include "mech/moteus.h"

namespace mjmech {
namespace mech {

namespace {
template <typename T>
uint32_t u32(T value) {
  return static_cast<uint32_t>(value);
}
}

#ifdef COM_GITHUB_MJBOTS_RASPBERRYPI
class Pi3hatWrapper::Impl {
 public:
  Impl(const boost::asio::any_io_executor& executor, const Options& options)
      : executor_(executor),
        options_(options),
        power_poll_timer_(executor) {
    thread_ = std::thread(std::bind(&Impl::CHILD_Run, this));
  }

  ~Impl() {
    child_context_.stop();
    thread_.join();
  }

  void AsyncStart(mjlib::io::ErrorCallback callback) {
    power_poll_timer_.start(base::ConvertSecondsToDuration(
                                options_.power_poll_period_s),
                            std::bind(&Impl::HandlePowerPoll, this,
                                      std::placeholders::_1));
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

  void AsyncTransmit(
      const Request* request,
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
        [this, callback=std::move(callback), request, reply,
         request_attitude=(attitude_ != nullptr),
         request_rf=rf_remote_ != nullptr]() mutable {
          this->CHILD_Transmit(
              request, reply,
              request_attitude, request_rf,
              std::move(callback));
        });
  }

  void Cycle(
      AttitudeData* attitude,
      const Request* request,
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
        [this, callback=std::move(callback), attitude, request, reply,
         request_rf=(rf_remote_ != nullptr)]() mutable {
          this->CHILD_Cycle(
              attitude, request, reply, request_rf,
              std::move(callback));
        });
  }

  mjlib::io::SharedStream MakeTunnel(
      uint8_t id,
      uint32_t channel,
      const TunnelOptions& options) {
    return std::make_shared<Tunnel>(this, id, channel, options);
  }

  PowerSignal* power_signal() { return &power_signal_; }

 private:
  void HandlePowerPoll(const mjlib::base::error_code& ec) {
    if (ec == boost::asio::error::operation_aborted) {
      return;
    }
    mjlib::base::FailIf(ec);

    power_poll_.store(true);
  }

  class Tunnel : public mjlib::io::AsyncStream,
                 public std::enable_shared_from_this<Tunnel> {
   public:
    Tunnel(Impl* parent, uint8_t id, uint32_t channel,
           const TunnelOptions& options)
        : parent_(parent),
          id_(id),
          channel_(channel),
          options_(options) {}

    ~Tunnel() override {}

    void async_read_some(mjlib::io::MutableBufferSequence buffers,
                         mjlib::io::ReadHandler handler) override {
      if (boost::asio::buffer_size(buffers) == 0) {
        // Post immediately.
        boost::asio::post(
            parent_->executor_,
            std::bind(std::move(handler), mjlib::base::error_code(), 0));
        return;
      }

      boost::asio::post(
          parent_->child_context_,
          [self=shared_from_this(), buffers,
           handler=std::move(handler)]() mutable {
            const auto bytes_read = self->parent_->CHILD_TunnelPoll(
                self->id_, self->channel_, buffers);
            if (bytes_read > 0) {
              boost::asio::post(
                  self->parent_->executor_,
                  [self, handler=std::move(handler), bytes_read]() mutable {
                    handler(mjlib::base::error_code(), bytes_read);
                  });
            } else {
              self->timer_.expires_from_now(self->options_.poll_rate);
              self->timer_.async_wait(
                  [self, handler=std::move(handler), buffers](
                      const auto& ec) mutable {
                    self->HandlePoll(ec, buffers, std::move(handler));
                  });
            }
          });
    }

    void async_write_some(mjlib::io::ConstBufferSequence buffers,
                          mjlib::io::WriteHandler handler) override {
      boost::asio::post(
          parent_->child_context_,
          [self=shared_from_this(), buffers, handler=std::move(handler)]() mutable {
            self->parent_->CHILD_TunnelWrite(
                self->id_, self->channel_, buffers, std::move(handler));
          });
    }

    boost::asio::any_io_executor get_executor() override {
      return parent_->executor_;
    }

    void cancel() override {
      timer_.cancel();
    }

   private:
    void HandlePoll(const mjlib::base::error_code& ec,
                    mjlib::io::MutableBufferSequence buffers,
                    mjlib::io::ReadHandler handler) {
      if (ec) {
        handler(ec, 0);
        return;
      }

      async_read_some(buffers, std::move(handler));
    }

    Impl* const parent_;
    const uint8_t id_;
    const uint32_t channel_;
    const TunnelOptions options_;

    mjlib::io::DeadlineTimer timer_{parent_->executor_};
  };

  void CHILD_Run() {
    if (options_.cpu_affinity >= 0) {
      cpu_set_t cpuset = {};
      CPU_ZERO(&cpuset);
      CPU_SET(options_.cpu_affinity, &cpuset);

      mjlib::base::system_error::throw_if(
          ::sched_setaffinity(0, sizeof(cpu_set_t), &cpuset) < 0,
          "error setting affinity");

      std::cout << fmt::format(
          "pi3hat cpu affinity set to {}\n", options_.cpu_affinity);
    }

    pi3hat_.emplace([&]() {
        mjbots::pi3hat::Pi3Hat::Configuration c;
        c.spi_speed_hz = options_.spi_speed_hz;
        c.mounting_deg.yaw = options_.mounting.yaw_deg;
        c.mounting_deg.pitch = options_.mounting.pitch_deg;
        c.mounting_deg.roll = options_.mounting.roll_deg;
        c.attitude_rate_hz = options_.imu_rate_hz;
        c.rf_id = options_.rf_id;

        if (options_.power_dist_rev >= 0x0400) {
          // The current power_dist boards use 1/5Mbps CAN-FD
          c.can[4].slow_bitrate = 1000000;
          c.can[4].fast_bitrate = 5000000;
          c.can[4].fdcan_frame = true;
          c.can[4].bitrate_switch = true;
        }

        return c;
      }());

    boost::asio::io_context::work work{child_context_};
    child_context_.run();

    // Destroy before we finish.
    pi3hat_.reset();
  }

  void CHILD_SetupRf(mjbots::pi3hat::Pi3Hat::Input* input) {
    auto& d = pi3data_;
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

      input->tx_rf = {&d.tx_rf[0], d.tx_rf.size()};
    }

    d.rx_rf.resize(16);
    input->rx_rf = {&d.rx_rf[0], d.rx_rf.size()};
  }

  void CHILD_SetupCAN(mjbots::pi3hat::Pi3Hat::Input* input,
                      const Request* requests) {
    auto& d = pi3data_;
    d.tx_can.clear();

    for (const auto& request : *requests) {
      d.tx_can.push_back({});
      auto& dst = d.tx_can.back();
      dst.id = request.id | (request.request.request_reply() ? 0x8000 : 0x00);
      dst.size = request.request.buffer().size();
      std::memcpy(&dst.data[0], request.request.buffer().data(), dst.size);
      dst.bus = SelectBus(request.id);
      dst.expect_reply = request.request.request_reply();
    }

    const bool power_poll = power_poll_.exchange(false);
    if (power_poll) {
      d.tx_can.push_back({});
      auto& dst = d.tx_can.back();
      dst.bus = 5;  // The auxiliary CAN bus

      if (options_.power_dist_rev >= 0x0400) {
        dst.id = 0x8020;

        dst.size = 9;
        dst.data[0] = 0x01;  // write 1 int8 register
        dst.data[1] = 0x03;  // register 3 = Lock Time
        dst.data[2] = base::Saturate<int8_t>(options_.shutdown_timeout_s / 0.1);
        dst.data[3] = 0x17;  // read 3 int16 registers
        dst.data[4] = 0x10;  // 0x010=voltage, 0x011=current, 0x012=temp

        dst.data[5] = 0x19;  // read 1 int32 register
        dst.data[6] = 0x13;  // 0x013 = energy
        dst.data[7] = 0x11;  // read 1 int8 register
        dst.data[8] = 0x02;  // 0x002=switch

        dst.expect_reply = true;
      } else {
        dst.id = 0x00010005;
        dst.size = 2;
        dst.data[0] = 0;
        dst.data[1] = base::Saturate<uint8_t>(options_.shutdown_timeout_s / 0.1);
      }

      input->force_can_check |= (1 << 5);
    }

    if (d.tx_can.size()) {
      input->tx_can = {&d.tx_can[0], d.tx_can.size()};
    }

    d.rx_can.resize(std::max<size_t>(d.tx_can.size() * 2, 24));
    input->rx_can = {&d.rx_can[0], d.rx_can.size()};
  }

  void CHILD_Cycle(AttitudeData* attitude_dest,
                   const Request* request,
                   Reply* reply,
                   bool request_rf,
                   mjlib::io::ErrorCallback callback) {
    mjbots::pi3hat::Pi3Hat::Input input;

    CHILD_SetupRf(&input);
    CHILD_SetupCAN(&input, request);

    input.attitude = &pi3data_.attitude;
    input.request_attitude = true;
    input.wait_for_attitude = true;
    input.request_attitude_detail = options_.attitude_detail;
    input.request_rf = request_rf;
    input.timeout_ns = options_.query_timeout_s * 1e9;
    input.rx_extra_wait_ns = 0;

    pi3data_.result = pi3hat_->Cycle(input);

    // Now come back to the main thread.
    boost::asio::post(
        executor_,
        [this, callback=std::move(callback), attitude_dest, reply]() mutable {
          this->FinishCycle(attitude_dest, reply, std::move(callback));
        });
  }

  void CHILD_Transmit(const Request* request,
                      Reply* reply,
                      bool request_attitude,
                      bool request_rf,
                      mjlib::io::ErrorCallback callback) {
    mjbots::pi3hat::Pi3Hat::Input input;

    CHILD_SetupRf(&input);
    CHILD_SetupCAN(&input, request);

    input.attitude = &pi3data_.attitude;
    input.request_attitude = request_attitude;
    input.wait_for_attitude = false;
    input.request_attitude_detail = options_.attitude_detail;
    input.request_rf = request_rf;
    input.timeout_ns = options_.query_timeout_s * 1e9;

    pi3data_.result = pi3hat_->Cycle(input);

    // Now come back to the main thread.
    boost::asio::post(
        executor_,
        [this, callback=std::move(callback), reply]() mutable {
          this->FinishTransmit(reply, std::move(callback));
        });
  }

  size_t CHILD_TunnelPoll(uint8_t id, uint32_t channel,
                          mjlib::io::MutableBufferSequence buffers) {
    mjbots::pi3hat::Pi3Hat::Input input;

    if (pi3data_.rx_can.size() < 4) {
      pi3data_.rx_can.resize(4);
    }

    for (auto& frame : pi3data_.rx_can) {
      std::memset(&frame.data[0], 0, sizeof(frame.data));
    }
    input.rx_can = {&pi3data_.rx_can[0], pi3data_.rx_can.size()};
    input.force_can_check = (1 << SelectBus(id));
    input.timeout_ns = 0;
    input.min_tx_wait_ns = 0;

    // Check for anything lying around first.
    pi3data_.result = pi3hat_->Cycle(input);

    if (pi3data_.result.rx_can_size > 0) {
      return CHILD_ParseTunnelPoll(id, channel, buffers);
    }

    input.timeout_ns = options_.query_timeout_s * 1e9;
    input.min_tx_wait_ns = options_.min_wait_s * 1e9;

    // Nope, so we should poll.
    if (pi3data_.tx_can.size() < 1) {
      pi3data_.tx_can.resize(1);
    }

    auto& out_frame = pi3data_.tx_can[0];
    mjlib::base::BufferWriteStream stream{
      {reinterpret_cast<char*>(&out_frame.data[0]), sizeof(out_frame.data)}};
    mjlib::multiplex::WriteStream writer{stream};

    writer.WriteVaruint(
        u32(mjlib::multiplex::Format::Subframe::kClientPollServer));
    writer.WriteVaruint(channel);
    writer.WriteVaruint(
        std::min<uint32_t>(48, boost::asio::buffer_size(buffers)));

    out_frame.expect_reply = true;
    out_frame.bus = SelectBus(id);
    out_frame.id = 0x8000 | id;
    out_frame.size = stream.offset();

    input.tx_can = {&pi3data_.tx_can[0], 1};

    pi3data_.result = pi3hat_->Cycle(input);

    return CHILD_ParseTunnelPoll(id, channel, buffers);
  }

  size_t CHILD_ParseTunnelPoll(uint8_t id, uint32_t channel,
                               mjlib::io::MutableBufferSequence buffers) {
    size_t result = 0;

    for (size_t i = 0; i < pi3data_.result.rx_can_size; i++) {
      const auto& src = pi3data_.rx_can[i];
      if (((src.id >> 8) & 0xff) != id) { continue; }

      mjlib::base::BufferReadStream buffer_stream{
        {reinterpret_cast<const char*>(&src.data[0]), src.size}};
      mjlib::multiplex::ReadStream<
        mjlib::base::BufferReadStream> stream{buffer_stream};

      const auto maybe_subframe = stream.ReadVaruint();
      if (!maybe_subframe || *maybe_subframe !=
          u32(mjlib::multiplex::Format::Subframe::kServerToClient)) {
        continue;
      }

      const auto maybe_channel = stream.ReadVaruint();
      if (!maybe_channel || *maybe_channel != channel) {
        continue;
      }

      const auto maybe_stream_size = stream.ReadVaruint();
      if (!maybe_stream_size) {
        continue;
      }

      const auto stream_size = *maybe_stream_size;
      if (stream_size == 0) { continue; }

      auto remaining_data = stream_size;
      for (auto buffer : buffers) {
        if (remaining_data == 0) { break; }

        const auto to_read = std::min<size_t>(buffer.size(), remaining_data);
        buffer_stream.read({static_cast<char*>(buffer.data()),
                static_cast<std::streamsize>(to_read)});
        remaining_data -= to_read;
        result += to_read;
      }
    }

    return result;
  }

  void CHILD_TunnelWrite(uint8_t id, uint32_t channel,
                         mjlib::io::ConstBufferSequence buffers,
                         mjlib::io::WriteHandler callback) {
    if (pi3data_.tx_can.size() < 1) {
      pi3data_.tx_can.resize(1);
    }
    auto& dst = pi3data_.tx_can[0];

    mjlib::base::BufferWriteStream stream{
      {reinterpret_cast<char*>(&dst.data[0]), sizeof(dst.data)}};
    mjlib::multiplex::WriteStream writer{stream};

    writer.WriteVaruint(u32(mjlib::multiplex::Format::Subframe::kClientToServer));
    writer.WriteVaruint(channel);

    const auto size = std::min<size_t>(48, boost::asio::buffer_size(buffers));
    writer.WriteVaruint(size);
    auto remaining_size = size;
    for (auto buffer : buffers) {
      if (remaining_size == 0) { break; }
      const auto to_write = std::min(remaining_size, buffer.size());
      stream.write({static_cast<const char*>(buffer.data()), to_write});
      remaining_size -= to_write;
    }

    dst.id = id;
    dst.bus = SelectBus(id);
    dst.size = stream.offset();

    mjbots::pi3hat::Pi3Hat::Input input;
    input.tx_can = {&pi3data_.tx_can[0], 1};

    pi3hat_->Cycle(input);

    boost::asio::post(
        executor_,
        std::bind(std::move(callback), mjlib::base::error_code(), size));
  }

  void Shutdown() {
    // The power switch is off.  Shut everything down!
    log_.warn("Shutting down!");
    mjlib::base::system_error::throw_if(
        ::system("shutdown -h now") < 0,
        "error trying to shutdown, at least we'll exit the app");
    // Quit the application to make the shutdown process go
    // faster.
    std::exit(0);
  }

  void FinishCAN(Reply* reply) {
    // First CAN.
    for (size_t i = 0; i < pi3data_.result.rx_can_size; i++) {
      const auto& src = pi3data_.rx_can[i];

      if (src.id == 0x10004) {
        // This came from a power_dist r3.x board.
        const bool power_switch = src.data[0] != 0;
        if (!power_switch) {
          Shutdown();
        }

        continue;
      }

      parsed_data_.clear();
      mjlib::base::BufferReadStream payload_stream{
        {reinterpret_cast<const char*>(&src.data[0]),
              pi3data_.rx_can[i].size}};
      mjlib::multiplex::ParseRegisterReply(payload_stream, &parsed_data_);

      if ((src.id & 0xff00) == 0x2000) {
        // This came from a power_dist r4.x board.

        Power power;

        for (const auto& pair : parsed_data_) {
          const auto* maybe_value =
              std::get_if<moteus::Value>(&pair.second);
          if (!maybe_value) { continue; }
          const auto& value = *maybe_value;

          if (pair.first == 0x010) {
            power.output_V = moteus::ReadVoltage(value);
          } else if (pair.first == 0x011) {
            power.output_A = moteus::ReadCurrent(value);
          } else if (pair.first == 0x012) {
            power.temp_C = moteus::ReadTemperature(value);
          } else if (pair.first == 0x013) {
            power.energy_Whr = moteus::ReadEnergy(value);
          } else if (pair.first == 0x002) {
            if (moteus::ReadInt(value) == 0) {
              Shutdown();
            }
          }
        }

        const auto now = mjlib::io::Now(executor_.context());
        power.timestamp = now;

        power.power_W = power.output_V * power.output_A;

        const double delta_hr =
            last_power_status_.is_not_a_date_time() ? 10.0 :
            base::ConvertDurationToSeconds(
                now - last_power_status_) / 3600.0;
        if (std::abs(delta_hr) > 1e-6) {
          // about 4ms
          if (!last_power_status_.is_not_a_date_time()) {
            power.avg_power_W =
                (power.energy_Whr - last_energy_Whr_) / delta_hr;
          }
          last_power_status_ = now;
          last_energy_Whr_ = power.energy_Whr;
        }

        power_signal_(&power);
        continue;
      }

      for (const auto& pair : parsed_data_) {
        reply->push_back({static_cast<uint8_t>((src.id >> 8) & 0xff),
                pair.first, pair.second});
      }
    }
  }

  void FinishAttitude(boost::posix_time::ptime now, AttitudeData* attitude) {
    auto make_point = [](const auto& p) {
      return base::Point3D(p.x, p.y, p.z);
    };
    auto make_quat = [](const auto& q) {
      return base::Quaternion(q.w, q.x, q.y, q.z);
    };
    attitude->timestamp = now;
    attitude->attitude = make_quat(pi3data_.attitude.attitude);
    attitude->rate_dps = make_point(pi3data_.attitude.rate_dps);
    attitude->euler_deg = (180.0 / M_PI) * attitude->attitude.euler_rad();
    attitude->accel_mps2 = make_point(pi3data_.attitude.accel_mps2);
    attitude->bias_dps = make_point(pi3data_.attitude.bias_dps);
    attitude->attitude_uncertainty =
        make_quat(pi3data_.attitude.attitude_uncertainty);
    attitude->bias_uncertainty_dps =
        make_point(pi3data_.attitude.bias_uncertainty_dps);
  }

  void FinishRF(boost::posix_time::ptime now) {
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
  }


  void FinishCycle(AttitudeData* attitude,
                   Reply* reply,
                   mjlib::io::ErrorCallback callback) {
    const auto now = mjlib::io::Now(executor_.context());

    FinishCAN(reply);
    FinishAttitude(now, attitude);
    FinishRF(now);

    boost::asio::post(
        executor_,
        std::bind(std::move(callback), mjlib::base::error_code()));
  }

  void FinishTransmit(Reply* reply, mjlib::io::ErrorCallback callback) {
    const auto now = mjlib::io::Now(executor_.context());

    FinishCAN(reply);

    if (attitude_) {
      FinishAttitude(now, attitude_);
      boost::asio::post(
          executor_,
          std::bind(std::move(attitude_callback_), mjlib::base::error_code()));
      attitude_ = nullptr;
      attitude_callback_ = {};
    }
    FinishRF(now);


    // Finally, post our CAN response.
    boost::asio::post(
        executor_,
        std::bind(std::move(callback), mjlib::base::error_code()));
  }

  int SelectBus(int id) const {
    if (options_.force_bus >= 0) { return options_.force_bus; }

    return (id >= 1 && id <= 3) ? 1 :
        (id >= 4 && id <= 6) ? 2 :
        (id >= 7 && id <= 9) ? 3 :
        (id >= 10 && id <= 12) ? 4 :
        1; // just send everything else out to 1 by default
  }

  base::LogRef log_ = base::GetLogInstance("Pi3hatWrapper");

  boost::asio::any_io_executor executor_;
  const Options options_;

  mjlib::io::RepeatingTimer power_poll_timer_;

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

  // A cache to hold parsed register data.
  std::vector<mjlib::multiplex::RegisterValue> parsed_data_;

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

  std::atomic<bool> power_poll_{false};

  double last_energy_Whr_ = 0.0;
  boost::posix_time::ptime last_power_status_;
  PowerSignal power_signal_;
};
#else

class Pi3hatWrapper::Impl {
 public:
  Impl(const boost::asio::any_io_executor&, const Options&) {}
  void AsyncStart(mjlib::io::ErrorCallback) {}
  void ReadImu(AttitudeData*, mjlib::io::ErrorCallback) {}
  void AsyncWaitForSlot(int*, uint16_t*, mjlib::io::ErrorCallback) {}
  Slot rx_slot(int, int) { return {}; }
  void tx_slot(int, int, const Slot&) {}
  Slot tx_slot(int, int) { return {}; }
  void AsyncTransmit(const Request*, Reply*, mjlib::io::ErrorCallback) {}
  void Cycle(AttitudeData*, const Request*, Reply*,
             mjlib::io::ErrorCallback) {}
  mjlib::io::SharedStream MakeTunnel(uint8_t, uint32_t, const TunnelOptions&) {
    return {};
  }
  PowerSignal* power_signal() { return &power_signal_; }

  PowerSignal power_signal_;
};
#endif

Pi3hatWrapper::Pi3hatWrapper(const boost::asio::any_io_executor& executor,
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

void Pi3hatWrapper::AsyncTransmit(const Request* request,
                                  Reply* reply,
                                  mjlib::io::ErrorCallback callback) {
  impl_->AsyncTransmit(request, reply, std::move(callback));
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

void Pi3hatWrapper::Cycle(AttitudeData* attitude,
                          const Request* request,
                          Reply* reply,
                          mjlib::io::ErrorCallback callback) {
  impl_->Cycle(attitude, request, reply, std::move(callback));
}

Pi3hatWrapper::PowerSignal* Pi3hatWrapper::power_signal() {
  return impl_->power_signal();
}

}
}
