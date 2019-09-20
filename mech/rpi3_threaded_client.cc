// Copyright 2019 Josh Pieper, jjp@pobox.com.
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

#include "mech/rpi3_threaded_client.h"

#include <sched.h>
#include <linux/serial.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/select.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <future>
#include <optional>
#include <sstream>
#include <thread>

#include <boost/asio/io_context.hpp>
#include <boost/asio/post.hpp>
#include <boost/crc.hpp>

#include <fmt/format.h>

#include "mjlib/base/buffer_stream.h"
#include "mjlib/base/system_error.h"
#include "mjlib/io/deadline_timer.h"
#include "mjlib/multiplex/frame.h"
#include "mjlib/multiplex/stream.h"

#include "mech/rpi3_raw_uart.h"

namespace pl = std::placeholders;

namespace mjmech {
namespace mech {

namespace {
auto u32 = [](auto v) { return static_cast<uint32_t>(v); };

void ThrowIf(bool value, std::string_view message = "") {
  if (value) {
    throw mjlib::base::system_error::syserrno(message.data());
  }
}

std::string Hexify(const std::string_view& data) {
  std::ostringstream ostr;
  for (auto c : data) {
    ostr << fmt::format("{:02x}", static_cast<int>(c));
  }
  return ostr.str();
}

struct FrameItem {
  mjlib::multiplex::Frame frame;
  char encoded[256] = {};
  std::size_t size = 0;
};
}

class Rpi3ThreadedClient::Impl {
 public:
  Impl(const boost::asio::executor& executor,
       const Options& options)
      : executor_(executor),
        options_(options) {
    Start();
  }

  ~Impl() {
    child_context_.stop();
    thread_.join();
  }

  void Start() {
    thread_ = std::thread(std::bind(&Impl::Run, this));
    startup_promise_.set_value(true);
  }

  void AsyncRegister(const Request* request,
                     Reply* reply,
                     mjlib::io::ErrorCallback callback) {
    AssertParent();
    boost::asio::post(
        child_context_,
        std::bind(&Impl::ThreadAsyncRegister,
                  this, request, reply, callback));
  }

  Stats stats() const {
    Stats result;
    result.checksum_errors = checksum_errors_.load();
    result.timeouts = timeouts_.load();
    result.malformed = malformed_.load();
    result.serial_errors = serial_errors_.load();
    result.extra_found = extra_found_.load();
    return result;
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
      boost::asio::post(
          parent_->child_context_,
          [self=shared_from_this(), buffers, handler]() {
            const auto bytes_read =
                self->parent_->ThreadTunnelPoll(
                    self->id_, self->channel_, buffers);
            if (bytes_read > 0) {
              boost::asio::post(
                  self->parent_->executor_, [self, handler, bytes_read]() {
                    handler(mjlib::base::error_code(), bytes_read);
                  });
            } else {
              self->timer_.expires_from_now(self->options_.poll_rate);
              self->timer_.async_wait(
                  std::bind(&Tunnel::HandlePoll, self, pl::_1,
                            buffers, handler));
            }
          });
    }

    void async_write_some(mjlib::io::ConstBufferSequence buffers,
                          mjlib::io::WriteHandler handler) override {
      boost::asio::post(
          parent_->child_context_,
          [self=shared_from_this(), buffers, handler]() {
            self->parent_->ThreadTunnelWrite(
                self->id_, self->channel_, buffers, handler);
          });
    }

    boost::asio::executor get_executor() override {
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

      async_read_some(buffers, handler);
    }

    Impl* const parent_;
    const uint8_t id_;
    const uint32_t channel_;
    const TunnelOptions options_;

    mjlib::io::DeadlineTimer timer_{parent_->executor_};
  };

  mjlib::io::SharedStream MakeTunnel(
      uint8_t id,
      uint32_t channel,
      const TunnelOptions& options) {
    return std::make_shared<Tunnel>(this, id, channel, options);
  }

 private:
  void Run() {
    startup_future_.get();
    AssertThread();

    if (options_.cpu_affinity >= 0) {
      cpu_set_t cpuset = {};
      CPU_ZERO(&cpuset);
      CPU_SET(options_.cpu_affinity, &cpuset);

      ThrowIf(::sched_setaffinity(0, sizeof(cpu_set_t), &cpuset) < 0);
    }

    // TODO: Set realtime priority and lock memory.

    OpenPort();

    boost::asio::io_context::work work(child_context_);
    child_context_.run();
  }

  void OpenPort() {
    AssertThread();

    serial_.emplace([&]() {
          Rpi3RawUart::Options options;
          options.baud_rate = options_.baud_rate;
          return options;
      }());
  }

  size_t ThreadTunnelPoll(uint8_t id, uint32_t channel,
                          mjlib::io::MutableBufferSequence buffers) {
    AssertThread();

    mjlib::base::FastOStringStream stream;
    mjlib::multiplex::WriteStream writer{stream};

    writer.WriteVaruint(
        u32(mjlib::multiplex::Format::Subframe::kClientPollServer));
    writer.WriteVaruint(channel);
    writer.WriteVaruint(
        std::min<uint32_t>(64, boost::asio::buffer_size(buffers)));

    mjlib::multiplex::Frame frame;
    frame.source_id = 0;
    frame.dest_id = id;
    frame.request_reply = true;
    frame.payload = stream.str();

    auto result = frame.encode();

    serial_->write(result);

    FrameItem item;
    ReadFrame(&item);

    return ParseTunnelPoll(&item, channel, buffers);
  }

  size_t ParseTunnelPoll(const FrameItem* item,
                         uint32_t channel,
                         mjlib::io::MutableBufferSequence buffers) {
    AssertThread();

    mjlib::base::BufferReadStream buffer_stream(
        {item->encoded, item->size});
    mjlib::multiplex::ReadStream<
      mjlib::base::BufferReadStream> stream{buffer_stream};

    stream.Read<uint16_t>();  // ignore header
    const auto maybe_source = stream.Read<uint8_t>();
    const auto maybe_dest = stream.Read<uint8_t>();
    const auto packet_size = stream.ReadVaruint();

    if (!maybe_source || !maybe_dest || !packet_size) {
      malformed_++;
      return 0;
    }

    const auto maybe_subframe = stream.ReadVaruint();
    if (!maybe_subframe || *maybe_subframe !=
        u32(mjlib::multiplex::Format::Subframe::kServerToClient)) {
      malformed_++;
      return 0;
    }

    const auto maybe_channel = stream.ReadVaruint();
    if (!maybe_channel || *maybe_channel != channel) {
      malformed_++;
      return 0;
    }

    const auto maybe_stream_size = stream.ReadVaruint();
    if (!maybe_stream_size) {
      malformed_++;
      return 0;
    }

    const auto stream_size = *maybe_stream_size;

    auto remaining_data = stream_size;
    size_t bytes_read = 0;
    for (auto buffer : buffers) {
      if (remaining_data == 0) { break; }

      const auto to_read = std::min<size_t>(buffer.size(), remaining_data);
      buffer_stream.read({static_cast<char*>(buffer.data()),
              static_cast<std::streamsize>(to_read)});
      remaining_data -= to_read;
      bytes_read += to_read;
    }

    // Ignore anything left over.
    buffer_stream.ignore(remaining_data);

    const auto maybe_read_crc = stream.Read<uint16_t>();
    if (!maybe_read_crc) {
      malformed_++;
      return 0;
    }
    const auto read_crc = *maybe_read_crc;
    boost::crc_ccitt_type crc;
    crc.process_bytes(item->encoded, item->size - 2);
    const auto expected_crc = crc.checksum();

    if (read_crc != expected_crc) {
      checksum_errors_++;
      return 0;
    }

    return bytes_read;
  }

  void ThreadTunnelWrite(uint8_t id, uint32_t channel,
                         mjlib::io::ConstBufferSequence buffers,
                         mjlib::io::WriteHandler callback) {
    AssertThread();

    mjlib::base::FastOStringStream stream;
    mjlib::multiplex::WriteStream writer{stream};

    writer.WriteVaruint(u32(mjlib::multiplex::Format::Subframe::kClientToServer));
    writer.WriteVaruint(channel);

    const auto size = boost::asio::buffer_size(buffers);
    writer.WriteVaruint(size);
    for (auto buffer : buffers) {
      stream.write({static_cast<const char*>(buffer.data()), buffer.size()});
    }

    mjlib::multiplex::Frame frame;
    frame.source_id = 0;
    frame.dest_id = id;
    frame.request_reply = false;
    frame.payload = stream.str();

    auto result = frame.encode();

    serial_->write(result);

    // Give the device a chance to turn around the line before we do
    // anything else.
    ::usleep(100);

    boost::asio::post(
        executor_,
        std::bind(callback, mjlib::base::error_code(), size));
  }

  void ThreadAsyncRegister(const Request* requests,
                           Reply* reply,
                           mjlib::io::ErrorCallback callback) {
    AssertThread();

    MJ_ASSERT(!requests->requests.empty());

    FrameItem* prev_frame = nullptr;
    FrameItem* this_frame = &frame_item1_;
    FrameItem* next_frame = &frame_item2_;

    auto encode_frame = [&](FrameItem* frame_item,
                            const SingleRequest& next_request) {
      // Now we encode the next frame.
      frame_item->frame.source_id = 0;
      frame_item->frame.dest_id = next_request.id;
      frame_item->frame.request_reply = next_request.request.request_reply();
      frame_item->frame.payload = next_request.request.buffer();
      mjlib::base::BufferWriteStream stream{{frame_item->encoded,
              sizeof(frame_item->encoded)}};
      frame_item->frame.encode(&stream);
      frame_item->size = stream.offset();
    };

    encode_frame(this_frame, requests->requests.front());

    // We stick all the processing work we can right after sending a
    // request.  That way it overlaps with the time it takes for the
    // device to respond.
    for (size_t i = 0; i < requests->requests.size(); i++) {
      serial_->write({this_frame->encoded, this_frame->size});

      if (i + 1 < requests->requests.size()) {
        encode_frame(next_frame, requests->requests[i + 1]);
      }

      // Now we would parse the prior frame.
      if (prev_frame && prev_frame->frame.request_reply) {
        ParseFrame(prev_frame, reply);
      }

      if (this_frame->frame.request_reply) {
        // And now we read this one.  Re-purpose our 'buf' to be for
        // reading from writing earlier.

        this_frame->size = 0;
        ReadFrame(this_frame);
      }

      // Now lets cycle all our frame pointers to be ready for the
      // next round.
      auto* new_next = prev_frame ? prev_frame : &frame_item3_;
      prev_frame = this_frame;
      this_frame = next_frame;
      next_frame = new_next;

      // Sleep a bit to let everyone turn around their direction pins.
      ::usleep(15);
    }

    // Parse the final frame.
    if (prev_frame->frame.request_reply) {
      ParseFrame(prev_frame, reply);
    }

    // Now we can report success.
    boost::asio::post(
        executor_,
        std::bind(callback, mjlib::base::error_code()));
  }

  void ParseFrame(const FrameItem* frame_item, Reply* reply) {
    if (frame_item->size == 0) { return; }
    if (frame_item->size < 7) {
      malformed_++;
    }

    mjlib::base::BufferReadStream buffer_stream(
        {frame_item->encoded, frame_item->size});
    mjlib::multiplex::ReadStream<
      mjlib::base::BufferReadStream> stream{buffer_stream};

    stream.Read<uint16_t>();  // ignore header
    const auto maybe_source = stream.Read<uint8_t>();
    const auto maybe_dest = stream.Read<uint8_t>();
    (void) maybe_dest;
    const auto packet_size = stream.ReadVaruint();

    SingleReply this_reply;
    this_reply.id = *maybe_source;

    mjlib::base::BufferReadStream payload_stream{
      {buffer_stream.position(), *packet_size}};
    this_reply.reply = mjlib::multiplex::ParseRegisterReply(payload_stream);

    buffer_stream.ignore(*packet_size);
    const auto maybe_read_crc = stream.Read<uint16_t>();
    if (!maybe_read_crc) {
      malformed_++;
      return;
    }
    const auto read_crc = *maybe_read_crc;
    boost::crc_ccitt_type crc;
    crc.process_bytes(frame_item->encoded, frame_item->size - 2);
    const auto expected_crc = crc.checksum();

    if (read_crc != expected_crc) {
      checksum_errors_++;

      if (options_.debug_checksum_errors) {
        std::cout << "csum_error: "
                  << Hexify({frame_item->encoded, frame_item->size}) << "\n";
      }
      return;
    }

    reply->replies.push_back(std::move(this_reply));
  }

  struct Packet {
    const char* start = nullptr;
    std::size_t size = 0;

    Packet(const char* start_in, std::size_t size_in)
        : start(start_in),
          size(size_in) {}
  };

  std::optional<Packet> FindCompletePacket() {
    auto size = receive_pos_;
    auto* const packet = static_cast<const char*>(
        ::memmem(receive_buffer_, size,
                 &mjlib::multiplex::Format::kHeader, 2));
    if (packet == nullptr) { return {}; }

    const auto offset = packet - receive_buffer_;
    size -= offset;

    if (size < 7) {
      // This is below the minimum size.
      return {};
    }

    mjlib::base::BufferReadStream base_stream(
        {packet + 4, static_cast<std::size_t>(size - 4)});
    mjlib::multiplex::ReadStream<
      mjlib::base::BufferReadStream> stream(base_stream);

    const auto packet_size = stream.ReadVaruint();
    if (!packet_size) {
      return {};
    }
    if (*packet_size > (size - 6)) {
      return {};
    }

    // OK, we've got a full packet worth.
    return Packet{packet, static_cast<std::size_t>(
          4 + base_stream.offset() + *packet_size + 2)};
  }

  void ReadFrame(FrameItem* frame_item) {
    struct timespec ts = {};
    ::clock_gettime(CLOCK_MONOTONIC, &ts);
    const auto i64 = [](auto v) { return static_cast<int64_t>(v); };
    const int64_t now = i64(ts.tv_sec) * 1000000000ll + i64(ts.tv_nsec);
    const int64_t timeout = now + options_.query_timeout_s * 1000000000ll;

    auto read_until = [&](int expected) {
      while (true) {
        auto result = serial_->getc(timeout);
        if (result == -1) { return -1; }
        if (result < 0) { return result; }
        if (result == expected) { return result; }
      }
    };

    auto record_timeout = [&]() {
      frame_item->size = 0;
      timeouts_++;
    };

    while (true) {
      frame_item->size = 0;

      // Header byte 1
      {
        auto result = read_until(mjlib::multiplex::Format::kHeader & 0xff);
        if (result == -1) {
          record_timeout();
          return;
        }
        if (result < 0) {
          serial_errors_++;
          continue;
        }
        frame_item->encoded[frame_item->size++] = result;
      }

      // Header byte 2
      {
        auto result = serial_->getc(timeout);
        if (result == -1) {
          record_timeout();
          return;
        }
        if (result < 0) {
          malformed_++;
          continue;
        }
        if (result != (mjlib::multiplex::Format::kHeader >> 8)) {
          malformed_++;
          continue;
        }
        frame_item->encoded[frame_item->size++] = result;
      }

      // Address bytes and size.
      for (int i = 0; i < 3; i++) {
        auto result = serial_->getc(timeout);
        if (result == -1) {
          record_timeout();
          return;
        }
        if (result < 0) {
          serial_errors_++;
          continue;
        }
        frame_item->encoded[frame_item->size++] = result;
      }

      // We only support packets with up to 127 bytes payload for now.
      auto payload_size = frame_item->encoded[frame_item->size - 1];
      if (payload_size > 127) {
        malformed_++;
        continue;
      }

      // Now payload and checksum;
      for (int i = 0; i < (payload_size + 2); i++) {
        auto result = serial_->getc(timeout);
        if (result == -1) {
          record_timeout();
          return;
        }
        if (result < 0) {
          serial_errors_++;
          continue;
        }
        frame_item->encoded[frame_item->size++] = result;
      }

      return;
    }
  }

  void AssertThread() {
    MJ_ASSERT(std::this_thread::get_id() == thread_.get_id());
  }

  void AssertParent() {
    MJ_ASSERT(std::this_thread::get_id() != thread_.get_id());
  }

  boost::asio::executor executor_;
  const Options options_;

  std::thread thread_;

  // Accessed from both.
  std::promise<bool> startup_promise_;
  std::future<bool> startup_future_ = startup_promise_.get_future();

  std::atomic<uint64_t> checksum_errors_{0};
  std::atomic<uint64_t> timeouts_{0};
  std::atomic<uint64_t> malformed_{0};
  std::atomic<uint64_t> serial_errors_{0};
  std::atomic<uint64_t> extra_found_{0};

  // Only accessed from the child thread.
  boost::asio::io_context child_context_;
  std::optional<Rpi3RawUart> serial_;

  FrameItem frame_item1_;
  FrameItem frame_item2_;
  FrameItem frame_item3_;

  char receive_buffer_[256] = {};
  std::streamsize receive_pos_ = 0;
};

Rpi3ThreadedClient::Rpi3ThreadedClient(const boost::asio::executor& executor,
                                         const Options& options)
    : impl_(std::make_unique<Impl>(executor, options)) {}

Rpi3ThreadedClient::~Rpi3ThreadedClient() {}

void Rpi3ThreadedClient::AsyncRegister(const Request* request,
                                       Reply* reply,
                                       mjlib::io::ErrorCallback callback) {
  impl_->AsyncRegister(request, reply, callback);
}

Rpi3ThreadedClient::Stats Rpi3ThreadedClient::stats() const {
  return impl_->stats();
}

mjlib::io::SharedStream Rpi3ThreadedClient::MakeTunnel(
    uint8_t id,
    uint32_t channel,
    const TunnelOptions& options) {
  return impl_->MakeTunnel(id, channel, options);
}

}
}
