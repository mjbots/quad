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
#include "mjlib/multiplex/frame.h"
#include "mjlib/multiplex/stream.h"

#include "mech/rpi3_raw_uart.h"

namespace mjmech {
namespace mech {

namespace {
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
    mjlib::base::BufferReadStream buffer_stream(
        {frame_item->encoded, frame_item->size});
    mjlib::multiplex::ReadStream<
      mjlib::base::BufferReadStream> stream{buffer_stream};

    stream.Read<uint16_t>();  // ignore header
    const auto maybe_source = stream.Read<uint8_t>();
    const auto maybe_dest = stream.Read<uint8_t>();
    const auto packet_size = stream.ReadVaruint();

    if (!maybe_source || !maybe_dest || !packet_size) {
      malformed_++;
      return;
    }

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

}
}
