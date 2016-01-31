// Copyright 2015 Mikhail Afanasyev.  All rights reserved.
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

#include "mcast_video_link.h"

#include <mutex>
#include <thread>

#include <gst/gst.h>

#include <boost/format.hpp>
#include <boost/crc.hpp>

#include "base/common.h"
#include "base/context_full.h"
#include "base/deadline_timer.h"
#include "base/fail.h"
#include "base/now.h"

#include "base/logging.h"

namespace mjmech {
namespace mech {

namespace {
// A header is located at the beginnning of the packet
struct DataPacketHeader {
  const static uint32_t kMagic = 0x314C564D; // MVL1
  const static int kSize = 32;

  // Packet magic
  uint32_t magic;
  uint32_t reserved1;

  // Packet transmission time, microseconds since epocj
  uint64_t tx_time_us;

  // Video frame number
  uint32_t frame_num;
  // Most recent i-frame (two LSB bytes only)
  uint16_t iframe_num_lsb;
  // Start of video data (length of header + any aux informaton)
  uint16_t video_start;
  // Number of packets for this frame - 1
  uint8_t total_packets;
  // Index of this packet in this frame
  uint8_t packet_index;
  // Total number of this packet is sent
  uint8_t repeat_count;
  // Which copy of packet is it 0..(repeat_count - 1)
  uint8_t repeat_index;
  // crc32 of original video frame
  uint32_t frame_crc32;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(magic));
    a->Visit(MJ_NVP(reserved1));
    a->Visit(MJ_NVP(tx_time_us));
    a->Visit(MJ_NVP(frame_num));
    a->Visit(MJ_NVP(iframe_num_lsb));
    a->Visit(MJ_NVP(video_start));
    a->Visit(MJ_NVP(total_packets));
    a->Visit(MJ_NVP(packet_index));
    a->Visit(MJ_NVP(repeat_count));
    a->Visit(MJ_NVP(repeat_index));
    a->Visit(MJ_NVP(frame_crc32));
  }

  DataPacketHeader() {
    ::memset(this, 0, sizeof(*this));
  }
};

BOOST_STATIC_ASSERT(sizeof(DataPacketHeader) == DataPacketHeader::kSize);

struct DataPacketLog {
  boost::posix_time::ptime timestamp;
  int queue_len = -1;
  double interval_s = 0;
  DataPacketHeader header;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(timestamp));
    a->Visit(MJ_NVP(queue_len));
    a->Visit(MJ_NVP(interval_s));
    a->Visit(MJ_NVP(header));
  }
};

typedef boost::signals2::signal<void (const DataPacketLog*)> DataPacketSignal;

}


//
//                          TRANSMITTER
//

class McastVideoLinkTransmitter::Impl : boost::noncopyable {
 public:
  Impl(McastVideoLinkTransmitter* parent, boost::asio::io_service& service)
      : parent_(parent),
        service_(service),
        parameters_(parent->parameters_),
        main_id_(std::this_thread::get_id()),
        tx_timer_(service) {}

  ~Impl() {
  }

  void AsyncStart(base::ErrorHandler handler) {
    link_.reset(new base::UdpDataLink(
                    service_, log_, parameters_.link));
    if (parameters_.repeat_count == 0) {
      // Link is disabled
      log_.info("video_link tx disabled -- repeat_count is zero");
    }
    started_ = true;
  }

  void HandleVideoPacket(uint8_t* data, int len, bool key_frame) {
    if (!started_) {
      return;
    }
    if (parameters_.repeat_count == 0) {
      // Link is disabled
      return;
    }

    std::lock_guard<std::mutex> guard(queue_mutex_);

    if (key_frame) {
      last_iframe_num_ = total_frames_;
    }
    // Get number of packets and each individual packets' payload size.
    int max_data_size = link_->get_max_data_size() - DataPacketHeader::kSize;
    int num_packets = 1 + ((len - 1) / (max_data_size - 8));
    // tweak data_size to re-distribute data evenly across all packets
    int data_size = len / num_packets + 1;

    log_.debug("Got packet %d bytes, is_key=%d, split into %d packets %d"
               "(/%d) bytes long", len, key_frame, num_packets, data_size,
               max_data_size);

    BOOST_ASSERT(data_size <= max_data_size);
    BOOST_ASSERT(num_packets >= 1);
    BOOST_ASSERT(num_packets <= 0xFF);      // Wwe have 1 byte for # of packets.

    // Generic header fields.
    DataPacketHeader hdr;
    hdr.magic = DataPacketHeader::kMagic;
    hdr.frame_num = total_frames_;
    hdr.iframe_num_lsb = static_cast<uint16_t>(last_iframe_num_);
    BOOST_ASSERT(DataPacketHeader::kSize == sizeof(hdr));
    hdr.video_start = DataPacketHeader::kSize;
    hdr.total_packets = num_packets;
    hdr.repeat_count = parameters_.repeat_count;

    boost::crc_32_type crc_calc; // pkzip/ethernet
    crc_calc.process_bytes(data, len);
    hdr.frame_crc32 = crc_calc.checksum();

    // Produce and enqueue packets.
    for (int copy=0; copy < parameters_.repeat_count; copy++) {
      hdr.repeat_index = copy;
      int offset = 0;
      for (int index=0; index < num_packets; index++) {
        hdr.packet_index = index;
        int this_data_size = std::min(data_size, len - offset);
        BOOST_ASSERT(this_data_size > 0);
        to_tx_.emplace_back(
            reinterpret_cast<const char*>(&hdr), sizeof(hdr));
        to_tx_.back().resize(hdr.video_start + this_data_size);
        ::memcpy(&to_tx_.back().front() + hdr.video_start,
                 reinterpret_cast<char*>(data) + offset,
                 this_data_size);
        offset += this_data_size;
      }
      BOOST_ASSERT(offset == len);
    }

    total_frames_++;
    double tx_time_s = 1.0 / parameters_.min_fps;
    // TODO theamk: add framerate estimation, and decrease tx_time_s as
    // needed.
    tx_deadline_ = base::Now(service_) +
                   base::ConvertSecondsToDuration(tx_time_s);
    if (!tx_running_) {
      tx_running_ = true;
      service_.post(
          std::bind(&Impl::TxNextPacket, this, boost::system::error_code()));
    }
  }

  DataPacketSignal* packet_signal() { return &packet_signal_; }

 private:
  void TxNextPacket(boost::system::error_code ec) {
    base::FailIf(ec);
    std::lock_guard<std::mutex> guard(queue_mutex_);

    BOOST_ASSERT(tx_running_);
    BOOST_ASSERT(!to_tx_.empty());

    DataPacketHeader* header = reinterpret_cast<DataPacketHeader*>(
        &to_tx_.front().front());;
    boost::posix_time::ptime now = base::Now(service_);
    header->tx_time_us = base::ConvertPtimeToMicroseconds(now);

    DataPacketLog dpl;
    // Transmit packet
    dpl.timestamp = now;
    dpl.header = *header;
    link_->Send(to_tx_.front());
    to_tx_.pop_front();

    //log_.debug("TX packet, %d in queue", to_tx_.size());

    // Schedule next packet if needed.
    dpl.queue_len = to_tx_.size();

    double interval_s = 0;
    if (to_tx_.empty()) {
      tx_running_ = false;
    } else {
      // schedule next packet.
      double time_left = base::ConvertDurationToSeconds(
          tx_deadline_ - dpl.timestamp);
      interval_s = std::max(0.0001, time_left / to_tx_.size());
      tx_timer_.expires_from_now(base::ConvertSecondsToDuration(interval_s));
      tx_timer_.async_wait(std::bind(&Impl::TxNextPacket, this,
                                     std::placeholders::_1));
    }

    dpl.interval_s = interval_s;
    packet_signal_(&dpl);
  }

  // Variables which are only set in constructor or in Start()
  DataPacketSignal packet_signal_;
  McastVideoLinkTransmitter* const parent_;
  boost::asio::io_service& service_;
  const Parameters& parameters_;
  std::thread::id main_id_;
  bool started_ = false;
  base::LogRef log_ = base::GetLogInstance("mcast_video_link");
  std::unique_ptr<base::UdpDataLink> link_;
  std::mutex queue_mutex_;

  // Queue-related variables. Protected by queue_mutex_.
  uint32_t total_frames_ = 0;
  uint32_t last_iframe_num_ = 0;
  std::deque<std::string> to_tx_;
  // All packets must be sent by this time.
  boost::posix_time::ptime tx_deadline_;
  base::DeadlineTimer tx_timer_;
  bool tx_running_ = false;
};


class McastVideoLinkTransmitter::FrameConsumerImpl :
      public CameraFrameConsumer {
 public:
  FrameConsumerImpl(std::shared_ptr<McastVideoLinkTransmitter::Impl> impl)
      : impl_(impl) {};

  // CameraFrameConsumer interface
  virtual void ConsumeH264Sample(GstSample* sample) {
    GstBuffer* buf = gst_sample_get_buffer(sample);
    size_t len = gst_buffer_get_size(buf);
    bool key_frame = !GST_BUFFER_FLAG_IS_SET(buf, GST_BUFFER_FLAG_DELTA_UNIT);
    GstMemory* mem = gst_buffer_get_all_memory(buf);
    GstMapInfo info;
    memset(&info, 0, sizeof(info));
    bool ok = gst_memory_map(mem, &info, GST_MAP_READ);
    BOOST_ASSERT(ok);
    BOOST_ASSERT(info.size == len);

    impl_->HandleVideoPacket(info.data, info.size, key_frame);

    gst_memory_unmap(mem, &info);
    gst_memory_unref(mem);
  }

  virtual void PreEmitStats(CameraDriver::CameraStats* stats) {
    //impl_->PreEmitStats(stats);
  }

 private:
  std::shared_ptr<McastVideoLinkTransmitter::Impl> impl_;
};

McastVideoLinkTransmitter::McastVideoLinkTransmitter(base::Context& context)
    : impl_(new Impl(this, context.service)),
      frame_consumer_impl_(new FrameConsumerImpl(impl_)) {
  context.telemetry_registry->Register("mvl_data_tx", impl_->packet_signal());
}

McastVideoLinkTransmitter::~McastVideoLinkTransmitter() {}

void McastVideoLinkTransmitter::AsyncStart(base::ErrorHandler handler) {
  impl_->AsyncStart(handler);
}

std::weak_ptr<CameraFrameConsumer
              > McastVideoLinkTransmitter::get_frame_consumer() {
  return frame_consumer_impl_;
}


//
//                          RECEIVER
//

class McastVideoLinkReceiver::Impl : boost::noncopyable {
 public:
  Impl(McastVideoLinkReceiver* parent, boost::asio::io_service& service)
      : parent_(parent),
        service_(service),
        parameters_(parent->parameters_),
        main_id_(std::this_thread::get_id()) {}

  ~Impl() {
  }

  void AsyncStart(base::ErrorHandler handler) {
    started_ = true;
  }

  DataPacketSignal* packet_signal() { return &packet_signal_; }

 private:
  DataPacketSignal packet_signal_;
  McastVideoLinkReceiver* const parent_;
  boost::asio::io_service& service_;
  const Parameters& parameters_;
  std::thread::id main_id_;
  bool started_ = false;
  base::LogRef log_ = base::GetLogInstance("mcast_video_link");
};


McastVideoLinkReceiver::McastVideoLinkReceiver(base::Context& context)
    : impl_(new Impl(this, context.service)) {
  context.telemetry_registry->Register("mvl_data_rx", impl_->packet_signal());
}

McastVideoLinkReceiver::~McastVideoLinkReceiver() {}

void McastVideoLinkReceiver::AsyncStart(base::ErrorHandler handler) {
  impl_->AsyncStart(handler);
}


}
}
