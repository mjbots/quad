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

#include "base/common.h"
#include "base/context_full.h"
#include "base/fail.h"
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
  // Retransmit counter. 0 if each packet is sent exactly once,
  // else upper 4 bits is (total # of repeats - 1), lower
  // 4 bits is (# of this repeat)
  uint8_t repeat_info;
  uint8_t reserved2;
  uint32_t reserved3;

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
    a->Visit(MJ_NVP(repeat_info));
    a->Visit(MJ_NVP(reserved2));
    a->Visit(MJ_NVP(reserved3));
  }

  DataPacketHeader() {
    ::memset(this, 0, sizeof(*this));
  }
};

BOOST_STATIC_ASSERT(sizeof(DataPacketHeader) == DataPacketHeader::kSize);

struct DataPacketLog {
  boost::posix_time::ptime timestamp;
  int peer_id;
  DataPacketHeader header;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(timestamp));
    a->Visit(MJ_NVP(peer_id));
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
        main_id_(std::this_thread::get_id()) {}

  ~Impl() {
  }

  void AsyncStart(base::ErrorHandler handler) {
    started_ = true;
  }

  void HandleVideoPacket(uint8_t* data, int len, bool key_frame) {
    if (!started_) {
      return;
    }
    BOOST_ASSERT(false); // not implemented
  }

  DataPacketSignal* packet_signal() { return &packet_signal_; }

 private:
  DataPacketSignal packet_signal_;
  McastVideoLinkTransmitter* const parent_;
  boost::asio::io_service& service_;
  const Parameters& parameters_;
  std::thread::id main_id_;
  bool started_ = false;
  base::LogRef log_ = base::GetLogInstance("mcast_video_link");
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
