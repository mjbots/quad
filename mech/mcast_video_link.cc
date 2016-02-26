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
#include <bitset>
#include <unordered_map>
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
  const static int kSize = 40;

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
  // crc32 of this packet only (payload only)
  uint32_t packet_crc32;
  uint32_t reserved2;

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
    a->Visit(MJ_NVP(packet_crc32));
    a->Visit(MJ_NVP(reserved2));
  }

  DataPacketHeader() {
    ::memset(this, 0, sizeof(*this));
  }
};

BOOST_STATIC_ASSERT(sizeof(DataPacketHeader) == DataPacketHeader::kSize);


class DataPacket : boost::noncopyable {
 public:
  DataPacket() {};

  // Create new packet
  DataPacket(const DataPacketHeader& hdr,
             const char* aux_data, int aux_len,
             const char* video_data, int video_len)
      : data_(hdr.video_start + video_len, 0) {
    BOOST_ASSERT(video_len >= 0);
    BOOST_ASSERT(hdr.video_start >= sizeof(hdr));
    BOOST_ASSERT(aux_len == (hdr.video_start - DataPacketHeader::kSize));
    ::memcpy(&data_[0], &hdr, sizeof(hdr));
    if (aux_len) {
      ::memcpy(&data_[DataPacketHeader::kSize], aux_data, aux_len);
    }
    ::memcpy(&data_[hdr.video_start], video_data, video_len);

    boost::crc_32_type crc_calc;
    crc_calc.process_bytes(video_data, video_len);
    header_nv().packet_crc32 = crc_calc.checksum();

    CheckValid();
  }

  // Parse from bytes
  DataPacket(const std::string& data)
      : data_(data) {
    CheckValid();
  }

  // Accessors
  bool valid() const { return valid_; };

  // Mutable header
  DataPacketHeader& header() {
    BOOST_ASSERT(valid_);
    return header_nv();
  }

  const DataPacketHeader& header() const {
    BOOST_ASSERT(valid_);
    return header_nv();
  }

  const std::string& data() {
    BOOST_ASSERT(valid_);
    return data_;
  }

  const char* aux() const {
    return &data_[DataPacketHeader::kSize];
  }

  int aux_len() const {
    BOOST_ASSERT(valid_);
    return header().video_start - DataPacketHeader::kSize;
  }

  const char* payload() const {
    return &data_[header().video_start];
  }

  int payload_len() const {
    BOOST_ASSERT(valid_);
    return data_.size() - header().video_start;
  }

 private:
  DataPacketHeader& header_nv() {
    return *reinterpret_cast<DataPacketHeader*>(&data_.front());
  }

  const DataPacketHeader& header_nv() const {
    return *reinterpret_cast<const DataPacketHeader*>(&data_.front());
  }

  void CheckValid() {
    if (data_.size() <= DataPacketHeader::kSize) {
      valid_ = false;
      return;
    }
    const DataPacketHeader& dph = header_nv();
    valid_ = (dph.magic == DataPacketHeader::kMagic &&
              dph.video_start >= sizeof(DataPacketHeader) &&
              dph.video_start < data_.size() &&
              dph.total_packets > 0 &&
              dph.packet_index < dph.total_packets &&
              dph.repeat_count > 0 &&
              dph.repeat_index < dph.repeat_count);
  }

  bool valid_ = false;
  std::string data_;
};

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

  void HandleVideoPacket(uint8_t* data, int video_len, bool key_frame) {
    if (!started_) {
      return;
    }
    if (parameters_.repeat_count == 0) {
      // Link is disabled
      return;
    }

    std::lock_guard<std::mutex> guard(queue_mutex_);

    ExpireTelemetry();
    const std::string telemetry = MakeTelemetryData();
    const int len = video_len + telemetry.size();

    if (key_frame) {
      last_iframe_num_ = total_frames_;
    }
    // Get number of packets and each individual packets' payload size.
    int max_data_size = link_->get_max_data_size() - DataPacketHeader::kSize;
    int num_packets = 1 + ((len - 1) / (max_data_size - 8));
    // tweak data_size to re-distribute data evenly across all packets
    int data_size = len / num_packets + 1;

    log_.debug("TXing frame %d: %d bytes, is_key=%d, split into %d packets %d"
               "(/%d) bytes long",
               total_frames_, len, key_frame, num_packets, data_size,
               max_data_size);

    BOOST_ASSERT(data_size <= max_data_size);
    BOOST_ASSERT(num_packets >= 1);
    BOOST_ASSERT(num_packets <= 0xFF);      // We have 1 byte for # of packets.

    // Generic header fields.
    DataPacketHeader hdr;
    hdr.magic = DataPacketHeader::kMagic;
    hdr.frame_num = total_frames_;
    hdr.iframe_num_lsb = static_cast<uint16_t>(last_iframe_num_);
    BOOST_ASSERT(DataPacketHeader::kSize == sizeof(hdr));
    hdr.total_packets = num_packets;
    hdr.repeat_count = parameters_.repeat_count;
    boost::crc_32_type crc_calc; // pkzip/ethernet
    crc_calc.process_bytes(data, video_len);
    hdr.frame_crc32 = crc_calc.checksum();

    // Produce and enqueue packets.
    for (int copy = 0; copy < parameters_.repeat_count; copy++) {
      hdr.repeat_index = copy;
      int telemetry_offset = 0;
      int video_offset = 0;

      for (int index=0; index < num_packets; index++) {
        const int telemetry_remaining = telemetry.size() - telemetry_offset;
        const int video_remaining = video_len - video_offset;
        const int total_remaining = telemetry_remaining + video_remaining;
        const int this_data_size = std::min(data_size, total_remaining);

        const int telemetry_to_send =
            std::min(this_data_size, telemetry_remaining);
        const int video_to_send =
            std::min(this_data_size - telemetry_to_send, video_remaining);

        hdr.video_start = DataPacketHeader::kSize + telemetry_to_send;
        hdr.packet_index = index;
        BOOST_ASSERT(this_data_size > 0);

        to_tx_.emplace_back(
            hdr,
            telemetry.data() + telemetry_offset, telemetry_to_send,
            reinterpret_cast<char*>(data) + video_offset, video_to_send);
        BOOST_ASSERT(to_tx_.back().valid());
        telemetry_offset += telemetry_to_send;
        video_offset += video_to_send;
      }
      BOOST_ASSERT(telemetry_offset == static_cast<int>(telemetry.size()));
      BOOST_ASSERT(video_offset == video_len);
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

  void SetTelemetry(const std::string& name, const std::string& data,
                    boost::posix_time::ptime expiration) {
    std::lock_guard<std::mutex> guard(queue_mutex_);

    telemetry_[name] = Telemetry{data, expiration};
  }

  DataPacketSignal* packet_signal() { return &packet_signal_; }

 private:
  void TxNextPacket(boost::system::error_code ec) {
    base::FailIf(ec);
    BOOST_ASSERT(std::this_thread::get_id() == main_id_);
    std::lock_guard<std::mutex> guard(queue_mutex_);

    BOOST_ASSERT(tx_running_);
    BOOST_ASSERT(!to_tx_.empty());

    boost::posix_time::ptime now = base::Now(service_);
    to_tx_.front().header().tx_time_us = base::ConvertPtimeToMicroseconds(now);

    DataPacketLog dpl;
    // Transmit packet
    dpl.timestamp = now;
    dpl.header = to_tx_.front().header();
    link_->Send(to_tx_.front().data());
    to_tx_.pop_front();

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

  // NOTE: This requires that the queue_mutex_ lock be held.
  void ExpireTelemetry() {
    const auto now = base::Now(service_);

    for (auto it = telemetry_.begin(); it != telemetry_.end();) {
      auto next = std::next(it);

      if (now > it->second.expiration) {
        telemetry_.erase(it);
      }

      it = next;
    }
  }

  // NOTE: This requires that the queue_mutex_ lock be held.
  std::string MakeTelemetryData() const {
    base::FastOStringStream ostr;
    base::TelemetryWriteStream<base::FastOStringStream> os(ostr);

    for (const auto& pair: telemetry_) {
      os.Write(pair.first); // name
      os.Write(pair.second.data); // data
    }

    return ostr.str();
  }

  // Variables which are only set in constructor or in Start()
  DataPacketSignal packet_signal_;
  McastVideoLinkTransmitter* const parent_;
  boost::asio::io_service& service_;
  const Parameters& parameters_;
  std::thread::id main_id_;
  bool started_ = false;
  base::LogRef log_ = base::GetLogInstance("mcast_video_tx");
  std::unique_ptr<base::UdpDataLink> link_;
  std::mutex queue_mutex_;

  // Queue-related variables. Protected by queue_mutex_.
  uint32_t total_frames_ = 0;
  uint32_t last_iframe_num_ = 0;
  std::deque<DataPacket> to_tx_;
  // All packets must be sent by this time.
  boost::posix_time::ptime tx_deadline_;
  base::DeadlineTimer tx_timer_;
  bool tx_running_ = false;

  struct Telemetry {
    std::string data;
    boost::posix_time::ptime expiration;
  };

  std::map<std::string, Telemetry> telemetry_;
};


class McastVideoLinkTransmitter::FrameConsumerImpl :
      public CameraFrameConsumer {
 public:
  FrameConsumerImpl(std::shared_ptr<McastVideoLinkTransmitter::Impl> impl)
      : impl_(impl) {}
  virtual ~FrameConsumerImpl() {}

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

class McastVideoLinkTransmitter::TelemetryImpl :
      public McastTelemetryInterface {
 public:
  TelemetryImpl(std::shared_ptr<McastVideoLinkTransmitter::Impl> impl)
      : impl_(impl) {}

  virtual ~TelemetryImpl() {}

  void SetTelemetry(const std::string& name,
                    const std::string& data,
                    boost::posix_time::ptime expiration) override {
    impl_->SetTelemetry(name, data, expiration);
  }

 private:
  std::shared_ptr<McastVideoLinkTransmitter::Impl> impl_;
};

McastVideoLinkTransmitter::McastVideoLinkTransmitter(base::Context& context)
    : impl_(new Impl(this, context.service)),
      frame_consumer_impl_(new FrameConsumerImpl(impl_)),
      telemetry_impl_(new TelemetryImpl(impl_)) {
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

std::weak_ptr<McastTelemetryInterface
              > McastVideoLinkTransmitter::get_telemetry_interface() {
  return telemetry_impl_;
}


//
//                          RECEIVER
//
namespace {

struct ReceivedFrameInfo {
  // time when the last packet for the frame is received
  // (this may be after the frame has been passed to a decoder)
  boost::posix_time::ptime timestamp;
  // time from the first packet to the time it was passed to a decoder
  // (or to a last packet if the frame was never complete)
  double rx_latency_s = 0;

  // number for this frame and for the latest i-frame
  uint32_t frame_num = 0;
  uint32_t iframe_num = 0;
  uint32_t frame_crc32 = 0;

  // data packets sent (counting all duplicates only once)
  int total_packets = 0;
  // data packets lost -- if this is non-zero, frame is not passed to decoder.
  int lost_packets = 0;

  // repeat count for this frame
  int repeat_count = 0;
  // fraction of packets received (0..1)
  double rx_rate = 0;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(timestamp));
    a->Visit(MJ_NVP(rx_latency_s));

    a->Visit(MJ_NVP(frame_num));
    a->Visit(MJ_NVP(iframe_num));
    a->Visit(MJ_NVP(frame_crc32));

    a->Visit(MJ_NVP(total_packets));
    a->Visit(MJ_NVP(lost_packets));

    a->Visit(MJ_NVP(repeat_count));
    a->Visit(MJ_NVP(rx_rate));
  }
};
typedef boost::signals2::signal<void (const ReceivedFrameInfo*)
                                > ReceivedFrameInfoSignal;

class RxFrameBuffer : boost::noncopyable {
 public:
  RxFrameBuffer(const DataPacketHeader& header) {
    info_.frame_num = header.frame_num;
    // TODO theamk recover iframe_num
    info_.iframe_num = 0;
    info_.total_packets = header.total_packets;
    info_.repeat_count = header.repeat_count;
    info_.frame_crc32 = header.frame_crc32;

    BOOST_ASSERT(info_.total_packets <= static_cast<int>(missing_.size()));
    packets_.resize(info_.total_packets);
    for (int i=0; i<info_.total_packets; i++) {
      missing_.set(i);
    }
    info_.lost_packets = missing_.count();
  }

  // Is the frame complete (do we have all packets?)
  bool complete() const { return info_.lost_packets == 0; };

  // ReceivedFrameInfo structure, suitable for logging.
  const ReceivedFrameInfo* info() {
    return &info_;
  }

  struct FrameResult {
    std::shared_ptr<std::string> aux;
    std::shared_ptr<std::string> video;
  };

  FrameResult AssembleFrame(base::LogRef& log) {
    BOOST_ASSERT(missing_.none());
    int aux_len = 0;
    int video_len = 0;
    for (auto& packet: packets_) {
      aux_len += packet->aux_len();
      video_len += packet->payload_len();
    }

    FrameResult result;
    result.aux.reset(new std::string(aux_len, 0));
    result.video.reset(new std::string(video_len, 0));

    int aux_pos = 0;
    int video_pos = 0;
    int idx = 0;
    for (auto& packet: packets_) {
      BOOST_ASSERT(packet->header().packet_index == idx);
      idx++;
      ::memcpy(&(*result.aux)[aux_pos],
               packet->aux(),
               packet->aux_len());
      ::memcpy(&(*result.video)[video_pos],
               packet->payload(),
               packet->payload_len());
      aux_pos += packet->aux_len();
      video_pos += packet->payload_len();
    }
    BOOST_ASSERT(aux_pos == aux_len);
    BOOST_ASSERT(video_pos == video_len);

    boost::crc_32_type crc32;
    crc32.process_bytes(&(*result.video)[0], video_len);
    uint32_t real = crc32.checksum();
    if (real != info_.frame_crc32) {
      log.warn("CRC32 mismatch for frame %d (%d bytes): 0x%08X != 0x%08X",
               info_.frame_num, video_len, real, info_.frame_crc32);
    }
    return result;
  }

  // Handle the packet. it will be moved 'in' if it is interesting.
  void HandlePacket(const boost::posix_time::ptime now,
                    std::unique_ptr<DataPacket>* packet) {
    const DataPacketHeader& header = (*packet)->header();
    BOOST_ASSERT(info_.frame_num == header.frame_num);
    BOOST_ASSERT(info_.total_packets == header.total_packets);
    BOOST_ASSERT(info_.repeat_count == header.repeat_count);
    BOOST_ASSERT(info_.frame_crc32 == header.frame_crc32);

    info_.timestamp = now;
    if (first_rx_time_.is_not_a_date_time()) {
      first_rx_time_ = now;
    }
    if (!complete()) {
      info_.rx_latency_s = base::ConvertDurationToSeconds(
          now - first_rx_time_);
    }

    if (missing_[header.packet_index]) {
      // This is a new packet. keep it.
      BOOST_ASSERT(missing_.test(header.packet_index));
      missing_.reset(header.packet_index);
      info_.lost_packets = missing_.count();
      packets_[header.packet_index].swap(*packet);
    }
    rx_count_ += 1;
    info_.rx_rate = rx_count_ * 1.0 / (
        info_.total_packets * info_.repeat_count);
  }
 private:
  ReceivedFrameInfo info_;
  boost::posix_time::ptime first_rx_time_ = boost::posix_time::not_a_date_time;
  std::vector<std::unique_ptr<DataPacket>> packets_;
  std::bitset<64> missing_;
  int rx_count_ = 0;
};

};


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
    link_.reset(new base::UdpDataLink(
                    service_, log_, parameters_.link));
    link_->data_signal()->connect(
        std::bind(&Impl::HandlePacket, this, std::placeholders::_1,
                  std::placeholders::_2));
    started_ = true;
  }

  DataPacketSignal* packet_signal() { return &packet_signal_; }
  ReceivedFrameInfoSignal* frame_info_signal() { return &frame_info_signal_; }

 private:
  void ExpireOldFrames() {
    // Remove oldest frames
    while (pending_deque_.size() > 3) {
      int id = pending_deque_.front();
      auto it = pending_.find(id);
      BOOST_ASSERT(it != pending_.end());
      const auto& info = it->second->info();
      frame_info_signal_(info);
      if (!it->second->complete()) {
        log_.info("packet %d lost (%d/%d, %.0f%% recvd)",
                  info->frame_num, info->total_packets, info->lost_packets,
                  info->rx_rate * 100.0);
      }
      pending_deque_.pop_front();
      pending_.erase(it);
    }
  }

  void HandlePacket(const std::string& data,
                    const base::UdpDataLink::PeerInfo& peer) {
    std::unique_ptr<DataPacket> packet(new DataPacket(data));
    if (!packet->valid()) {
      log_.warn("got invalid packet (%d bytes) from %d",
                data.size(), peer.id);
      return;
    }

    const DataPacketHeader* header = &packet->header();
    if (0) {
      log_.debug("got %d bytes from %d (frame %d, pkt %d/%d, rep %d/%d)",
                 data.size(), peer.id, header->frame_num,
                 header->packet_index, header->total_packets,
                 header->repeat_index, header->repeat_count);
    }
    boost::posix_time::ptime now = base::Now(service_);

    // Log the packet before RxFrameBuffer graphs it.
    DataPacketLog dpl;
    dpl.timestamp = now;
    dpl.header = *header;
    dpl.queue_len = 0;
    if (!last_rx_time_.is_not_a_date_time()) {
      dpl.interval_s = base::ConvertDurationToSeconds(now - last_rx_time_);
    }
    last_rx_time_ = now;
    packet_signal_(&dpl);

    // Verify packet CRC32
    boost::crc_32_type crc_calc;
    crc_calc.process_bytes(packet->payload(), packet->payload_len());
    if (header->packet_crc32 != crc_calc.checksum()) {
      log_.warn("packet %d.%d.%d has bad checksum",
                header->frame_num, header->packet_index,
                header->repeat_index);
    }

    // Find RxFrameBuffer or construct new if not found.
    auto p_it = pending_.find(header->frame_num);
    RxFrameBuffer* buff = nullptr;
    if (p_it == pending_.end()) {
      // Create a new frame buffer
      buff = new RxFrameBuffer(*header);
      pending_[header->frame_num] = std::unique_ptr<RxFrameBuffer>(buff);
      pending_deque_.push_back(header->frame_num);
    } else {
      buff = p_it->second.get();
    }
    bool was_complete = buff->complete();

    buff->HandlePacket(now, &packet);
    // 'packet' is invalid after this point.

    if (buff->complete() && !was_complete) {
      // packet received. pass to decoder
      auto ready_frame = buff->AssembleFrame(log_);
      log_.debug("RXed frame %d: %d bytes, from %d packets",
                 buff->info()->frame_num, ready_frame.video->size(),
                 buff->info()->total_packets);
      (*parent_->frame_ready_signal())(ready_frame.video);
      DoAux(*ready_frame.aux);
    }
    ExpireOldFrames();
  }

  void DoAux(const std::string& aux) {
    base::FastIStringStream istr(aux);
    base::TelemetryReadStream<base::FastIStringStream> is(istr);

    try {
      while (istr.remaining()) {
        const std::string name = is.ReadString();
        const std::string data = is.ReadString();
        (*parent_->telemetry_ready_signal())(name, data);
      }
    } catch (base::SystemError& se) {
      log_.warn("corrupt aux data: " + se.error_code().message());
    }
  }

  DataPacketSignal packet_signal_;
  ReceivedFrameInfoSignal frame_info_signal_;
  McastVideoLinkReceiver* const parent_;
  boost::asio::io_service& service_;
  const Parameters& parameters_;
  std::thread::id main_id_;
  bool started_ = false;
  base::LogRef log_ = base::GetLogInstance("mcast_video_rx");
  std::unique_ptr<base::UdpDataLink> link_;

  boost::posix_time::ptime last_rx_time_ = boost::posix_time::not_a_date_time;
  std::unordered_map<uint32_t, std::unique_ptr<RxFrameBuffer>> pending_;
  std::deque<uint32_t> pending_deque_;
};


McastVideoLinkReceiver::McastVideoLinkReceiver(base::Context& context)
    : impl_(new Impl(this, context.service)) {
  context.telemetry_registry->Register("mvl_data_rx", impl_->packet_signal());
  context.telemetry_registry->Register(
      "mvl_frame_rx", impl_->frame_info_signal());
}

McastVideoLinkReceiver::~McastVideoLinkReceiver() {}

void McastVideoLinkReceiver::AsyncStart(base::ErrorHandler handler) {
  impl_->AsyncStart(handler);
}


}
}
