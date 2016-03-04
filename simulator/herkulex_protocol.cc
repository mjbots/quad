// Copyright 2015 Josh Pieper, jjp@pobox.com.  All rights reserved.
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

#include "herkulex_protocol.h"

#include <boost/asio/read.hpp>
#include <boost/asio/write.hpp>

namespace {
const auto u8 = [](char c) { return static_cast<uint8_t>(c); };

enum {
  kBroadcast = 0xfe,
};

enum {
  kErrorInvalidPacket = 0x08,
};

enum {
  kDetailChecksumError = 0x04,
  kDetailUnknownCommand = 0x08,
  kDetailGarbageDetected = 0x20,
  kDetailMotorOn = 0x40,
};

enum {
  kPacketSize = 2,
  kPacketID = 3,
  kPacketCmd = 4,
  kPacketCs1 = 5,
  kPacketCs2 = 6,
  kPacketHeaderSize = 7,
  kPacketAddress = 7,
  kPacketDataLength = 8,
  kPacketData = 9,
  kPacketSJogTime = 7,
  kPacketSJogData = 8,
};

uint8_t CalculateCs1(const char* buf, uint8_t len) {
  if (len < 7) { return 0; }

  const uint8_t size = u8(buf[kPacketSize]);
  const uint8_t id = u8(buf[kPacketID]);
  const uint8_t cmd = u8(buf[kPacketCmd]);

  uint8_t expected_cs1 = size ^ id ^ cmd;
  for (uint8_t i = 7; i < len; i++) {
    expected_cs1 ^= u8(buf[i]);
  }

  expected_cs1 &= 0xfe;

  return expected_cs1;
}
}

namespace mjmech {
namespace simulator {

class HerkulexProtocol::Impl {
 public:
  Impl(base::AsyncStream& stream, Operations& operations)
      : stream_(stream), operations_(operations) {}

  void StartReadHeader() {
    boost::asio::async_read(
        stream_, boost::asio::buffer(buffer_, 1),
        [this](base::ErrorCode error, std::size_t) {
          this->HandleReadHeader1(error);
        });
  }

  void HandleReadHeader1(base::ErrorCode error) {
    if (error) { HandleError(error); return; }

    if (u8(buffer_[0]) != 0xff) { StartReadHeader(); return; }

    boost::asio::async_read(
        stream_, boost::asio::buffer(buffer_ + 1, 1),
        [this](base::ErrorCode error, std::size_t) {
          this->HandleReadHeader2(error);
        });
  }

  void HandleReadHeader2(base::ErrorCode error) {
    if (error) { HandleError(error); return; }
    if (u8(buffer_[1]) != 0xff) { StartReadHeader(); return; }

    boost::asio::async_read(
        stream_, boost::asio::buffer(buffer_ + 2, 5),
        [this](base::ErrorCode error, std::size_t) {
          this->HandleReadHeader3(error);
        });
  }

  void HandleReadHeader3(base::ErrorCode error) {
    if (error) { HandleError(error); return; }

    const uint8_t packet_size = u8(buffer_[kPacketSize]);

    if (packet_size < 7) {
      status_error_ = kErrorInvalidPacket;
      status_detail_ = kDetailGarbageDetected;
      HandleError(base::ErrorCode::einval("invalid packet"));
      return;
    }

    boost::asio::async_read(
        stream_, boost::asio::buffer(buffer_ + 7, packet_size - 7),
        [this](base::ErrorCode error, std::size_t) {
          this->HandleReadData(error);
        });
  }

  void HandleReadData(base::ErrorCode error) {
    if (error) { HandleError(error); return; }

    ProcessPacket();

    StartReadHeader();
  }

  void HandleError(base::ErrorCode error) {
    // TODO jpieper: Log this in some way.
    StartReadHeader();
  }

  bool VerifyChecksum() const {
    const uint8_t cs1 = u8(buffer_[kPacketCs1]);
    const uint8_t cs2 = u8(buffer_[kPacketCs2]);

    const uint8_t size = u8(buffer_[kPacketSize]);

    const uint8_t expected_cs1 = CalculateCs1(buffer_, size);
    const uint8_t expected_cs2 = (expected_cs1 ^ 0xff) & 0xfe;

    if (cs1 != expected_cs1 ||
        cs2 != expected_cs2) {
      return false;
    }

    return true;
  }

  void ProcessPacket() {
    if (!VerifyChecksum()) {
      status_error_ |= kErrorInvalidPacket;
      status_detail_ |= kDetailChecksumError;
      return;
    }

    const uint8_t id = u8(buffer_[kPacketID]);

    if (!operations_.address_valid(id)) {
      // They aren't talking to us.
      return;
    }

    const uint8_t cmd = u8(buffer_[kPacketCmd]);

    if (cmd == 0x09) { // REBOOT
      operations_.Reboot(id);
    } else if (cmd == 0x07) {
      // For now, we'll handle replying to all STAT messages ourselves.
      HandleStat(id);
    } else if (cmd == 0x03) {
      HandleRamWrite(id);
    } else if (cmd == 0x04) {
      HandleRamRead(id);
    } else if (cmd == 0x06) {
      HandleSJog();
    } else if (cmd == 0x05 || // I_JOG
               cmd == 0x08) { // ROLLBACK
      // silently ignore
    } else {
      status_error_ |= kErrorInvalidPacket;
      status_detail_ |= kDetailChecksumError;
    }
  }

  void HandleStat(uint8_t servo) {
    if (write_outstanding_) { return; }

    tx_buffer_[kPacketData] = GetStatusError(servo);
    tx_buffer_[kPacketData + 1] = GetStatusDetail(servo);

    SendData(servo, 0x47, 2); // STAT_ACK
  }

  void HandleRamWrite(int servo) {
    const uint8_t size = u8(buffer_[kPacketSize]);
    if (size < 9) {
      status_error_ = kErrorInvalidPacket;
      status_detail_ = kDetailGarbageDetected;
      return;
    }

    const uint8_t addr = u8(buffer_[kPacketAddress]);
    const uint8_t len = u8(buffer_[kPacketDataLength]);

    if (len + 9 > size) {
      status_error_ = kErrorInvalidPacket;
      status_detail_ = kDetailGarbageDetected;
      return;
    }

    for (uint8_t i = 0; i < len; i++) {
      operations_.WriteRam(servo, addr + i, u8(buffer_[kPacketData + i]));
    }
  }

  void HandleRamRead(int servo) {
    if (write_outstanding_) { return; }

    const uint8_t size = u8(buffer_[kPacketSize]);
    if (size < 9) {
      status_error_ = kErrorInvalidPacket;
      status_detail_ = kDetailGarbageDetected;
      return;
    }

    const uint8_t address = u8(buffer_[kPacketAddress]);
    const uint8_t len = u8(buffer_[kPacketDataLength]);

    tx_buffer_[kPacketAddress] = address;
    tx_buffer_[kPacketDataLength] = len;
    for (uint8_t i = 0; i < len; i++) {
      tx_buffer_[kPacketData + i] = operations_.ReadRam(servo, address + i);
    }
    tx_buffer_[kPacketData + 0 + len] = GetStatusError(servo);
    tx_buffer_[kPacketData + 1 + len] = GetStatusDetail(servo);

    SendData(servo, 0x44, len + 4);
  }

  void HandleSJog() {
    // TODO jpieper: For now, we don't expose this.
    //const uint8_t int_time = u8(buffer_[kPacketSJogTime]);
    BOOST_ASSERT(u8(buffer_[kPacketSize]) >= kPacketHeaderSize);
    const uint8_t len = u8(buffer_[kPacketSize]) - kPacketHeaderSize;
    std::vector<Operations::ServoAngle> angles;
    for (int i = 0; (i + 3) < len; i+= 4) {
      Operations::ServoAngle angle;
      angle.first = u8(buffer_[kPacketSJogData + i + 3]);
      angle.second = (
          u8(buffer_[kPacketSJogData + i + 0]) |
          (u8(buffer_[kPacketSJogData + i + 1]) << 8));
      angles.push_back(angle);
    }

    operations_.SJog(angles);
  }

  uint8_t GetStatusError(int servo) const {
    return status_error_ | operations_.ReadRam(servo, 48);
  }

  uint8_t GetStatusDetail(int servo) const {
    return status_detail_ | operations_.ReadRam(servo, 49);
  }

  void SendData(uint8_t from_servo, uint8_t cmd, std::size_t size) {
    tx_buffer_[0] = 0xff;
    tx_buffer_[1] = 0xff;
    tx_buffer_[kPacketSize] = size + 7;
    tx_buffer_[kPacketID] = from_servo;
    tx_buffer_[kPacketCmd] = cmd;
    tx_buffer_[kPacketCs1] =
        CalculateCs1(reinterpret_cast<const char*>(tx_buffer_), size + 7);
    tx_buffer_[kPacketCs2] = (tx_buffer_[kPacketCs1] ^ 0xff) & 0xfe;

    const char* const cbuf = reinterpret_cast<const char*>(tx_buffer_);
    write_outstanding_ = true;
    boost::asio::async_write(
        stream_, boost::asio::buffer(cbuf, size + 7),
        [this](base::ErrorCode error, std::size_t) {
          this->HandleWriteData(error);
        });
  }

  void HandleWriteData(base::ErrorCode error) {
    // TODO jpieper: Log some kind of error if necessary.
    write_outstanding_ = false;
  }

  base::AsyncStream& stream_;
  Operations& operations_;

  char buffer_[256] = {};
  uint8_t status_error_ = 0;
  uint8_t status_detail_ = 0;
  uint8_t tx_buffer_[256] = {};
  bool write_outstanding_ = false;
};

HerkulexProtocol::HerkulexProtocol(
    base::AsyncStream& stream, Operations& operations)
    : impl_(new Impl(stream, operations)) {}

HerkulexProtocol::~HerkulexProtocol() {}

void HerkulexProtocol::AsyncStart(base::ErrorHandler callback) {
  impl_->StartReadHeader();
  callback(base::ErrorCode());
}

}
}
