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

#include "async_stream.h"

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
  kPacketAddress = 7,
  kPacketDataLength = 8,
  kPacketData = 9,
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

class HerkulexProtocol::Impl {
 public:
  Impl(AsyncStream& stream, Operations& operations)
      : stream_(stream), operations_(operations) {}

  void StartReadHeader() {
    AsyncRead(stream_, gsl::string_span(buffer_, 1),
              [this](int error) { this->HandleReadHeader1(error); });
  }

  void HandleReadHeader1(int error) {
    if (error) { HandleError(error); return; }

    if (u8(buffer_[0]) != 0xff) { StartReadHeader(); return; }

    AsyncRead(stream_, gsl::string_span(buffer_ + 1, buffer_ + 2),
              [this](int error) { this->HandleReadHeader2(error); });
  }

  void HandleReadHeader2(int error) {
    if (error) { HandleError(error); return; }
    if (u8(buffer_[1]) != 0xff) { StartReadHeader(); return; }

    AsyncRead(stream_, gsl::string_span(buffer_ + 2, buffer_ + 7),
              [this](int error) { this->HandleReadHeader3(error); });
  }

  void HandleReadHeader3(int error) {
    if (error) { HandleError(error); return; }

    const uint8_t packet_size = u8(buffer_[kPacketSize]);

    if (packet_size < 7) {
      status_error_ = kErrorInvalidPacket;
      status_detail_ = kDetailGarbageDetected;
      HandleError(1);
      return;
    }

    AsyncRead(
        stream_, gsl::string_span(buffer_ + 7, buffer_ + packet_size),
        [this](int error) { this->HandleReadData(error); });
  }

  void HandleReadData(int error) {
    if (error) { HandleError(error); return; }

    ProcessPacket();

    StartReadHeader();
  }

  void HandleError(int error) {
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

    if (id != operations_.address() &&
        id != kBroadcast) {
      // They aren't talking to us.
      return;
    }

    const uint8_t cmd = u8(buffer_[kPacketCmd]);

    if (cmd == 0x09) { // REBOOT
      operations_.Reboot();
    } else if (cmd == 0x07) {
      HandleStat();
    } else if (cmd == 0x03) {
      HandleRamWrite();
    } else if (cmd == 0x04) {
      HandleRamRead();
    } else if (cmd == 0x05 || // I_JOG
               cmd == 0x06 || // S_JOG
               cmd == 0x08) { // ROLLBACK
      // silently ignore
    } else {
      status_error_ |= kErrorInvalidPacket;
      status_detail_ |= kDetailChecksumError;
    }
  }

  void HandleStat() {
    if (write_outstanding_) { return; }

    tx_buffer_[kPacketData] = status_error_;
    tx_buffer_[kPacketData + 1] = status_detail_;

    SendData(0x47, 2); // STAT_ACK
  }

  void HandleRamWrite() {
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
      operations_.WriteRam(addr + i, u8(buffer_[kPacketData + i]));
    }
  }

  void HandleRamRead() {
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
      tx_buffer_[kPacketData + i] = operations_.ReadRam(address + i);
    }
    tx_buffer_[kPacketData + 0 + len] = status_error_;
    tx_buffer_[kPacketData + 1 + len] = status_detail_;

    SendData(0x44, len + 4);
  }

  void SendData(uint8_t cmd, std::size_t size) {
    tx_buffer_[0] = 0xff;
    tx_buffer_[1] = 0xff;
    tx_buffer_[kPacketSize] = size + 7;
    tx_buffer_[kPacketID] = operations_.address();
    tx_buffer_[kPacketCmd] = cmd;
    tx_buffer_[kPacketCs1] =
        CalculateCs1(reinterpret_cast<const char*>(tx_buffer_), size + 7);
    tx_buffer_[kPacketCs2] = (tx_buffer_[kPacketCs1] ^ 0xff) & 0xfe;

    const char* const cbuf = reinterpret_cast<const char*>(tx_buffer_);
    write_outstanding_ = true;
    AsyncWrite(stream_, gsl::cstring_span(cbuf, cbuf + size + 7),
               [this](int error) { this->HandleWriteData(error); });
  }

  void HandleWriteData(int error) {
    // TODO jpieper: Log some kind of error if necessary.
    write_outstanding_ = false;
  }

  AsyncStream& stream_;
  Operations& operations_;

  char buffer_[256] = {};
  uint8_t status_error_ = 0;
  uint8_t status_detail_ = 0;
  uint8_t tx_buffer_[256] = {};
  bool write_outstanding_ = false;
};

HerkulexProtocol::HerkulexProtocol(
    Pool& pool, AsyncStream& stream, Operations& operations)
    : impl_(&pool, stream, operations) {}

HerkulexProtocol::~HerkulexProtocol() {}

void HerkulexProtocol::AsyncStart(ErrorCallback callback) {
  impl_->StartReadHeader();
  callback(0);
}
