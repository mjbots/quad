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

#include "mech/nrfusb_client.h"

#include <array>
#include <functional>
#include <iostream>
#include <sstream>

#include <boost/algorithm/string.hpp>
#include <boost/asio/post.hpp>
#include <boost/asio/read_until.hpp>
#include <boost/asio/streambuf.hpp>
#include <boost/asio/write.hpp>

#include <fmt/format.h>

#include "mjlib/base/assert.h"
#include "mjlib/base/fail.h"
#include "mjlib/base/tokenizer.h"
#include "mjlib/io/now.h"

namespace pl = std::placeholders;

namespace mjmech {
namespace mech {

namespace {
std::optional<std::string> ParseHexData(const std::string& data) {
  if (data.size() % 2 != 0) { return {}; }
  std::string result;
  for (size_t i = 0; i < data.size(); i += 2) {
    const int element = std::stoi(data.substr(i, 2), nullptr, 16);
    result.push_back(static_cast<char>(element));
  }
  return result;
}

std::string FormatHexData(const std::string& data) {
  std::ostringstream ostr;
  for (char c : data) {
    ostr << fmt::format("{:02x}", static_cast<int>(static_cast<uint8_t>(c)));
  }
  return ostr.str();
}
}

class NrfusbClient::Impl {
 public:
  Impl(mjlib::io::AsyncStream* stream, const Options& options)
      : stream_(stream) {
    Write(fmt::format("\nconf set slot.ids.0 {}\nconf set slot.ids.1 {}\n",
                      options.id0, options.id1));
    StartRead();
  }

  void AsyncWaitForSlot(int* remote,
                        uint16_t* bitfield, mjlib::io::ErrorCallback callback) {
    BOOST_ASSERT(!bitfield_);
    remote_ = remote;
    bitfield_ = bitfield;
    callback_ = std::move(callback);
    MaybeProcessCallback();
  }

  Slot rx_slot(int remote, int slot_idx) {
    MJ_ASSERT(slot_idx >= 0 && slot_idx < 15);
    return rx_slots_[remote][slot_idx];
  }

  void tx_slot(int remote, int slot_idx, const Slot& slot) {
    MJ_ASSERT(slot_idx >= 0 && slot_idx < 15);
    tx_slots_[remote][slot_idx] = slot;

    Write(fmt::format("slot pri2 {} {} {:08x}\nslot tx2 {} {} {}\n",
                      remote, slot_idx, slot.priority,
                      remote, slot_idx, FormatHexData({slot.data, slot.size})));
  }

  Slot tx_slot(int remote, int slot_idx) {
    MJ_ASSERT(slot_idx >= 0 && slot_idx < 15);
    return tx_slots_[remote][slot_idx];
  }

 private:
  void MaybeProcessCallback() {
    if (!bitfield_) { return; }
    if (pending_bitfield_ == 0) { return; }

    *bitfield_ = pending_bitfield_;
    pending_bitfield_ = 0;

    boost::asio::post(
        stream_->get_executor(),
        std::bind(std::move(callback_), mjlib::base::error_code()));

    remote_ = nullptr;
    bitfield_ = nullptr;
    callback_ = {};
  }

  void StartRead() {
    boost::asio::async_read_until(
        *stream_,
        read_streambuf_,
        '\n',
        std::bind(&Impl::HandleRead, this, pl::_1));
  }

  void HandleRead(const mjlib::base::error_code& ec) {
    mjlib::base::FailIf(ec);

    std::istream in(&read_streambuf_);
    std::string line;
    std::getline(in, line);

    HandleLine(line);

    StartRead();
  }

  void HandleLine(const std::string& line_in) {
    std::string line = boost::trim_copy(line_in);
    if (line.empty()) { return; }
    if (line == "OK") { return; }

    if (line.substr(0, 5) == "rcv2 ") {
      HandleReceive2(line.substr(5));
    } else if (line.substr(0, 4) == "rcv ")  {
      HandleReceive(0, line.substr(4));
    }

    // Some unknown line.  We'll ignore for now.
  }

  void HandleReceive2(const std::string& line) {
    mjlib::base::Tokenizer tokenizer(line, " ");
    const int remote = std::strtol(tokenizer.next().data(), nullptr, 0);
    HandleReceive(remote, std::string(tokenizer.remaining()));
  }

  void HandleReceive(int remote, const std::string& line) {
    const auto now = mjlib::io::Now(stream_->get_executor().context());

    std::vector<std::string> fields;
    boost::split(fields, line, boost::is_any_of(" "));
    for (const auto& field : fields) {
      HandleField(now, remote, field);
    }

    if (remote_) {
      *remote_ = remote;
    }

    MaybeProcessCallback();
  }

  void HandleField(boost::posix_time::ptime now,
                   int remote,
                   const std::string& field) {
    const size_t pos = field.find(':');
    if (pos == std::string::npos) {
      return;
    }

    const int slot_num = std::stoi(field.substr(0, pos));

    if (slot_num < 0 || slot_num > 14) {
      return;
    }

    const auto maybe_data = ParseHexData(field.substr(pos + 1));
    if (!maybe_data) {
      return;
    }

    const auto data = *maybe_data;

    auto& slot = rx_slots_[remote][slot_num];
    const auto to_copy =
        std::min<std::size_t>(sizeof(slot.data), data.size());
    std::memcpy(&slot.data[0], data.data(), to_copy);
    slot.size = to_copy;
    slot.timestamp = now;

    pending_bitfield_ |= (1 << slot_num);
  }

  void Write(const std::string& data) {
    std::ostream os(&write_streambuf_);
    os.write(data.data(), data.size());
    MaybeStartWrite();
  }

  void MaybeStartWrite() {
    if (write_outstanding_) { return; }
    if (write_streambuf_.size() == 0) { return; }

    write_outstanding_ = true;
    boost::asio::async_write(
        *stream_,
        write_streambuf_,
        std::bind(&Impl::HandleWrite, this, pl::_1));
  }

  void HandleWrite(const mjlib::base::error_code& ec) {
    write_outstanding_ = false;
    mjlib::base::FailIf(ec);

    MaybeStartWrite();
  }

  mjlib::io::AsyncStream* const stream_;

  std::map<int, std::array<Slot, 15>> rx_slots_ = {};
  std::map<int, std::array<Slot, 15>> tx_slots_ = {};

  boost::asio::streambuf read_streambuf_;
  boost::asio::streambuf write_streambuf_;
  bool write_outstanding_ = false;

  uint16_t pending_bitfield_ = 0;

  int* remote_ = nullptr;
  uint16_t* bitfield_ = nullptr;
  mjlib::io::ErrorCallback callback_;
};

NrfusbClient::NrfusbClient(mjlib::io::AsyncStream* stream,
                           const Options& options)
    : impl_(std::make_unique<Impl>(stream, options)) {}

NrfusbClient::~NrfusbClient() {}

void NrfusbClient::AsyncWaitForSlot(
    int* remote, uint16_t* bitfield, mjlib::io::ErrorCallback callback) {
  impl_->AsyncWaitForSlot(remote, bitfield, std::move(callback));
}

NrfusbClient::Slot NrfusbClient::rx_slot(int remote, int slot_idx) {
  return impl_->rx_slot(remote, slot_idx);
}

void NrfusbClient::tx_slot(int remote, int slot_idx, const Slot& slot) {
  impl_->tx_slot(remote, slot_idx, slot);
}

NrfusbClient::Slot NrfusbClient::tx_slot(int remote, int slot_idx) {
  return impl_->tx_slot(remote, slot_idx);
}


}
}
