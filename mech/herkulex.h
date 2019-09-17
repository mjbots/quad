// Copyright 2015-2019 Josh Pieper, jjp@pobox.com.  All rights reserved.
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

#pragma once

#include <numeric>
#include <optional>

#include <boost/asio/read.hpp>
#include <boost/asio/read_until.hpp>
#include <boost/asio/streambuf.hpp>
#include <boost/asio/write.hpp>
#include <boost/signals2/signal.hpp>

#include <fmt/format.h>

#include "mjlib/base/fail.h"
#include "mjlib/base/program_options_archive.h"
#include "mjlib/base/visitor.h"
#include "mjlib/io/stream_factory.h"

#include "base/command_sequencer.h"
#include "base/common.h"
#include "base/program_options.h"
#include "base/signal_result.h"

namespace mjmech {
namespace mech {

enum class herkulex_error {
  synchronization_error = 1,
};

const boost::system::error_category& herkulex_category();

boost::system::error_code make_error_code(herkulex_error e);
}
}

namespace boost {
namespace system {
template <>
struct is_error_code_enum<mjmech::mech::herkulex_error> {
  static const bool value = true;
};
}
}

namespace mjmech {
namespace mech {

class HerkuleXBase : boost::noncopyable {
 public:
  struct Packet {
    int servo = 0;
    int command = 0;
    std::string data;
    bool cksum_good = false;
  };

  typedef std::optional<Packet> OptionalPacket;

  struct StatusResponse : public Packet {
    StatusResponse() {}

    StatusResponse(const Packet& packet)
        : StatusResponse(packet, packet.data) {}

    StatusResponse(const Packet& packet,
                   const std::string& status_data)
        : Packet(packet),
          reg48(status_data.size() > 0 ? status_data.at(0) : 0),
          reg49(status_data.size() > 1 ? status_data.at(1) : 0) {}

    uint8_t reg48 = 0;
    uint8_t reg49 = 0;

    bool exceeded_input_voltage_limit = (reg48 & 0x01) != 0;
    bool exceeded_allowed_pot_limit = (reg48 & 0x02) != 0;
    bool exceeded_temperature_limit = (reg48 & 0x04) != 0;
    bool invalid_packet = (reg48 & 0x08) != 0;
    bool overload_detected = (reg48 & 0x10);
    bool driver_fault_detected = (reg48 & 0x20);
    bool eep_reg_distorted = (reg48 & 0x40);

    bool moving = (reg49 & 0x01) != 0;
    bool inposition = (reg49 & 0x02) != 0;
    bool checksum_error = (reg49 & 0x04) != 0;
    bool unknown_command = (reg49 & 0x08) != 0;
    bool exceeded_reg_range = (reg49 & 0x10) != 0;
    bool garbage_detected = (reg49 & 0x20) != 0;
    bool motor_on = (reg49 & 0x40) != 0;
  };

  enum Command {
    EEP_WRITE = 0x01,
    EEP_READ = 0x02,
    RAM_WRITE = 0x03,
    RAM_READ = 0x04,
    I_JOG = 0x05,
    S_JOG = 0x06,
    STAT = 0x07,
    ROLLBACK = 0x08,
    REBOOT = 0x09,
    ACK_EEP_WRITE = 0x41,
    ACK_EEP_READ = 0x42,
    ACK_RAM_WRITE = 0x43,
    ACK_RAM_READ = 0x44,
    ACK_I_JOG = 0x45,
    ACK_S_JOG = 0x46,
    ACK_STAT = 0x47,
    ACK_ROLLBACK = 0x48,
    ACK_REBOOT = 0x49,
  };

  enum MagicID {
    BROADCAST = 0xfe,
  };

  struct MemReadResponse : public StatusResponse {
    MemReadResponse() {}

    MemReadResponse(const Packet& packet)
    : StatusResponse(packet,
                     packet.data.substr(packet.data.size() - 2)),
      register_start(packet.data.at(0)),
      length(packet.data.at(1)) {}

    uint8_t register_start = 0;
    uint8_t length = 0;
    std::string register_data{
      StatusResponse::data.substr(
          std::min(static_cast<std::size_t>(2),
                   StatusResponse::data.size()), length)};
  };

  static uint16_t AngleToCount(double angle_deg) {
    return std::min(
        1023, std::max(0, static_cast<int>(512 + angle_deg / 0.325)));
  }

  static double CountsToAngleDeg(int counts) {
    return (counts - 512) * 0.325;
  }

  static double CountsToTemperatureC(int counts) {
    // Note, this formula was derived from the Dongbu lookup table,
    // and becomes terribly inaccurate below -20C.
    return (counts - 40) * 0.5125 - 19.38;
  }

  static double CountsToVoltage(int counts) {
    return counts * 0.074;
  }
};

struct HerkuleXConstants {
  struct Register {
    constexpr Register(
        const uint8_t position, const uint8_t length,
        const uint8_t bit_align=8, bool sign=false)
        : position(position), length(length),
          bit_align(bit_align), sign(sign) {}

    uint8_t position;
    uint8_t length;
    uint8_t bit_align;
    bool sign;
  };

  HerkuleXConstants();

  const std::map<std::string, Register> ram_registers;

  // RAM registers
  static constexpr Register status_error() { return Register{48, 1}; }
  static constexpr Register status_details() { return Register{49, 1}; }
  static constexpr Register temperature_c() { return Register{55, 1}; }
  static constexpr Register voltage() { return Register{54, 1}; }
  static constexpr Register position() { return Register{60, 2}; }
  // NOTE: Older versions of the datasheet say this should be a RAM
  // register 62, but the newer ones have the correct value of 64.
  static constexpr Register pwm() { return Register{64, 2}; }
  static constexpr Register cal_diff() { return Register{53, 1}; }
  static constexpr Register torque_control() { return Register{52, 1}; }
  static constexpr Register min_voltage() { return Register{6, 1}; }
  static constexpr Register max_voltage() { return Register{7, 1}; }


  // EEPROM registers
  static constexpr Register id() { return Register{6, 1}; }
};

class HerkuleXProtocol : public HerkuleXBase {
 public:
  HerkuleXProtocol(boost::asio::io_context& service,
                   mjlib::io::StreamFactory& factory)
      : service_(service),
        factory_(factory),
        sequencer_(service) {
    mjlib::base::ProgramOptionsArchive(&options_).Accept(&parameters_);
    mjlib::base::ProgramOptionsArchive(&stream_options_).Accept(&stream_parameters_);
    base::MergeProgramOptions(&stream_options_, "stream.", &options_);
  }

  struct Parameters {
    double packet_timeout_s = 0.05;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(packet_timeout_s));
    }
  };

  Parameters* parameters() { return &parameters_; }
  boost::program_options::options_description* options() { return &options_; }

  template <typename Handler>
  void AsyncStart(Handler handler) {
    factory_.AsyncCreate(
        stream_parameters_,
        std::bind(&HerkuleXProtocol::HandleStart, this, handler,
                  std::placeholders::_1,
                  std::placeholders::_2));
  }

  template <typename Handler>
  void SendPacket(const Packet& packet, Handler handler) {
    sequencer_.Invoke(
        std::bind(&HerkuleXProtocol::RawSendPacket,
                  this,
                  packet,
                  std::placeholders::_1),
        handler);
  }

  /// Receive a single packet, or raise a TimeoutError if no packet is
  /// received within @param timeout_s seconds.
  template <typename Handler>
  void ReceivePacket(Handler handler) {
    base::SignalResult::Wait(
        service_, &read_signal_,
        parameters_.packet_timeout_s, handler);
  }

  template <typename Handler>
  void SendReceivePacket(const Packet& to_send, Handler handler) {
    typedef std::function<void (const boost::system::error_code& ec,
                                const Packet& packet)> PacketHandler;
    sequencer_.Invoke(
        [=](PacketHandler packet_handler) {
          RawSendPacket(
              to_send,
              [=](mjlib::base::error_code ec) {
                if (ec) {
                  packet_handler(ec, Packet());
                  return;
                }

                base::SignalResult::Wait(
                    service_, &read_signal_,
                    parameters_.packet_timeout_s, packet_handler);
              });
        },
        handler);
  }

  template <typename Handler>
  void Post(Handler handler) { service_.post(handler); }

 private:
  void RawSendPacket(const Packet& packet, mjlib::io::ErrorCallback handler) {
    if (!stream_) {
      service_.post(std::bind(handler, mjlib::base::error_code()));
      return;
    }

    BOOST_ASSERT(packet.data.size() < (216 - 7));
    BOOST_ASSERT(packet.servo >= 0 && packet.servo <= 0xfe);
    BOOST_ASSERT(packet.command >= 0 && packet.command <= 0xff);

    const uint8_t packet_size = 7 + packet.data.size();

    const uint8_t cksum1 =
        Checksum1(packet_size, packet.servo, packet.command, packet.data);

    buffer_[0] = 0xff;
    buffer_[1] = 0xff;
    buffer_[2] = packet_size;
    buffer_[3] = packet.servo;
    buffer_[4] = packet.command;
    buffer_[5] = cksum1 & 0xfe;
    buffer_[6] = (cksum1 ^ 0xff) & 0xfe;
    std::memcpy(&buffer_[7], packet.data.data(), packet.data.size());

    boost::asio::async_write(
        *stream_,
        boost::asio::buffer(buffer_, 7 + packet.data.size()),
        [handler](const boost::system::error_code& ec, size_t) mutable {
          handler(ec);
        });
  }

  void HandleStart(mjlib::io::ErrorCallback handler,
                   mjlib::base::error_code ec,
                   mjlib::io::SharedStream stream) {
    if (ec) {
      service_.post(std::bind(handler, ec));
      return;
    }

    BOOST_ASSERT(!stream_);
    stream_ = stream;

    ReadLoop1();

    service_.post(std::bind(handler, mjlib::base::error_code()));
  }

  void ReadLoop1() {
    // Read until we get a frame header.
    boost::asio::async_read_until(
        *stream_, rx_streambuf_, "\xff\xff",
        std::bind(&HerkuleXProtocol::ReadLoop2, this,
                  std::placeholders::_1,
                  std::placeholders::_2));
  }

  void ReadLoop2(mjlib::base::error_code ec, std::size_t) {
    if (ec == boost::asio::error::not_found) {
      // We're just seeing junk.  It would be nice to log, but for
      // now, just go back and read some more.
      rx_streambuf_.consume(rx_streambuf_.size());
      ReadLoop1();
      return;
    }
    mjlib::base::FailIf(ec);

    // Clear out the streambuf until we have our delimeter.
    std::istream istr(&rx_streambuf_);
    std::string buf;
    while (buf != "\xff\xff") {
      char c = 0;
      istr.read(&c, 1);
      if (istr.gcount() != 1) {
        mjlib::base::Fail("inconsistent header");
      }
      buf = buf + std::string(&c, 1);
      if (buf.size() > 2) {
        buf = buf.substr(buf.size() - 2);
      }
    }

    // Read the header.
    if (rx_streambuf_.size() < 5) {
      boost::asio::async_read(
          *stream_, rx_streambuf_,
          boost::asio::transfer_at_least(5 - rx_streambuf_.size()),
          std::bind(&HerkuleXProtocol::ReadLoop3, this,
                    std::placeholders::_1,
                    std::placeholders::_2));
    } else {
      ReadLoop3(mjlib::base::error_code(), 0);
    }
  }

  void ReadLoop3(mjlib::base::error_code ec, std::size_t) {
    mjlib::base::FailIf(ec);

    std::array<char, 5> header = {};
    std::istream istr(&rx_streambuf_);
    istr.read(&header[0], 5);
    if (istr.gcount() != 5) {
      mjlib::base::Fail("inconsistent header");
    }

    const uint8_t size = header[0];

    if (size < 7) {
      // Malformed header.  Skip back to looking for
      // synchronization.  Probably should log?

      // TODO jpieper: Think about logging.
      ReadLoop1();
      return;
    }

    if (static_cast<int>(rx_streambuf_.size()) < (size - 7)) {
      boost::asio::async_read(
          *stream_,
          rx_streambuf_,
          boost::asio::transfer_at_least(
              (size - 7) - rx_streambuf_.size()),
          std::bind(&HerkuleXProtocol::ReadLoop4, this,
                    std::placeholders::_1,
                    std::placeholders::_2, header));
    } else {
      ReadLoop4(mjlib::base::error_code(), 0, header);
    }
  }

  void ReadLoop4(mjlib::base::error_code ec, std::size_t,
                 std::array<char, 5> header) {
    mjlib::base::FailIf(ec);

    const uint8_t size = header[0];
    const uint8_t servo = header[1];
    const uint8_t command = header[2];
    const uint8_t cksum1 = header[3];
    const uint8_t cksum2 = header[4];

    std::istream istr(&rx_streambuf_);
    char data_buffer[256] = {};
    istr.read(data_buffer, size - 7);
    std::string data(data_buffer, size - 7);

    uint8_t expected_cksum1 = Checksum1(size, servo, command, data);
    uint8_t expected_cksum2 = expected_cksum1 ^ 0xff;

    Packet result;
    result.servo = servo;
    result.command = command;
    result.data = data;
    result.cksum_good = (cksum1 == (expected_cksum1 & 0xfe) &&
                         cksum2 == (expected_cksum2 & 0xfe));

    read_signal_(&result);

    ReadLoop1();
  }

  static uint8_t Checksum1(uint8_t size, uint8_t servo, uint8_t cmd,
                           const std::string& data) {
    return size ^ servo ^ cmd ^
        std::accumulate(data.begin(), data.end(), 0,
                        [](uint8_t a, uint8_t b) { return a ^ b; });
  }

  boost::asio::io_context& service_;
  mjlib::io::StreamFactory& factory_;
  Parameters parameters_;
  mjlib::io::StreamFactory::Options stream_parameters_;
  boost::program_options::options_description options_;
  boost::program_options::options_description stream_options_;
  base::CommandSequencer sequencer_;
  mjlib::io::SharedStream stream_;
  boost::signals2::signal<void (const Packet*)> read_signal_;
  char buffer_[256] = {};
  boost::asio::streambuf rx_streambuf_{512};
};

class HerkuleX : public HerkuleXProtocol {
 public:
  typedef HerkuleXProtocol Base;
  typedef Base::MemReadResponse MemReadResponse;
  typedef Base::Command Command;
  typedef Base::MagicID MagicID;

  template <typename... Args>
  HerkuleX(Args&&... args)
      : HerkuleXProtocol(std::forward<Args>(args)...) {}

  template <typename Handler>
  void MemRead(uint8_t command, uint8_t servo,
               uint8_t reg, uint8_t length,
               Handler handler) {
    Base::Packet to_send;
    to_send.servo = servo;
    to_send.command = command;
    uint8_t buf[2] = { reg, length };
    to_send.data = std::string(reinterpret_cast<const char*>(buf), 2);

    this->SendReceivePacket(
        to_send, std::bind(&HerkuleX::MemReadHandler, this,
                           std::placeholders::_1,
                           std::placeholders::_2,
                           reg, length, to_send, handler));
  }

  void MemReadHandler(mjlib::base::error_code ec,
                      Base::Packet result,
                      uint8_t reg,
                      uint8_t length,
                      Base::Packet to_send,
                      std::function<void (mjlib::base::error_code,
                                          MemReadResponse)> handler) {
    auto post_error = [&](mjlib::base::error_code ec) {
      ec.Append(fmt::format("when reading servo 0x{:02x}",
                            static_cast<int>(to_send.servo)));
      this->Post(std::bind(handler, ec, MemReadResponse()));
    };

    if (ec) {
      ec.Append(fmt::format("when reading register {}",
                            static_cast<int>(reg)));
      post_error(ec);
      return;
    }

    if (result.servo != to_send.servo && to_send.servo != MagicID::BROADCAST) {
      post_error(
          MakeSynchronizationError(
              fmt::format("Synchronization error, sent request for servo "
                          "0x{:02x}, response from 0x{:02x}",
                          static_cast<int>(to_send.servo),
                          static_cast<int>(result.servo))));
      return;
    }

    if (result.command != (0x40 | to_send.command)) {
      post_error(
          MakeSynchronizationError(
              fmt::format(
                  "Expected response command 0x{:02x}, received 0x{:02x}",
                  static_cast<int>(0x40 | to_send.command),
                  static_cast<int>(result.command))));
      return;
    }

    if (static_cast<int>(result.data.size()) != (length + 4)) {
      post_error(
          MakeSynchronizationError(
              fmt::format("Expected length response {}, received {}",
                          static_cast<int>(length + 4),
                          result.data.size())));
      return;
    }

    MemReadResponse response = result;

    if (response.register_start != reg) {
      post_error(
          MakeSynchronizationError(
              fmt::format("Expected register 0x{:02x}, received 0x{:02x}",
                          static_cast<int>(reg),
                          static_cast<int>(response.register_start))));
      return;
    }

    if (response.length != length) {
      post_error(
          MakeSynchronizationError(
              fmt::format("Expected length {}, received {}",
                          static_cast<int>(length),
                          static_cast<int>(response.length))));
      return;
    }

    handler(mjlib::base::error_code(), response);
  }

  typedef std::function<void (const boost::system::error_code&,
                              Base::StatusResponse)> StatusHandler;

  template <typename Handler>
  void Status(uint8_t servo, Handler handler) {
    Base::Packet to_send;
    to_send.servo = servo;
    to_send.command = Command::STAT;

    this->SendReceivePacket(
        to_send, std::bind(&HerkuleX::HandleStatusResponse, this,
                           std::placeholders::_1,
                           std::placeholders::_2,
                           to_send, StatusHandler(handler)));
  }

  void HandleStatusResponse(
      mjlib::base::error_code ec,
      Base::StatusResponse result,
      Base::Packet to_send,
      StatusHandler handler) {
    auto post_error = [&](mjlib::base::error_code ec) {
      this->Post(std::bind(handler, ec, Base::StatusResponse()));
    };

    if (ec) {
      post_error(ec);
      return;
    }

    if (result.servo != to_send.servo && to_send.servo != MagicID::BROADCAST) {
      post_error(
          MakeSynchronizationError(
              fmt::format(
                  "Synchronization error, send status to servo 0x{:02x}, "
                  "response from 0x{:02x}",
                  static_cast<int>(to_send.servo),
                  static_cast<int>(result.servo))));
      return;
    }

    if (result.command != Command::ACK_STAT) {
      post_error(
          MakeSynchronizationError(
              fmt::format(
                  "Expected response command 0x{:02x}, received 0x{:02x}",
                  static_cast<int>(Command::ACK_STAT),
                  static_cast<int>(result.command))));
      return;
    }

    if (result.data.size() != 2) {
      post_error(
          MakeSynchronizationError(
              fmt::format("Received status of incorrect size, expected 2, "
                          "got {}",
                          result.data.size())));
      return;
    }
    handler(mjlib::base::error_code(), Base::StatusResponse(result));
  }

  template <typename Handler>
  void Reboot(uint8_t servo, Handler handler) {
    Base::Packet to_send;
    to_send.servo = servo;
    to_send.command = Command::REBOOT;

    this->SendPacket(to_send, mjlib::io::ErrorCallback(handler));
  }

  template <typename Handler>
  void MemWrite(uint8_t command, uint8_t servo, uint8_t address,
                const std::string& data,
                Handler handler) {
    Base::Packet to_send;
    to_send.servo = servo;
    to_send.command = command;
    std::ostringstream ostr;

    ostr.write(reinterpret_cast<const char*>(&address), 1);
    uint8_t length = data.size();
    ostr.write(reinterpret_cast<const char*>(&length), 1);

    ostr.write(data.data(), data.size());

    to_send.data = ostr.str();

    this->SendPacket(to_send, mjlib::io::ErrorCallback(handler));
  }

  template <typename T, typename Handler>
  void MemWriteValue(uint8_t command, uint8_t servo, T field, int value_in,
                     Handler handler) {
    int value = value_in;

    std::ostringstream ostr;
    for (int i = 0; i < field.length; i++) {
      const uint8_t byte = value & ((1 << field.bit_align) - 1);
      ostr.write(reinterpret_cast<const char*>(&byte), 1);
      value = value >> field.bit_align;
    }

    MemWrite(command, servo, field.position, ostr.str(),
             mjlib::io::ErrorCallback(handler));
  }

  template <typename T, typename Handler>
  void RamWrite(uint8_t servo, T field, int value, Handler handler) {
    MemWriteValue(Command::RAM_WRITE, servo, field, value, handler);
  }

  template <typename T, typename Handler>
  void EepWrite(uint8_t servo, T field, int value, Handler handler) {
    MemWriteValue(Command::EEP_WRITE, servo, field, value, handler);
  }

  typedef std::function<void (mjlib::base::error_code, int)> IntHandler;

  template <typename T, typename Handler>
  void MemReadValue(uint8_t command, uint8_t servo, T field,
                    Handler handler) {
    MemRead(
        command, servo, field.position, field.length,
        [field, handler](mjlib::base::error_code ec, MemReadResponse response) mutable {
          if (ec) {
            handler(ec, 0);
            return;
          }

          int result = 0;
          BOOST_ASSERT(response.register_data.size() >= field.length);
          for (int i = 0; i < field.length; i++) {
            result |= (static_cast<uint8_t>(
                           response.register_data[i]) << (i * field.bit_align));
          }

          handler(mjlib::base::error_code(), result);
        });
  }

  template <typename T, typename Handler>
  void RamRead(uint8_t servo, T field, Handler handler) {
    MemReadValue(Command::RAM_READ, servo, field, handler);
  }

  template <typename T, typename Handler>
  void EepRead(uint8_t servo, T field, Handler handler) {
    MemReadValue(Command::EEP_READ, servo, field, handler);
  }

  template <typename Targets, typename Handler>
  void SJog(const Targets& targets,
            double time_s,
            Handler handler) {
    std::ostringstream data;

    uint8_t int_time = static_cast<uint8_t>(
        std::max(0.0, std::min(255.0, time_s * 1000.0 / 11.2)));
    data.write(reinterpret_cast<const char*>(&int_time), 1);

    for (const auto& target: targets) {
      uint8_t buf[] = {
        static_cast<uint8_t>(target.position & 0xff),
        static_cast<uint8_t>((target.position >> 8) & 0xff),
        target.leds,
        target.address,
      };
      data.write(reinterpret_cast<const char*>(buf), sizeof(buf));
    }

    Base::Packet to_send;
    to_send.servo = MagicID::BROADCAST;
    to_send.data = data.str();
    to_send.command = Command::S_JOG;

    this->SendPacket(to_send, mjlib::io::ErrorCallback(handler));
  }

  static mjlib::base::error_code MakeSynchronizationError(const std::string& message) {
    mjlib::base::error_code result(herkulex_error::synchronization_error);
    result.Append(message);
    return result;
  }
};

class herkulex_category_impl : public boost::system::error_category {
 public:
  virtual const char* name() const noexcept;
  virtual std::string message(int ev) const noexcept;
};

}
}
