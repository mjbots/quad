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

#pragma once

#include <boost/asio/deadline_timer.hpp>
#include <boost/asio/read.hpp>
#include <boost/asio/read_until.hpp>
#include <boost/asio/streambuf.hpp>
#include <boost/asio/write.hpp>
#include <boost/signals2/signal.hpp>

#include "base/comm.h"
#include "base/command_sequencer.h"
#include "base/common.h"
#include "base/fail.h"
#include "base/signal_result.h"
#include "base/visitor.h"

namespace mjmech {
namespace mech {

template <class Factory>
class HerkuleXProtocol : boost::noncopyable {
 public:
  struct Packet {
    int servo = 0;
    int command = 0;
    std::string data;
    bool cksum_good = false;
  };

  typedef boost::optional<Packet> OptionalPacket;

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

  HerkuleXProtocol(boost::asio::io_service& service,
                   Factory& factory)
      : service_(service),
        factory_(factory),
        sequencer_(service) {}

  struct Parameters {
    typename Factory::Parameters stream;
    double packet_timeout_s = 0.05;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(stream));
      a->Visit(MJ_NVP(packet_timeout_s));
    }
  };

  Parameters* parameters() { return &parameters_; }

  template <typename Handler>
  BOOST_ASIO_INITFN_RESULT_TYPE(Handler,
                                void(boost::system::error_code))
  AsyncStart(Handler handler) {
    boost::asio::detail::async_result_init<
        Handler,
        void (boost::system::error_code)> init(
            BOOST_ASIO_MOVE_CAST(Handler)(handler));

    factory_.AsyncCreate(
        parameters_.stream,
        std::bind(&HerkuleXProtocol::HandleStart, this, init.handler,
                  std::placeholders::_1,
                  std::placeholders::_2));

    return init.result.get();
  }

  template <typename Handler>
  BOOST_ASIO_INITFN_RESULT_TYPE(Handler,
                                void(boost::system::error_code))
  SendPacket(const Packet& packet,
             Handler handler) {
    boost::asio::detail::async_result_init<
        Handler,
        void (boost::system::error_code)> init(
            BOOST_ASIO_MOVE_CAST(Handler)(handler));

    sequencer_.Invoke(
        std::bind(&HerkuleXProtocol::RawSendPacket,
                  this,
                  packet,
                  std::placeholders::_1),
        init.handler);

    return init.result.get();
  }

  /// Receive a single packet, or raise a TimeoutError if no packet is
  /// received within @param timeout_s seconds.
  template <typename Handler>
  BOOST_ASIO_INITFN_RESULT_TYPE(Handler,
                                void(boost::system::error_code, Packet))
  ReceivePacket(Handler handler) {
    return base::SignalResult::Wait(
        service_, &read_signal_,
        parameters_.packet_timeout_s, handler);
  }

  template <typename Handler>
  BOOST_ASIO_INITFN_RESULT_TYPE(Handler,
                                void(boost::system::error_code, Packet))
  SendReceivePacket(const Packet& to_send, Handler handler) {
    boost::asio::detail::async_result_init<
        Handler,
        void (boost::system::error_code, Packet)> init(
            BOOST_ASIO_MOVE_CAST(Handler)(handler));

    typedef std::function<void (const boost::system::error_code& ec,
                                const Packet& packet)> PacketHandler;
    sequencer_.Invoke(
        [=](PacketHandler packet_handler) {
          RawSendPacket(
              to_send,
              [=](base::ErrorCode ec) {
                if (ec) {
                  packet_handler(ec, Packet());
                  return;
                }

                base::SignalResult::Wait(
                    service_, &read_signal_,
                    parameters_.packet_timeout_s, packet_handler);
              });
        },
        init.handler);

    return init.result.get();
  }

  template <typename Handler>
  void Post(Handler handler) { service_.post(handler); }

 private:
  void RawSendPacket(const Packet& packet, base::ErrorHandler handler) {
    if (!stream_) {
      service_.post(std::bind(handler, base::ErrorCode()));
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
        [handler](const boost::system::error_code& ec,
                  size_t written) mutable {
          handler(ec);
        });
  }

  void HandleStart(base::ErrorHandler handler,
                   base::ErrorCode ec,
                   base::SharedStream stream) {
    if (ec) {
      service_.post(std::bind(handler, ec));
      return;
    }

    BOOST_ASSERT(!stream_);
    stream_ = stream;

    ReadLoop1();

    service_.post(std::bind(handler, base::ErrorCode()));
  }

  void ReadLoop1() {
    // Read until we get a frame header.
    boost::asio::async_read_until(
        *stream_, rx_streambuf_, "\xff\xff",
        std::bind(&HerkuleXProtocol::ReadLoop2, this,
                  std::placeholders::_1,
                  std::placeholders::_2));
  }

  void ReadLoop2(base::ErrorCode ec, std::size_t) {
    if (ec == boost::asio::error::not_found) {
      // We're just seeing junk.  It would be nice to log, but for
      // now, just go back and read some more.
      rx_streambuf_.consume(rx_streambuf_.size());
      ReadLoop1();
      return;
    }
    FailIf(ec);

    // Clear out the streambuf until we have our delimeter.
    std::istream istr(&rx_streambuf_);
    std::string buf;
    while (buf != "\xff\xff") {
      char c = 0;
      istr.read(&c, 1);
      if (istr.gcount() != 1) {
        base::Fail("inconsistent header");
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
      ReadLoop3(base::ErrorCode(), 0);
    }
  }

  void ReadLoop3(base::ErrorCode ec, std::size_t) {
    FailIf(ec);

    std::array<char, 5> header = {};
    std::istream istr(&rx_streambuf_);
    istr.read(&header[0], 5);
    if (istr.gcount() != 5) {
      base::Fail("inconsistent header");
    }

    const uint8_t size = header[0];

    if (size < 7) {
      // Malformed header.  Skip back to looking for
      // synchronization.  Probably should log?

      // TODO jpieper: Think about logging.
      ReadLoop1();
      return;
    }

    if (rx_streambuf_.size() < (size - 7)) {
      boost::asio::async_read(
          *stream_,
          rx_streambuf_,
          boost::asio::transfer_at_least(
              (size - 7) - rx_streambuf_.size()),
          std::bind(&HerkuleXProtocol::ReadLoop4, this,
                    std::placeholders::_1,
                    std::placeholders::_2, header));
    } else {
      ReadLoop4(base::ErrorCode(), 0, header);
    }
  }

  void ReadLoop4(base::ErrorCode ec, std::size_t,
                 std::array<char, 5> header) {
    FailIf(ec);

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

  boost::asio::io_service& service_;
  Factory& factory_;
  Parameters parameters_;
  base::CommandSequencer sequencer_;
  base::SharedStream stream_;
  boost::signals2::signal<void (const Packet*)> read_signal_;
  char buffer_[256] = {};
  boost::asio::streambuf rx_streambuf_{512};
};

struct HerkuleXConstants {
  struct Register {
    constexpr Register(const uint8_t position, const uint8_t length)
        : position(position), length(length) {}

    uint8_t position;
    uint8_t length;
  };

  HerkuleXConstants();

  const std::map<std::string, Register> ram_registers;

  // RAM registers
  static constexpr Register temperature_c() { return Register{55, 1}; }
  static constexpr Register voltage() { return Register{54, 1}; }
  static constexpr Register position() { return Register{60, 2}; }
  // NOTE: Older versions of the datasheet say this should be a RAM
  // register 62, but the newer ones have the correct value of 64.
  static constexpr Register pwm() { return Register{64, 2}; }
  static constexpr Register cal_diff() { return Register{53, 1}; }
  static constexpr Register torque_control() { return Register{52, 1}; }


  // EEPROM registers
  static constexpr Register id() { return Register{6, 1}; }
};

template <class Factory>
class HerkuleX : public HerkuleXProtocol<Factory> {
 public:
  typedef HerkuleXProtocol<Factory> Base;

  template <typename... Args>
  HerkuleX(Args&&... args)
      : HerkuleXProtocol<Factory>(std::forward<Args>(args)...) {}

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

  struct MemReadResponse : public Base::StatusResponse {
    MemReadResponse() {}

    MemReadResponse(const typename Base::Packet& packet)
        : Base::StatusResponse(packet,
                               packet.data.substr(packet.data.size() - 2)),
          register_start(packet.data.at(0)),
          length(packet.data.at(1)) {}

    uint8_t register_start = 0;
    uint8_t length = 0;
    std::string register_data{
      Base::StatusResponse::data.substr(
          std::min(static_cast<std::size_t>(2),
                   Base::StatusResponse::data.size()), length)};
  };

  template <typename Handler>
  BOOST_ASIO_INITFN_RESULT_TYPE(
      Handler, void(boost::system::error_code, MemReadResponse))
  MemRead(uint8_t command, uint8_t servo,
          uint8_t reg, uint8_t length,
          Handler handler) {
    boost::asio::detail::async_result_init<
        Handler,
        void (boost::system::error_code, MemReadResponse)> init(
            BOOST_ASIO_MOVE_CAST(Handler)(handler));

    typename Base::Packet to_send;
    to_send.servo = servo;
    to_send.command = command;
    uint8_t buf[2] = { reg, length };
    to_send.data = std::string(reinterpret_cast<const char*>(buf), 2);

    this->SendReceivePacket(
        to_send, std::bind(&HerkuleX::MemReadHandler, this,
                           std::placeholders::_1,
                           std::placeholders::_2,
                           reg, length, to_send, init.handler));

    return init.result.get();
  }

  void MemReadHandler(base::ErrorCode ec,
                      typename Base::Packet result,
                      uint8_t reg,
                      uint8_t length,
                      typename Base::Packet to_send,
                      std::function<void (base::ErrorCode,
                                          MemReadResponse)> handler) {
    auto post_error = [&](base::ErrorCode ec) {
      this->Post(std::bind(handler, ec, MemReadResponse()));
    };

    if (ec) {
      ec.Append(boost::format("when reading register %d") %
                static_cast<int>(reg));
      post_error(ec);
      return;
    }

    if (result.servo != to_send.servo && to_send.servo != BROADCAST) {
      post_error(
          base::ErrorCode::einval(
              (boost::format("Synchronization error, sent request for servo "
                             "0x%02x, response from 0x%02x") %
               static_cast<int>(to_send.servo) %
               static_cast<int>(result.servo)).str()));
      return;
    }

    if (result.command != (0x40 | to_send.command)) {
      post_error(
          base::ErrorCode::einval(
              (boost::format(
                  "Expected response command 0x%02x, received 0x%02x") %
               static_cast<int>(0x40 | to_send.command) %
               static_cast<int>(result.command)).str()));
      return;
    }

    if (result.data.size() != length + 4) {
      post_error(
          base::ErrorCode::einval(
              (boost::format("Expected length response %d, received %d") %
               static_cast<int>(length + 4) %
               result.data.size()).str()));
      return;
    }

    MemReadResponse response = result;

    if (response.register_start != reg) {
      post_error(
          base::ErrorCode::einval(
              (boost::format("Expected register 0x%02x, received 0x%02x") %
               static_cast<int>(reg) %
               static_cast<int>(response.register_start)).str()));
      return;
    }

    if (response.length != length) {
      post_error(
          base::ErrorCode::einval(
              (boost::format("Expected length %d, received %d") %
               static_cast<int>(length) %
               static_cast<int>(response.length)).str()));
      return;
    }

    handler(base::ErrorCode(), response);
  }

  typedef std::function<void (const boost::system::error_code&,
                              typename Base::StatusResponse)> StatusHandler;

  template <typename Handler>
  BOOST_ASIO_INITFN_RESULT_TYPE(
      Handler, void(boost::system::error_code, typename Base::StatusResponse))
  Status(uint8_t servo, Handler handler) {
    boost::asio::detail::async_result_init<
      Handler,
      void (boost::system::error_code, typename Base::StatusResponse)> init(
          BOOST_ASIO_MOVE_CAST(Handler)(handler));

    typename Base::Packet to_send;
    to_send.servo = servo;
    to_send.command = STAT;

    this->SendReceivePacket(
        to_send, std::bind(&HerkuleX::HandleStatusResponse, this,
                           std::placeholders::_1,
                           std::placeholders::_2,
                           to_send, StatusHandler(init.handler)));

    return init.result.get();
  }

  void HandleStatusResponse(
      base::ErrorCode ec,
      typename Base::StatusResponse result,
      typename Base::Packet to_send,
      StatusHandler handler) {
    auto post_error = [&](base::ErrorCode ec) {
      this->Post(std::bind(handler, ec, typename Base::StatusResponse()));
    };

    if (ec) {
      post_error(ec);
      return;
    }

    if (result.servo != to_send.servo && to_send.servo != BROADCAST) {
      post_error(
          base::ErrorCode::einval(
              (boost::format(
                  "Synchronization error, send status to servo 0x%02x, "
                  "response from 0x%02x") %
               static_cast<int>(to_send.servo) %
               static_cast<int>(result.servo)).str()));
      return;
    }

    if (result.command != ACK_STAT) {
      post_error(
          base::ErrorCode::einval(
              (boost::format(
                  "Expected response command 0x%02x, received 0x%02x") %
               static_cast<int>(ACK_STAT) %
               static_cast<int>(result.command)).str()));
      return;
    }

    if (result.data.size() != 2) {
      post_error(
          base::ErrorCode::einval(
              (boost::format("Received status of incorrect size, expected 2, "
                             "got %d") %
               result.data.size()).str()));
      return;
    }
    handler(base::ErrorCode(), typename Base::StatusResponse(result));
  }

  template <typename Handler>
  BOOST_ASIO_INITFN_RESULT_TYPE(
      Handler, void(boost::system::error_code))
  Reboot(uint8_t servo, Handler handler) {
    boost::asio::detail::async_result_init<
        Handler,
        void (boost::system::error_code)> init(
            BOOST_ASIO_MOVE_CAST(Handler)(handler));

    typename Base::Packet to_send;
    to_send.servo = servo;
    to_send.command = REBOOT;

    this->SendPacket(to_send, base::ErrorHandler(init.handler));

    return init.result.get();
  }

  template <typename Handler>
  BOOST_ASIO_INITFN_RESULT_TYPE(
      Handler, void(boost::system::error_code))
  MemWrite(uint8_t command, uint8_t servo, uint8_t address,
           const std::string& data,
           Handler handler) {
    boost::asio::detail::async_result_init<
        Handler,
        void (boost::system::error_code)> init(
            BOOST_ASIO_MOVE_CAST(Handler)(handler));

    typename Base::Packet to_send;
    to_send.servo = servo;
    to_send.command = command;
    std::ostringstream ostr;

    ostr.write(reinterpret_cast<const char*>(&address), 1);
    uint8_t length = data.size();
    ostr.write(reinterpret_cast<const char*>(&length), 1);

    ostr.write(data.data(), data.size());

    to_send.data = ostr.str();

    this->SendPacket(to_send, base::ErrorHandler(init.handler));

    return init.result.get();
  }

  template <typename T, typename Handler>
  BOOST_ASIO_INITFN_RESULT_TYPE(
      Handler, void(boost::system::error_code))
  MemWriteValue(uint8_t command, uint8_t servo, T field, int value_in,
                Handler handler) {
    boost::asio::detail::async_result_init<
        Handler,
        void (boost::system::error_code)> init(
            BOOST_ASIO_MOVE_CAST(Handler)(handler));

    int value = value_in;

    std::ostringstream ostr;
    for (int i = 0; i < field.length; i++) {
      uint8_t byte = value & 0xff;
      ostr.write(reinterpret_cast<const char*>(&byte), 1);
      value = value >> 8;
    }

    MemWrite(command, servo, field.position, ostr.str(),
             base::ErrorHandler(init.handler));

    return init.result.get();
  }

  template <typename T, typename Handler>
  void RamWrite(uint8_t servo, T field, int value, Handler handler) {
    MemWriteValue(RAM_WRITE, servo, field, value, handler);
  }

  template <typename T, typename Handler>
  void EepWrite(uint8_t servo, T field, int value, Handler handler) {
    MemWriteValue(EEP_WRITE, servo, field, value, handler);
  }

  typedef std::function<void (base::ErrorCode, int)> IntHandler;

  template <typename T, typename Handler>
  BOOST_ASIO_INITFN_RESULT_TYPE(
      Handler, void(boost::system::error_code, int))
  MemReadValue(uint8_t command, uint8_t servo, T field,
               Handler handler) {
    boost::asio::detail::async_result_init<
        Handler,
        void (boost::system::error_code, int)> init(
            BOOST_ASIO_MOVE_CAST(Handler)(handler));

    MemRead(
        command, servo, field.position, field.length,
        [=](base::ErrorCode ec, MemReadResponse response) mutable {
          if (ec) {
            init.handler(ec, 0);
            return;
          }

          int result = 0;
          BOOST_ASSERT(response.register_data.size() >= field.length);
          for (int i = 0; i < field.length; i++) {
            result |= (static_cast<uint8_t>(
                           response.register_data[i]) << (i * 8));
          }

          init.handler(base::ErrorCode(), result);
        });

    return init.result.get();
  }

  template <typename T, typename Handler>
  BOOST_ASIO_INITFN_RESULT_TYPE(
      Handler, void(boost::system::error_code, int))
  RamRead(uint8_t servo, T field, Handler handler) {
    return MemReadValue(RAM_READ, servo, field, handler);
  }

  template <typename T, typename Handler>
  BOOST_ASIO_INITFN_RESULT_TYPE(
      Handler, void(boost::system::error_code, int))
  EepRead(uint8_t servo, T field, Handler handler) {
    return MemReadValue(EEP_READ, servo, field, handler);
  }

  template <typename Targets, typename Handler>
  BOOST_ASIO_INITFN_RESULT_TYPE(
      Handler, void(boost::system::error_code))
  SJog(const Targets& targets,
       double time_s,
       Handler handler) {
    boost::asio::detail::async_result_init<
        Handler,
        void (boost::system::error_code)> init(
            BOOST_ASIO_MOVE_CAST(Handler)(handler));

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

    typename Base::Packet to_send;
    to_send.servo = BROADCAST;
    to_send.data = data.str();
    to_send.command = S_JOG;

    this->SendPacket(to_send, base::ErrorHandler(init.handler));

    return init.result.get();
  }
};
}
}
