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

#include "herkulex.h"

#include <boost/test/auto_unit_test.hpp>

#include "base/comm_factory.h"

using namespace mjmech::base;
using namespace mjmech::mech;

namespace {
class Fixture {
 public:
  Fixture() {
    herkulex_.parameters()->stream.type = "pipe";
    herkulex_.parameters()->stream.Get<PipeGenerator>()->key = "test";

    stream_ = factory_.generator<PipeGenerator>()->GetStream(
        service_, "test", PipeGenerator::Mode::kDirectionB);

    boost::asio::spawn(service_,
                       ErrorWrap([=](boost::asio::yield_context yield) {
        char buf[4096] = {};
        while (true) {
          std::size_t bytes_read =
              stream_->async_read_some(boost::asio::buffer(buf), yield);
          std::ostream ostr(&streambuf_);
          ostr.write(buf, bytes_read);
          bool gotit = true;
          data_received_(&gotit);
        }
        }));
  }

  boost::asio::io_service service_;
  typedef StreamFactory<PipeGenerator> Factory;
  Factory factory_{service_};
  typedef HerkuleX<Factory> Servo;
  Servo herkulex_{service_, factory_};
  SharedStream stream_;
  boost::asio::streambuf streambuf_;
  boost::signals2::signal<void (const bool*)> data_received_;
};
}

BOOST_FIXTURE_TEST_CASE(HerkuleXTest, Fixture) {
  boost::asio::spawn(service_, ErrorWrap([&](boost::asio::yield_context yield) {
      herkulex_.AsyncStart(yield);

      Servo::Packet packet;
      packet.servo = 0xfd;
      packet.command = Servo::STAT;
      packet.data = "";
      herkulex_.SendPacket(packet, yield);

      // The packet should have ended up in our streambuf now.
      {
        std::ostringstream ostr;
        ostr << &streambuf_;
        std::string data = ostr.str();
        BOOST_CHECK_EQUAL(data.size(), 7);
        BOOST_CHECK_EQUAL(data, "\xff\xff\x07\xfd\x07\xfc\x02");
      }

      // Start listening for a packet.
      boost::optional<Servo::Packet> listen;
      boost::asio::spawn(service_,
                         ErrorWrap([&](boost::asio::yield_context yield) {
          listen = herkulex_.ReceivePacket(yield);
          }));

      BOOST_CHECK(!listen);

      // Send out a response over the wire.
      std::string response("\xff\xff\x09\xfd\x47\xf2\x0c\x01\x40");
      boost::asio::async_write(*stream_,  boost::asio::buffer(response), yield);

      // At this point, the other coroutine should have finished with
      // a receipt.
      BOOST_REQUIRE(listen);
      BOOST_CHECK_EQUAL(listen->data, "\x01\x40");
      BOOST_CHECK_EQUAL(listen->servo, 0xfd);
      BOOST_CHECK_EQUAL(listen->command, Servo::ACK_STAT);
      BOOST_CHECK_EQUAL(listen->cksum_good, true);

      Servo::StatusResponse status(*listen);
      BOOST_CHECK_EQUAL(status.reg48, 0x01);
      BOOST_CHECK_EQUAL(status.reg49, 0x40);
      BOOST_CHECK_EQUAL(status.exceeded_input_voltage_limit, true);
      BOOST_CHECK_EQUAL(status.exceeded_allowed_pot_limit, false);
      }));

  service_.run();
}

BOOST_FIXTURE_TEST_CASE(HerkuleXMemRead, Fixture) {
  // Listen for the request, and reply with a response when necessary.
  boost::asio::spawn(service_, ErrorWrap([&](boost::asio::yield_context yield) {
        for (int i = 0; i < 2; i++) {
          while (streambuf_.size() < 9) {
            SignalResult::Wait(service_, &data_received_, 1.0, yield);
          }

          std::istream istr(&streambuf_);
          char received[9];
          istr.read(received, sizeof(received));
          BOOST_REQUIRE_EQUAL(istr.gcount(), sizeof(received));
          BOOST_CHECK_EQUAL(std::string(received, 9),
                            "\xff\xff\x09\xfd\x04\xc4\x3a\x35\x01");

          std::string response(
              "\xff\xff\x0c\xfd\x44\xc2\x3c\x35\x01\x01\x00\x42", 12);
          boost::asio::async_write(*stream_,
                                   boost::asio::buffer(response), yield);
        }
      }));

  bool done = false;

  using HC = HerkuleXConstants;

  boost::asio::spawn(service_, ErrorWrap([&](boost::asio::yield_context yield) {
        herkulex_.AsyncStart(yield);

        auto response = herkulex_.MemRead(herkulex_.RAM_READ, 0xfd, 0x35, 1,
                                          yield);
        BOOST_CHECK_EQUAL(response.register_start, 0x35);
        BOOST_CHECK_EQUAL(response.length, 1);
        BOOST_CHECK_EQUAL(response.register_data, std::string("\x01"));
        BOOST_CHECK_EQUAL(response.reg48, 0);
        BOOST_CHECK_EQUAL(response.reg49, 0x42);
        BOOST_CHECK_EQUAL(response.inposition, true);

        int value = herkulex_.RamRead(0xfd, HC::cal_diff(), yield);
        BOOST_CHECK_EQUAL(value, 1);

        done = true;
      }));

  service_.run();

  BOOST_CHECK_EQUAL(done, true);
}
