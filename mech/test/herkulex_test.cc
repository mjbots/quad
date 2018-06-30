// Copyright 2015-2016 Josh Pieper, jjp@pobox.com.  All rights reserved.
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

#include "mech/herkulex.h"

#include <functional>

#include <boost/test/auto_unit_test.hpp>

#include "base/comm_factory_generators.h"
#include "base/concrete_comm_factory.h"
#include "base/error_code.h"
#include "base/fail_functor.h"

using namespace mjmech::base;
using namespace mjmech::mech;
namespace pl = std::placeholders;

namespace {
class Fixture {
 public:
  Fixture() {
    SetOption(herkulex_.options(), "stream.type", "pipe");
    SetOption(herkulex_.options(), "stream.pipe.key", "test");

    stream_ = factory_.pipe_generator()->GetStream(
        service_, "test", PipeGenerator::Mode::kDirectionB);

    StartRead();
  }

  void StartRead() {
    stream_->async_read_some(
        boost::asio::buffer(buf_),
        std::bind(&Fixture::HandleRead, this, pl::_1, pl::_2));
  }

  void HandleRead(ErrorCode ec, size_t bytes_read) {
    FailIf(ec);

    std::ostream ostr(&streambuf_);
    ostr.write(buf_, bytes_read);

    bool gotit = true;
    data_received_(&gotit);

    StartRead();
  }

  void Poll() {
    service_.poll();
    service_.reset();
  }

  boost::asio::io_service service_;
  typedef ConcreteStreamFactory Factory;
  Factory factory_{service_};
  typedef HerkuleX Servo;
  Servo herkulex_{service_, factory_};
  SharedStream stream_;
  boost::asio::streambuf streambuf_;
  boost::signals2::signal<void (const bool*)> data_received_;

  char buf_[4096] = {};
};
}

BOOST_FIXTURE_TEST_CASE(HerkuleXTest, Fixture) {
  herkulex_.AsyncStart(FailIf);

  Poll();

  Servo::Packet packet;
  packet.servo = 0xfd;
  packet.command = Servo::STAT;
  packet.data = "";
  herkulex_.SendPacket(packet, FailIf);

  Poll();

  // The packet should have ended up in our streambuf now.
  {
    std::ostringstream ostr;
    ostr << &streambuf_;
    std::string data = ostr.str();
    BOOST_CHECK_EQUAL(data.size(), 7);
    BOOST_CHECK_EQUAL(data, "\xff\xff\x07\xfd\x07\xfc\x02");
  }

  boost::optional<Servo::Packet> listen;
  herkulex_.ReceivePacket([&](const ErrorCode& ec, const auto& result) {
      FailIf(ec);
      listen = result;
    });

  Poll();

  BOOST_CHECK(!listen);

  // Send out a response over the wire.
  std::string response("\xff\xff\x09\xfd\x47\xf2\x0c\x01\x40");
  boost::asio::async_write(*stream_,  boost::asio::buffer(response), FailFunctor());

  Poll();

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
}

namespace {
class Responder : public Fixture {
 public:
  void Start() {
    if (count_ >= 2) { return; }

    SignalResult::Wait(service_, &data_received_, 1.0,
                       std::bind(&Responder::Handle, this, pl::_1));
  }

  void Handle(const ErrorCode& ec) {
    FailIf(ec);

    if (streambuf_.size() < 9) {
      Start();
      return;
    }


    count_++;

    std::istream istr(&streambuf_);
    char received[9] = {};
    istr.read(received, sizeof(received));
    BOOST_REQUIRE_EQUAL(istr.gcount(), sizeof(received));
    BOOST_CHECK_EQUAL(std::string(received, 9),
                      "\xff\xff\x09\xfd\x04\xc4\x3a\x35\x01");

    boost::asio::async_write(*stream_, boost::asio::buffer(response_),
                             std::bind(&Responder::HandleWrite, this, pl::_1));
  }

  void HandleWrite(const ErrorCode& ec) {
    FailIf(ec);

    Start();
  }


  int count_ = 0;
  std::string response_{
    "\xff\xff\x0c\xfd\x44\xc2\x3c\x35\x01\x01\x00\x42", 12};
};
}

BOOST_FIXTURE_TEST_CASE(HerkuleXMemRead, Responder) {
  // Listen for the request, and reply with a response when necessary.
  Start();

  bool done = false;

  using HC = HerkuleXConstants;

  herkulex_.AsyncStart(FailFunctor());

  bool read = false;
  herkulex_.MemRead(
      herkulex_.RAM_READ, 0xfd, 0x35, 1,
      [&](const ErrorCode& ec, const auto& response) {
        BOOST_CHECK_EQUAL(response.register_start, 0x35);
        BOOST_CHECK_EQUAL(response.length, 1);
        BOOST_CHECK_EQUAL(response.register_data, std::string("\x01"));
        BOOST_CHECK_EQUAL(response.reg48, 0);
        BOOST_CHECK_EQUAL(response.reg49, 0x42);
        BOOST_CHECK_EQUAL(response.inposition, true);
        read = true;
      });

  Poll();
  BOOST_ASSERT(read);
  read = false;


  herkulex_.RamRead(
      0xfd, HC::cal_diff(),
      [&](const ErrorCode& ec, int value) {
        FailIf(ec);
        BOOST_CHECK_EQUAL(value, 1);
        read = true;
      });

  Poll();
  BOOST_ASSERT(read);
  BOOST_CHECK_EQUAL(done, true);
}
