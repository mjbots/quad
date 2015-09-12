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

#include "comm_factory.h"

#include <functional>

#include <boost/property_tree/json_parser.hpp>

#include "property_tree_archive.h"

#include <boost/test/auto_unit_test.hpp>

namespace pl = std::placeholders;
using namespace legtool;
namespace pt = boost::property_tree;

BOOST_AUTO_TEST_CASE(StreamFactoryTest) {
  typedef StreamFactory<StdioGenerator,
                        SerialPortGenerator,
                        TcpClientGenerator> Factory;

  boost::asio::io_service service;
  Factory factory(service);
  Factory::Parameters params;
  params.type = "stdio";

  std::ostringstream ostr;
  pt::write_json(ostr,
                 PropertyTreeWriteArchive().Accept(&params).tree());
  std::string expected = R"XX({
    "type": "stdio",
    "stdio_debug": "false",
    "stdio":
    {
        "in": "0",
        "out": "1"
    },
    "serial":
    {
        "serial_port": "",
        "baud_rate": "115200",
        "parity": "n",
        "data_bits": "8"
    },
    "tcp":
    {
        "host": "",
        "port": "0"
    }
}
)XX";
  BOOST_CHECK_EQUAL(ostr.str(), expected);
  factory.AsyncCreate(params, [](boost::system::error_code,
                                 std::shared_ptr<AsyncStream>){});
}

namespace {
struct TestHandler {
  void Handle(const boost::system::error_code& ec_in,
              std::size_t written_in) {
    count++;
    ec = ec_in;
    written = written_in;
  }

  int count = 0;
  boost::system::error_code ec;
  std::size_t written = 0;
};
}

BOOST_AUTO_TEST_CASE(PipeGeneratorTest) {
  boost::asio::io_service service;

  PipeGenerator dut;

  auto stream1a = dut.GetStream(
      service, "test1", PipeGenerator::Mode::kDirectionA);
  auto stream1b = dut.GetStream(
      service, "test1", PipeGenerator::Mode::kDirectionB);

  auto stream2a = dut.GetStream(
      service, "testB", PipeGenerator::Mode::kDirectionB);
  auto stream2b = dut.GetStream(
      service, "testB", PipeGenerator::Mode::kDirectionA);

  TestHandler write_handler;
  stream1a->async_write_some(
      boost::asio::buffer("test1"),
      std::bind(&TestHandler::Handle, &write_handler, pl::_1, pl::_2));
  service.poll(); service.reset();

  BOOST_CHECK_EQUAL(write_handler.count, 0);

  // Kick off a fractional read, which should get both to complete.
  char buffer[3] = {};
  TestHandler read_handler;
  stream1b->async_read_some(
      boost::asio::buffer(buffer),
      std::bind(&TestHandler::Handle, &read_handler, pl::_1, pl::_2));

  // Nothing should have fired yet, since we haven't polled.
  BOOST_CHECK_EQUAL(read_handler.count, 0);
  BOOST_CHECK_EQUAL(write_handler.count, 0);

  service.poll(); service.reset();

  BOOST_CHECK_EQUAL(read_handler.count, 1);
  BOOST_CHECK_EQUAL(read_handler.written, 3);
  BOOST_CHECK_EQUAL(read_handler.ec, boost::system::error_code());

  BOOST_CHECK_EQUAL(write_handler.count, 1);
  BOOST_CHECK_EQUAL(write_handler.written, 3);
  BOOST_CHECK_EQUAL(write_handler.ec, boost::system::error_code());

  BOOST_CHECK_EQUAL(buffer[0], 't');
  BOOST_CHECK_EQUAL(buffer[1], 'e');
  BOOST_CHECK_EQUAL(buffer[2], 's');

  // Now kick off a read first, with the write coming later.
  char buffer2[100] = {};
  stream1a->async_read_some(
      boost::asio::buffer(buffer2),
      std::bind(&TestHandler::Handle, &read_handler, pl::_1, pl::_2));

  service.poll(); service.reset();

  BOOST_CHECK_EQUAL(read_handler.count, 1);
  BOOST_CHECK_EQUAL(write_handler.count, 1);

  // And make a write which is smaller than the read.

  stream1b->async_write_some(
      boost::asio::buffer("smaller"),
      std::bind(&TestHandler::Handle, &write_handler, pl::_1, pl::_2));

  BOOST_CHECK_EQUAL(read_handler.count, 1);
  BOOST_CHECK_EQUAL(write_handler.count, 1);

  service.poll(); service.reset();

  BOOST_CHECK_EQUAL(read_handler.count, 2);
  BOOST_CHECK_EQUAL(read_handler.written, 8);
  BOOST_CHECK_EQUAL(read_handler.ec, boost::system::error_code());

  BOOST_CHECK_EQUAL(write_handler.count, 2);
  BOOST_CHECK_EQUAL(write_handler.written, 8);
  BOOST_CHECK_EQUAL(write_handler.ec, boost::system::error_code());

  BOOST_CHECK_EQUAL(std::string(buffer2, 7), "smaller");
}
