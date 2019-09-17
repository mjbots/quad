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

#include "base/telemetry_log.h"

#include <boost/asio/deadline_timer.hpp>
#include <boost/asio/io_service.hpp>
#include <boost/asio/posix/stream_descriptor.hpp>
#include <boost/asio/read.hpp>
#include <boost/asio/read_until.hpp>
#include <boost/asio/streambuf.hpp>
#include <boost/test/auto_unit_test.hpp>

using namespace mjmech::base;

BOOST_AUTO_TEST_CASE(TelemetryLogBasicTest) {
  boost::asio::io_context service;
  TelemetryLog log;

  int pipefd[2] = {};
  int result = ::pipe(pipefd);
  BOOST_REQUIRE(result >= 0);

  boost::asio::posix::stream_descriptor in(service, pipefd[0]);
  log.Open(pipefd[1]);

  log.WriteBlock(
      mjlib::telemetry::TelemetryFormat::BlockType::kBlockData,
      "invalid block\n");
  log.Flush();

  boost::asio::streambuf streambuf;
  boost::asio::async_read_until(
      in, streambuf, '\n',
      [&](boost::system::error_code ec, std::size_t){
        BOOST_REQUIRE(!ec);
        service.stop();
      });
  boost::asio::deadline_timer timer(service);
  timer.expires_from_now(boost::posix_time::milliseconds(500));
  timer.async_wait([&](boost::system::error_code){ service.stop(); });

  service.run();

  std::ostringstream ostr;
  ostr << &streambuf;
  BOOST_CHECK_EQUAL(ostr.str().size(), 28);
  mjlib::base::FastIStringStream istr(ostr.str());
  char header[9] = {};
  istr.read(mjlib::base::string_span(header, 8));
  BOOST_CHECK_EQUAL(std::string(header), "TLOG0002");

  mjlib::telemetry::TelemetryReadStream stream(istr);
  BOOST_CHECK_EQUAL(stream.Read<uint16_t>(), 2);
  BOOST_CHECK_EQUAL(stream.Read<uint32_t>(), 14);
  char buf[15] = {};
  istr.read(mjlib::base::string_span(buf, 14));
  BOOST_CHECK_EQUAL(std::string(buf), "invalid block\n");

  log.Close();
}
