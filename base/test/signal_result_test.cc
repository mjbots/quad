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

#include "base/signal_result.h"

#include <boost/test/auto_unit_test.hpp>

using namespace mjmech::base;

BOOST_AUTO_TEST_CASE(TestSignalResultCallback1) {
  boost::asio::io_context context;
  auto executor = context.get_executor();

  boost::signals2::signal<void (const int*)> signal;
  int value = 0;
  bool done = false;
  int count = 0;

  SignalResult::Wait(executor, &signal, 1.0,
                     [&](const boost::system::error_code& ec,
                         int value_in) {
                       BOOST_CHECK(!ec);
                       done = true;
                       value = value_in;
                       count++;
                     });

  // We haven't even started the io_context yet, so nothing should be
  // running.
  BOOST_CHECK_EQUAL(done, false);
  BOOST_CHECK_EQUAL(value, 0);

  mjlib::io::DeadlineTimer timer1(executor);
  timer1.expires_from_now(boost::posix_time::milliseconds(500));
  timer1.async_wait([&](const boost::system::error_code& ec) {
      BOOST_CHECK(!ec);
      BOOST_CHECK_EQUAL(done, false);

      int send = 5;
      signal(&send);

      BOOST_CHECK_EQUAL(done, true);
      BOOST_CHECK_EQUAL(value, 5);
      BOOST_CHECK_EQUAL(count, 1);
    });

  context.run();
  BOOST_CHECK_EQUAL(done, true);
  BOOST_CHECK_EQUAL(count, 1);
}

BOOST_AUTO_TEST_CASE(TestSignalResultCallback2) {
  boost::asio::io_context context;
  auto executor = context.get_executor();

  boost::signals2::signal<void (const int*)> signal;
  bool done = false;
  int count = 0;

  SignalResult::Wait(executor, &signal, 1.0,
                     [&](const boost::system::error_code& ec,
                         int) {
                       BOOST_CHECK(ec);
                       BOOST_CHECK_EQUAL(ec, boost::asio::error::operation_aborted);
                       done = true;
                       count++;
                     });

  mjlib::io::DeadlineTimer timer1(executor);
  timer1.expires_from_now(boost::posix_time::milliseconds(1500));
  timer1.async_wait([&](const boost::system::error_code& ec) {
      BOOST_CHECK(!ec);
      BOOST_CHECK_EQUAL(done, true);
      BOOST_CHECK_EQUAL(count, 1);
    });

  context.run();
  BOOST_CHECK_EQUAL(count, 1);
}
