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

#include "base/signal_result.h"

#include <boost/test/auto_unit_test.hpp>

using namespace mjmech::base;

// BOOST_AUTO_TEST_CASE(TestSignalResultCoroutine1) {
//   boost::asio::io_service service;

//   boost::signals2::signal<void (const int*)> signal;

//   bool done = false;

//   boost::asio::spawn(service, [&](boost::asio::yield_context yield) {
//       int result = SignalResult::Wait(service, &signal, 1.0, yield);
//       BOOST_CHECK_EQUAL(result, 3);
//       done = true;
//     });

//   boost::asio::spawn(service, [&](boost::asio::yield_context yield) {
//       int result = 3;
//       BOOST_CHECK_EQUAL(done, false);
//       signal(&result);

//       // When a signal is emitted, the result should have been made
//       // available immediately.
//       BOOST_CHECK_EQUAL(done, true);
//     });

//   service.run();
// }

// BOOST_AUTO_TEST_CASE(TestSignalResultCoroutine2) {
//   boost::asio::io_service service;

//   boost::signals2::signal<void (const int*)> signal;

//   bool done = false;
//   bool timeout = false;

//   boost::asio::spawn(service, [&](boost::asio::yield_context yield) {
//       try {
//         SignalResult::Wait(service, &signal, 1.0, yield);
//         BOOST_CHECK(false);
//         done = true;
//       } catch (TimeoutError& e) {
//         timeout = true;
//       }
//     });

//   boost::asio::spawn(service, [&](boost::asio::yield_context yield) {
//       BOOST_CHECK_EQUAL(done, false);
//       BOOST_CHECK_EQUAL(timeout, false);

//       DeadlineTimer timer(service);
//       timer.expires_from_now(boost::posix_time::milliseconds(900));
//       timer.async_wait(yield);
//       BOOST_CHECK_EQUAL(timeout, false);

//       timer.expires_from_now(boost::posix_time::milliseconds(200));
//       timer.async_wait(yield);
//       BOOST_CHECK_EQUAL(timeout, true);
//     });


//   service.run();
//   BOOST_CHECK_EQUAL(done, false);
//   BOOST_CHECK_EQUAL(timeout, true);
// }

BOOST_AUTO_TEST_CASE(TestSignalResultCallback1) {
  boost::asio::io_service service;

  boost::signals2::signal<void (const int*)> signal;
  int value = 0;
  bool done = false;
  int count = 0;

  SignalResult::Wait(service, &signal, 1.0,
                     [&](const boost::system::error_code& ec,
                         int value_in) {
                       BOOST_CHECK(!ec);
                       done = true;
                       value = value_in;
                       count++;
                     });

  // We haven't even started the io_service yet, so nothing should be
  // running.
  BOOST_CHECK_EQUAL(done, false);
  BOOST_CHECK_EQUAL(value, 0);

  DeadlineTimer timer1(service);
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

  service.run();
  BOOST_CHECK_EQUAL(done, true);
  BOOST_CHECK_EQUAL(count, 1);
}

BOOST_AUTO_TEST_CASE(TestSignalResultCallback2) {
  boost::asio::io_service service;

  boost::signals2::signal<void (const int*)> signal;
  bool done = false;
  int count = 0;

  SignalResult::Wait(service, &signal, 1.0,
                     [&](const boost::system::error_code& ec,
                         int value_in) {
                       BOOST_CHECK(ec);
                       BOOST_CHECK_EQUAL(ec, boost::asio::error::operation_aborted);
                       done = true;
                       count++;
                     });

  DeadlineTimer timer1(service);
  timer1.expires_from_now(boost::posix_time::milliseconds(1500));
  timer1.async_wait([&](const boost::system::error_code& ec) {
      BOOST_CHECK(!ec);
      BOOST_CHECK_EQUAL(done, true);
      BOOST_CHECK_EQUAL(count, 1);
    });

  service.run();
  BOOST_CHECK_EQUAL(count, 1);
}
