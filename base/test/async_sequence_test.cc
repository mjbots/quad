// Copyright 2019 Josh Pieper, jjp@pobox.com.  All rights reserved.
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

#include "base/async_sequence.h"

#include <boost/test/auto_unit_test.hpp>

#include "mjlib/base/fail.h"

using namespace mjmech::base;

BOOST_AUTO_TEST_CASE(BasicAsyncSequence) {
  boost::asio::io_service service;

  bool first_called = false;
  auto first_op = [&](mjlib::io::ErrorCallback cbk) {
    first_called = true;
    cbk({});
  };

  bool second_called = false;
  auto second_op = [&](mjlib::io::ErrorCallback cbk) {
    second_called = true;
    cbk({});
  };

  bool final_called = false;
  auto shared_bool = std::make_shared<bool>();
  auto weak_bool = std::weak_ptr<bool>(shared_bool);

  AsyncSequence(service)
      .Add(first_op)
      .Add(second_op)
      .Start([shared_bool, &final_called](const mjlib::base::error_code& ec) {
          mjlib::base::FailIf(ec);
          final_called = true;
        });

  shared_bool.reset();

  BOOST_TEST(first_called == false);
  BOOST_TEST(second_called == false);
  BOOST_TEST(final_called == false);

  service.poll_one();
  service.reset();

  BOOST_TEST(first_called == true);
  BOOST_TEST(second_called == false);
  BOOST_TEST(final_called == false);

  service.poll_one();
  service.reset();

  BOOST_TEST(first_called == true);
  BOOST_TEST(second_called == true);
  BOOST_TEST(final_called == false);

  BOOST_TEST(weak_bool.lock());

  service.poll_one();
  service.reset();

  BOOST_TEST(first_called == true);
  BOOST_TEST(second_called == true);
  BOOST_TEST(final_called == true);

  // After the last callback is done, it should be destroyed and the
  // captured shared ptr should be gone.
  BOOST_TEST(!weak_bool.lock());
}

BOOST_AUTO_TEST_CASE(ErrorAsyncSequence) {
  boost::asio::io_service service;

  bool first_called = false;
  auto first_op = [&](mjlib::io::ErrorCallback cbk) {
    first_called = true;
    cbk(boost::asio::error::operation_aborted);
  };

  bool second_called = false;
  auto second_op = [&](mjlib::io::ErrorCallback cbk) {
    second_called = true;
    cbk({});
  };

  bool final_called = false;
  auto shared_bool = std::make_shared<bool>();
  auto weak_bool = std::weak_ptr<bool>(shared_bool);

  AsyncSequence(service)
      .Add(first_op)
      .Add(second_op)
      .Start([shared_bool, &final_called](const mjlib::base::error_code& ec) {
          BOOST_TEST(ec == boost::asio::error::operation_aborted);
          final_called = true;
        });

  BOOST_TEST(first_called == false);
  BOOST_TEST(second_called == false);
  BOOST_TEST(final_called == false);

  service.poll_one();
  service.reset();

  // An error causes immediate invocation of the final result with no
  // additional polling required (and no other callbacks invoked).

  BOOST_TEST(first_called == true);
  BOOST_TEST(second_called == false);
  BOOST_TEST(final_called == true);
}
