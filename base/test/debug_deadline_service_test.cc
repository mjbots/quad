// Copyright 2016 Josh Pieper, jjp@pobox.com.  All rights reserved.
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

#include "base/debug_deadline_service.h"

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/test/auto_unit_test.hpp>

#include "base/deadline_timer.h"
#include "base/now.h"

using namespace mjmech;

BOOST_AUTO_TEST_CASE(BasicDebugDeadlineServiceTest) {
  boost::asio::io_service service;
  auto debug_service = base::DebugDeadlineService::Install(service);

  auto start = boost::posix_time::microsec_clock::universal_time();
  debug_service->SetTime(start);

  BOOST_CHECK_EQUAL(base::Now(service), start);

  base::DeadlineTimer timer(service);
  timer.expires_from_now(boost::posix_time::seconds(100));
  BOOST_CHECK_EQUAL(timer.expires_from_now().total_microseconds(),
                    boost::posix_time::seconds(100).total_microseconds());

  auto next = start + boost::posix_time::seconds(300);
  timer.expires_at(next);
  BOOST_CHECK_EQUAL(timer.expires_at(), next);
  BOOST_CHECK_EQUAL(timer.expires_from_now(),
                    boost::posix_time::seconds(300));

  int count = 0;
  timer.async_wait([&](boost::system::error_code ec) {
      BOOST_CHECK(!ec);
      count++;
    });

  service.poll(); service.reset();
  BOOST_CHECK_EQUAL(count, 0);

  debug_service->SetTime(start + boost::posix_time::seconds(299));
  service.poll(); service.reset();
  BOOST_CHECK_EQUAL(count, 0);

  debug_service->SetTime(start + boost::posix_time::seconds(301));
  service.poll(); service.reset();
  BOOST_CHECK_EQUAL(count, 1);

  timer.expires_from_now(boost::posix_time::seconds(200));
  int count2 = 0;
  timer.async_wait([&](boost::system::error_code ec) {
      BOOST_CHECK(ec == boost::asio::error::operation_aborted);
      count2++;
    });

  service.poll(); service.reset();
  BOOST_CHECK_EQUAL(count2, 0);
  timer.cancel();
  service.poll(); service.reset();
  BOOST_CHECK_EQUAL(count2, 1);
}
