// Copyright 2014-2015 Josh Pieper, jjp@pobox.com.  All rights reserved.
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

#include <cmath>
#include <limits>

#include <boost/asio/spawn.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <boost/format.hpp>
#include <boost/system/system_error.hpp>

namespace mjmech {
namespace base {
const double kNaN = std::numeric_limits<double>::signaling_NaN();

inline double Degrees(double radians) {
  return 180.0 * radians / M_PI;
}

inline double Radians(double degrees) {
  return M_PI * degrees / 180.0;
}

inline double GetSign(double value) {
  if (value < 0.0) { return -1.0; }
  else if (value > 0.0) { return 1.0; }
  else { return 0.0; }
}

/// The following routine can be used to wrap coroutines such that
/// boost::system_error information is captured.  The default
/// boost::exception_ptr ignores this exception, making it challenging
/// to even report what happened.
template <typename Coroutine>
auto ErrorWrap(Coroutine coro) {
  return [=](boost::asio::yield_context yield) {
    try {
      return coro(yield);
    } catch (boost::system::system_error& e) {
      std::throw_with_nested(
          std::runtime_error(
              (boost::format("system_error: %s: %s") %
               e.what() % e.code()).str()));
    } catch (std::runtime_error& e) {
      std::throw_with_nested(std::runtime_error(e.what()));
    }
  };
}

inline boost::posix_time::time_duration
ConvertSecondsToDuration(double time_s) {
  const int64_t int_time = static_cast<int64_t>(time_s);
  const int64_t counts =
      static_cast<int64_t>(
          (time_s - static_cast<double>(int_time)) *
          boost::posix_time::time_duration::ticks_per_second());
  return boost::posix_time::seconds(static_cast<int>(time_s)) +
      boost::posix_time::time_duration(0, 0, 0, counts);
}

inline int64_t
ConvertPtimeToMicroseconds(boost::posix_time::ptime time) {
  if (time.is_pos_infinity()) {
    return std::numeric_limits<int64_t>::max();
  } else if (time.is_neg_infinity()) {
    return std::numeric_limits<int64_t>::min();
  } else if (time.is_special()) {
    return 0;
  }

  const boost::posix_time::ptime epoch(
      boost::gregorian::date(1970, boost::gregorian::Jan, 1));
  return (time - epoch).total_microseconds();
}

inline boost::posix_time::ptime
ConvertMicrosecondsToPtime(int64_t value) {
  if (value == std::numeric_limits<int64_t>::max()) {
    return boost::posix_time::pos_infin;
  } else if (value == std::numeric_limits<int64_t>::min()) {
    return boost::posix_time::neg_infin;
  } else if (value == 0) {
    return boost::posix_time::ptime();
  }

  const boost::posix_time::ptime epoch(
      boost::gregorian::date(1970, boost::gregorian::Jan, 1));
  return epoch + boost::posix_time::time_duration(
      0, 0, 0,
      value *
      boost::posix_time::time_duration::ticks_per_second() / 1000000);
}
}
}
