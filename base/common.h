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

#include <boost/date_time/posix_time/posix_time_types.hpp>

namespace mjmech {
namespace base {
const double kNaN = std::numeric_limits<double>::signaling_NaN();
const double kPi = 3.14159265358979323846;

inline double Degrees(double radians) {
  return 180.0 * radians / kPi;
}

inline double Radians(double degrees) {
  return kPi * degrees / 180.0;
}

inline double GetSign(double value) {
  if (value < 0.0) { return -1.0; }
  else if (value > 0.0) { return 1.0; }
  else { return 0.0; }
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

inline double ConvertDurationToSeconds(
    boost::posix_time::time_duration duration) {
  return duration.total_microseconds() / 1e6;
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
