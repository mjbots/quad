// Copyright 2020 Josh Pieper, jjp@pobox.com.
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

#include "base/timestamped_log.h"

#include <fmt/format.h>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/filesystem.hpp>


namespace mjmech {
namespace base {

void OpenMaybeTimestampedLog(mjlib::telemetry::FileWriter* writer,
                             std::string_view filename,
                             TimestampMode mode) {
  // Make sure that the log file has a date and timestamp somewhere
  // in the name.
  namespace fs = boost::filesystem;
  fs::path log_file_path{std::string(filename)};
  std::string extension = log_file_path.extension().native();

  const auto now = boost::posix_time::microsec_clock::universal_time();
  std::string datestamp =
      fmt::format("{}-{:02d}{:02d}{:02d}",
                  to_iso_string(now.date()),
                  now.time_of_day().hours(),
                  now.time_of_day().minutes(),
                  now.time_of_day().seconds());

  const std::string stem = log_file_path.stem().native();

  fs::path stamped_path = log_file_path.parent_path() /
      fmt::format("{}-{}{}", stem, datestamp, extension);

  switch (mode) {
    case kShort: {
      writer->Open(filename);
      break;
    }
    case kTimestamped: {
      writer->Open(stamped_path.native());
      break;
    }
  }
}

}
}
