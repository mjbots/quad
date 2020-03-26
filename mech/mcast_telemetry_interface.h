// Copyright 2016 Josh Pieper, jjp@pobox.com.
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

#include <string>

#include <boost/date_time/posix_time/posix_time_types.hpp>

namespace mjmech {
namespace mech {

/// The multi-cast system can associate additional telemetry with
/// video frames as they are sent.  This interface provides a mechanism
/// to provide that additional telemetry. */
class McastTelemetryInterface {
 public:
  virtual ~McastTelemetryInterface() {}

  /// Include @p data with @p name in packets that are sent until @p
  /// expiration has passed, after which no data with this name will
  /// be included.  Any previous information associated with @p name
  /// is overwritten.
  virtual void SetTelemetry(const std::string& name,
                            const std::string& data,
                            boost::posix_time::ptime expiration) = 0;
};
}
}
