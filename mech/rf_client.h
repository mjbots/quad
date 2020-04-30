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

#pragma once

#include <boost/date_time/posix_time/posix_time_types.hpp>

#include "mjlib/io/async_types.h"

namespace mjmech {
namespace mech {

class RfClient {
 public:
  virtual ~RfClient() {}

  /// Wait for one or more receive slots to be updated.
  ///
  /// @param bitfield will be set with a 1 for each slot which has
  /// been updated.
  virtual void AsyncWaitForSlot(
      int* remote, uint16_t* bitfield, mjlib::io::ErrorCallback) = 0;

  struct Slot {
    /// The timestamp is only valid for rx, and is ignored for tx.
    boost::posix_time::ptime timestamp;

    /// The priority is only valid for tx, and will be left
    /// unspecified for rx.
    uint32_t priority = 0;

    uint8_t size = 0;
    char data[16] = {};
  };

  /// Return the current value of the given receive slot.
  virtual Slot rx_slot(int remote, int slot_idx) = 0;

  /// Set the given transmit slot.
  virtual void tx_slot(int remote, int slot_idx, const Slot&) = 0;

  /// Retrieve the currently selected transmit slot.
  virtual Slot tx_slot(int remote, int slot_idx) = 0;
};

}
}
