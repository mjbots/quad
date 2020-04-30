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

#include "mjlib/io/async_stream.h"

#include "mech/rf_client.h"

namespace mjmech {
namespace mech {

/// An implementation of RfClient that uses the nrfusb.
class NrfusbClient : public RfClient {
 public:
  struct Options {
    uint32_t id0 = 5678;
    uint32_t id1 = 88754;

    Options() {}
  };
  NrfusbClient(mjlib::io::AsyncStream*, const Options& = Options());
  ~NrfusbClient() override;

  void AsyncWaitForSlot(
      int* remote, uint16_t* bitfield, mjlib::io::ErrorCallback) override;

  Slot rx_slot(int remote, int slot_idx) override;
  void tx_slot(int remote, int slot_idx, const Slot&) override;
  Slot tx_slot(int remote, int slot_idx) override;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}
}
