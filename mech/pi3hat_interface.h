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

#include "mjlib/multiplex/asio_client.h"

#include "mech/imu_client.h"
#include "mech/rf_client.h"

namespace mjmech {
namespace mech {

/// The pi3hat has multiple functions.  This is just an interface that
/// combines all of them for use in an mjlib::io::Selector.
class Pi3hatInterface : public ImuClient,
                        public RfClient,
                        public mjlib::multiplex::AsioClient {
 public:
  ~Pi3hatInterface() override {}

  virtual void Cycle(
      AttitudeData*,
      const std::vector<IdRequest>&, Reply*,
      mjlib::io::ErrorCallback callback) = 0;
};

}
}
