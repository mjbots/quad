// Copyright 2019-2020 Josh Pieper, jjp@pobox.com.
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

#include "mjlib/base/visitor.h"
#include "mjlib/multiplex/asio_client.h"
#include "mjlib/multiplex/register.h"

namespace mjmech {
namespace mech {

class Rpi3HatSpidev : public mjlib::multiplex::AsioClient {
 public:
  struct Options {
    std::string spidev1 = "/dev/spidev0.0";
    std::string spidev2 = "/dev/spidev0.1";

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(spidev1));
      a->Visit(MJ_NVP(spidev2));
    }
  };

  Rpi3HatSpidev(const boost::asio::executor&, const Options&);
  ~Rpi3HatSpidev();

  void AsyncStart(mjlib::io::ErrorCallback);

  /// Request a request be made to one or more servos (and optionally
  /// have a reply sent back).
  void AsyncRegister(const IdRequest&, SingleReply*,
                     mjlib::io::ErrorCallback) override;

  void AsyncRegisterMultiple(const std::vector<IdRequest>&,
                             Reply*,
                             mjlib::io::ErrorCallback) override;

  mjlib::io::SharedStream MakeTunnel(
      uint8_t id,
      uint32_t channel,
      const TunnelOptions& options) override;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}
}
