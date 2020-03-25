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

#pragma once

#include <boost/program_options.hpp>

#include "mjlib/base/visitor.h"
#include "mjlib/multiplex/asio_client.h"

namespace mjmech {
namespace mech {

class MultiplexClient {
 public:
  MultiplexClient(const boost::asio::executor&);
  ~MultiplexClient();

  struct Parameters {
    int spi_speed_hz = 10000000;
    int cpu_affinity = 3;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(spi_speed_hz));
      a->Visit(MJ_NVP(cpu_affinity));
    }
  };

  Parameters* parameters();
  boost::program_options::options_description* options();
  void AsyncStart(mjlib::io::ErrorCallback);

  using Client = mjlib::multiplex::AsioClient;
  using ClientCallback = fu2::unique_function<
    void (const mjlib::base::error_code&, Client*)>;
  void RequestClient(ClientCallback);

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}
}
