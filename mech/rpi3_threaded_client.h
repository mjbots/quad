// Copyright 2019 Josh Pieper, jjp@pobox.com.
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

#include <memory>
#include <string>
#include <vector>

#include <boost/asio/executor.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>

#include "mjlib/base/visitor.h"
#include "mjlib/io/async_types.h"
#include "mjlib/io/async_stream.h"
#include "mjlib/multiplex/register.h"

namespace mjmech {
namespace mech {

/// This client uses a background thread, in which blocking reads and
/// writes are made using raw access to the RPI3s system memory.
///
/// This allows for less overall latency than the asio approach,
/// especially when multiple back to back queries are made such as
/// when commanding multiple devices.
class Rpi3ThreadedClient {
 public:
  struct Options {
    int baud_rate = 3000000;

    double query_timeout_s = 0.0005;

    // If set to a non-negative number, bind the time sensitive thread
    // to the given CPU.
    int cpu_affinity = -1;

    // If true, then print to stdout a hex dump of all received
    // packets which fail with a checksum error.
    bool debug_checksum_errors = false;
  };

  Rpi3ThreadedClient(const boost::asio::executor&, const Options&);
  ~Rpi3ThreadedClient();

  struct SingleRequest {
    uint8_t id = 0;
    mjlib::multiplex::RegisterRequest request;
  };

  struct Request {
    std::vector<SingleRequest> requests;
  };

  struct SingleReply {
    uint8_t id = 0;
    mjlib::multiplex::RegisterReply reply;
  };

  struct Reply {
    std::vector<SingleReply> replies;
  };

  /// Request a request be made to one or more servos (and optionally
  /// have a reply sent back).
  void AsyncRegister(const Request*, Reply*, mjlib::io::ErrorCallback);

  struct Stats {
    uint64_t checksum_errors = 0;
    uint64_t timeouts = 0;
    uint64_t malformed = 0;
    uint64_t serial_errors = 0;
    uint64_t extra_found = 0;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(checksum_errors));
      a->Visit(MJ_NVP(timeouts));
      a->Visit(MJ_NVP(malformed));
      a->Visit(MJ_NVP(serial_errors));
      a->Visit(MJ_NVP(extra_found));
    }
  };
  Stats stats() const;

  struct TunnelOptions {
    // Poll this often for data to be received.
    boost::posix_time::time_duration poll_rate =
        boost::posix_time::milliseconds(10);

    TunnelOptions() {}
  };

  mjlib::io::SharedStream MakeTunnel(
      uint8_t id,
      uint32_t channel,
      const TunnelOptions& options);

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}
}
