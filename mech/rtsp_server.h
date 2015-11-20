// Copyright 2014-2015 Mikhail Afanasyev.  All rights reserved.
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

#include <boost/asio/io_service.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <boost/signals2/signal.hpp>

#include "base/comm.h"
#include "base/visitor.h"

#include "camera_driver.h"

namespace mjmech {
namespace mech {

class RtspServer : boost::noncopyable {
 public:
  template <typename Context>
    RtspServer(Context& context)
    : RtspServer(context.service,
                   &context.telemetry_registry) {}

  template <typename TelemetryRegistry>
    RtspServer(boost::asio::io_service& service,
                 TelemetryRegistry* telemetry_registry)
    : RtspServer(service) {
    // no stats -- we log to camera
    //telemetry_registry->Register("rtsp_server_stats", &stats_signal_);
  }

  RtspServer(boost::asio::io_service&);
  ~RtspServer();

  void AsyncStart(base::ErrorHandler handler);

  struct Parameters {
    // set to 0 to dynamically assign
    int port = 8554;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(port));
    }
  };

  Parameters* parameters() { return &parameters_; }

  // Get a frameconsumer interface. pointer has same lifetime
  // as this object.
  std::weak_ptr<CameraFrameConsumer> get_frame_consumer();

 private:
  Parameters parameters_;

  class Impl;
  class FrameConsumerImpl;
  std::shared_ptr<Impl> impl_;
  std::shared_ptr<FrameConsumerImpl> frame_consumer_impl_;
};

}
}
