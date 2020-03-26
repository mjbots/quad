// Copyright 2014-2015 Mikhail Afanasyev.
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

#include <boost/asio/executor.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <boost/signals2/signal.hpp>

#include "mjlib/base/visitor.h"
#include "mjlib/io/async_types.h"

#include "base/context.h"

#include "camera_driver.h"

namespace mjmech {
namespace mech {

class RtspServer : boost::noncopyable {
 public:
  RtspServer(base::Context& context)
      : RtspServer(context.executor) {
    // no stats -- we log to camera
  }

  RtspServer(const boost::asio::executor&);
  ~RtspServer();

  void AsyncStart(mjlib::io::ErrorCallback handler);

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

  void HandleGstReady(GstMainLoopRef&);

 private:
  Parameters parameters_;

  class Impl;
  class FrameConsumerImpl;
  std::shared_ptr<Impl> impl_;
  std::shared_ptr<FrameConsumerImpl> frame_consumer_impl_;
};

}
}
