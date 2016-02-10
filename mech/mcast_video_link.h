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

#include <memory>

#include <boost/asio/io_service.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <boost/signals2/signal.hpp>

#include "base/comm.h"
#include "base/context.h"
#include "base/visitor.h"

#include "base/udp_data_link.h"

#include "camera_driver.h"

namespace mjmech {
namespace mech {


/*
  This class implements "multicast video link" -- a communication
  protocol which sends UDP packets over highly-lossy links (20%+ losses).

  It is designed to operate using multicast packets over Wifi link (so native
  WiFi retransmission mechanisms are bypassed), but it may also be used in other
  configurations.

  The protocol assumes a stream of video packets; each packet is only useful
  if it has been received completely. The packets arrive at a uniform rate,
  but this rate is not known in advance.

  For the reference, at the default settings (3 mbit/sec), packets are 14-17K
  (regular), 25-38K (i-frame) and come at 24 fps.
*/


class McastVideoLinkTransmitter : boost::noncopyable {
 public:
  McastVideoLinkTransmitter(base::Context& context);
  ~McastVideoLinkTransmitter();

  void AsyncStart(base::ErrorHandler handler);

  // Get a frameconsumer interface. pointer has same lifetime
  // as this object.
  std::weak_ptr<CameraFrameConsumer> get_frame_consumer();

  struct Parameters {
    // How many times to repeat each packet. 0 disables data sending.
    int repeat_count = 2;

    // Minimal framerate (corresponds to maximum inter-frame interval).
    // The packets are scheduled to be sent at the video freamerate, but not
    // slower than this value.
    double min_fps = 20;

    base::UdpDataLink::Parameters link;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(repeat_count));
      a->Visit(MJ_NVP(min_fps));
      link.Serialize(a);
    }

    Parameters() {
      link.socket_params.default_port = 2180;
      link.dest = "239.89.108.10";
    }

  };

  Parameters* parameters() { return &parameters_; }

 private:
  Parameters parameters_;

  class Impl;
  class FrameConsumerImpl;

  std::shared_ptr<Impl> impl_;
  std::shared_ptr<FrameConsumerImpl> frame_consumer_impl_;
};

class McastVideoLinkReceiver : boost::noncopyable {
 public:
  McastVideoLinkReceiver(base::Context& context);
  ~McastVideoLinkReceiver();

  void AsyncStart(base::ErrorHandler handler);

  struct Parameters {
    base::UdpDataLink::Parameters link;

    template <typename Archive>
    void Serialize(Archive* a) {
      link.Serialize(a);
    }

    Parameters() {
      link.socket_params.default_port = 2180;
      link.source = "239.89.108.10";
    }

  };

  Parameters* parameters() { return &parameters_; }

  typedef boost::signals2::signal<
    void (std::shared_ptr<std::string>&)> FrameReadySignal;
  FrameReadySignal* frame_ready_signal() { return &frame_ready_signal_; };

 private:
  FrameReadySignal frame_ready_signal_;
  Parameters parameters_;

  class Impl;
  std::shared_ptr<Impl> impl_;
};

}
}
