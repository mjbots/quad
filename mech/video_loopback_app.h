// Copyright 2015 Mikhail Afanasyev.  All rights reserved.
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

#include "base/component_archives.h"
#include "base/fail.h"
#include "base/logging.h"

#include "camera_driver.h"
#include "gst_main_loop.h"
#include "mcast_video_link.h"
#include "rtsp_server.h"
#include "video_display.h"

namespace mjmech {
namespace mech {

class VideoLoopbackApp : boost::noncopyable {
 public:
  template <typename Context>
    VideoLoopbackApp(Context& context)
    : service_(context.service) {
    m_.gst_main.reset(new GstMainLoop(context));
    m_.camera.reset(new CameraDriver(context));
    m_.video_link_tx.reset(new McastVideoLinkTransmitter(context));
    m_.video_link_rx.reset(new McastVideoLinkReceiver(context));
    m_.display.reset(new VideoDisplay(context));

    m_.gst_main->ready_signal()->connect(
        std::bind(&CameraDriver::HandleGstReady, m_.camera.get(),
                  std::placeholders::_1));
    m_.camera->AddFrameConsumer(m_.video_link_tx->get_frame_consumer());

    m_.gst_main->ready_signal()->connect(
        std::bind(&VideoDisplay::HandleGstReady, m_.display.get(),
                  std::placeholders::_1));

    m_.video_link_rx->frame_ready_signal()->connect(
      std::bind(&VideoDisplay::HandleIncomingFrame, m_.display.get(),
                std::placeholders::_1));

    m_.display->stats_signal()->connect(
       std::bind(&VideoLoopbackApp::HandleStats, this,
                 std::placeholders::_1));

    const std::string kAddr = "127.0.0.1:12542";
    m_.video_link_tx->parameters()->link.dest = kAddr;
    m_.video_link_rx->parameters()->link.source = kAddr;
  }

  void AsyncStart(base::ErrorHandler handler) {
    parameters_.children.Start(handler);
  }

  struct Members {
    std::unique_ptr<GstMainLoop> gst_main;
    std::unique_ptr<CameraDriver> camera;
    std::unique_ptr<VideoDisplay> display;
    std::unique_ptr<McastVideoLinkTransmitter> video_link_tx;
    std::unique_ptr<McastVideoLinkReceiver> video_link_rx;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(gst_main));
      a->Visit(MJ_NVP(camera));
      a->Visit(MJ_NVP(display));
      a->Visit(MJ_NVP(video_link_tx));
      a->Visit(MJ_NVP(video_link_rx));
    }
  };

  struct Parameters {
    base::ComponentParameters<Members> children;

    // if non-zero, exit after that many stats messages are received
    int max_stats = 0;
    // if True, crash when stats do indicate the video is not working.
    bool require_stats_good = false;

    template <typename Archive>
    void Serialize(Archive* a) {
      children.Serialize(a);
      a->Visit(MJ_NVP(max_stats));
      a->Visit(MJ_NVP(require_stats_good));
    }

    Parameters(Members* m) : children(m) { }
  };

  Parameters* parameters() { return &parameters_; }

 private:
  void HandleStats(const VideoDisplay::Stats* stats) {
    stats_count_++;
    if (parameters_.require_stats_good) {
      std::ostringstream errors;
      if (stats->raw_frames < 5) {
        errors << "not enough raw frames; ";
      }
      if (stats->h264_frames < 5) {
        errors << "not enough h264 frames; ";
      }
      if (stats->decoded_frames < 5) {
        errors << "not enough decoded frames; ";
      }
      if (errors.tellp()) {
        if (stats_count_ <= 1) {
          std::cerr << "First stat report was bad, hope next one is better: "
                    << errors.str() << "\n";
        } else {
          base::Fail("status report had errors:\n " + errors.str());
        };
      }
    }
    if (parameters_.max_stats && parameters_.max_stats <= stats_count_) {
      log_.notice("Got required number of stats, quitting");
      service_.stop();
    }
  }

  boost::asio::io_service& service_;
  Members m_;
  Parameters parameters_{&m_};
  int stats_count_ = 0;
  base::LogRef log_ = base::GetLogInstance("video_loopback_app");
};

}
}
