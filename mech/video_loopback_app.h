// Copyright 2020 Josh Pieper, jjp@pobox.com.
// Copyright 2015-2016 Mikhail Afanasyev.
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

#include <fmt/format.h>

#include "mjlib/io/now.h"

#include "base/component_archives.h"
#include "base/fail.h"
#include "base/logging.h"
#include "base/stringify.h"

#include "mech/camera_driver.h"
#include "mech/gst_main_loop.h"
#include "mech/mcast_video_link.h"
#include "mech/rtsp_server.h"
#include "mech/target_tracker.h"
#include "mech/video_display.h"

namespace mjmech {
namespace mech {

class VideoLoopbackApp : boost::noncopyable {
 public:
  template <typename Context>
    VideoLoopbackApp(Context& context)
    : executor_(context.executor) {
    m_.gst_main.reset(new GstMainLoop(context));
    m_.camera.reset(new CameraDriver(context));
    m_.video_link_tx.reset(new McastVideoLinkTransmitter(context));
    m_.video_link_rx.reset(new McastVideoLinkReceiver(context));
    m_.display.reset(new VideoDisplay(context));
    m_.target_tracker.reset(new TargetTracker(context));

    m_.gst_main->ready_signal()->connect(
        std::bind(&CameraDriver::HandleGstReady, m_.camera.get(),
                  std::placeholders::_1));
    m_.camera->AddFrameConsumer(m_.video_link_tx->get_frame_consumer());
    m_.camera->AddFrameConsumer(m_.target_tracker->get_frame_consumer());

    m_.gst_main->ready_signal()->connect(
        std::bind(&VideoDisplay::HandleGstReady, m_.display.get(),
                  std::placeholders::_1));

    m_.video_link_rx->frame_ready_signal()->connect(
      std::bind(&VideoDisplay::HandleIncomingFrame, m_.display.get(),
                std::placeholders::_1));

    m_.display->stats_signal()->connect(
       std::bind(&VideoLoopbackApp::HandleStats, this,
                 std::placeholders::_1));

    m_.video_link_rx->telemetry_ready_signal()->connect(
        std::bind(&VideoLoopbackApp::ReceiveTelemetry, this,
                  std::placeholders::_1, std::placeholders::_2));

    const std::string kAddr = "127.0.0.1:12542";
    m_.video_link_tx->parameters()->link.dest = kAddr;
    m_.video_link_rx->parameters()->link.source = kAddr;
  }

  void AsyncStart(mjlib::io::ErrorCallback handler) {
    parameters_.children.Start(handler);
    StartTimer();
  }

  struct Members {
    std::unique_ptr<GstMainLoop> gst_main;
    std::unique_ptr<CameraDriver> camera;
    std::unique_ptr<VideoDisplay> display;
    std::unique_ptr<TargetTracker> target_tracker;
    std::unique_ptr<McastVideoLinkTransmitter> video_link_tx;
    std::unique_ptr<McastVideoLinkReceiver> video_link_rx;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(gst_main));
      a->Visit(MJ_NVP(camera));
      a->Visit(MJ_NVP(display));
      a->Visit(MJ_NVP(target_tracker));
      a->Visit(MJ_NVP(video_link_tx));
      a->Visit(MJ_NVP(video_link_rx));
    }
  };

  struct Parameters {
    // if non-zero, exit after that many stats messages are received
    int max_stats = 0;
    // if True, crash when stats do indicate the video is not working.
    bool require_stats_good = false;

    double debug_telemetry_period_s = 0.0;
    double debug_telemetry_expire_s = 0.2;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(max_stats));
      a->Visit(MJ_NVP(require_stats_good));
      a->Visit(MJ_NVP(debug_telemetry_period_s));
      a->Visit(MJ_NVP(debug_telemetry_expire_s));
    }
  };

  clipp::group options() {
    return (
        mjlib::base::ClippArchive().Accept(&parameters_).release(),
        base::ClippComponentArchive().Accept(&m_).release()
          );
  }

 private:
  void HandleStats(const VideoDisplay::Stats* stats) {
    stats_count_++;
    if (parameters_.require_stats_good) {
      std::ostringstream errors;

      // TODO mafanasyev: re-enable this check once we hook raw frame counter
      //if (stats->raw_frames < 5) {
      //  errors << "not enough raw frames; ";
      //}
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
      std::exit(0);
    }
  }

  void ReceiveTelemetry(const std::string& name, const std::string& data) {
    if (log_.isDebugEnabled()) {
      log_.debug("telemetry: " + name + ": '" + data + "'");
    }
  }

  void StartTimer() {
    if (parameters_.debug_telemetry_period_s == 0.0) { return; }

    timer_.expires_from_now(base::ConvertSecondsToDuration(
                                parameters_.debug_telemetry_period_s));
    timer_.async_wait(std::bind(&VideoLoopbackApp::HandleTimer, this,
                                std::placeholders::_1));
  }

  void HandleTimer(boost::system::error_code ec) {
    base::FailIf(ec);
    StartTimer();

    auto tel = m_.video_link_tx->get_telemetry_interface().lock();
    const auto now = mjlib::io::Now(executor_.context());
    const auto next = now + base::ConvertSecondsToDuration(
        parameters_.debug_telemetry_expire_s);
    tel->SetTelemetry(
        "debug",
        base::Stringify(mjlib::io::Now(executor_.context())),
        next);
  }

  boost::asio::executor executor_;
  Members m_;
  Parameters parameters_;
  int stats_count_ = 0;
  base::LogRef log_ = base::GetLogInstance("video_loopback_app");
  base::DeadlineTimer timer_{executor_};
};

}
}
