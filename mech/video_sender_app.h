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
#include "rtsp_server.h"

namespace mjmech {
namespace mech {

class VideoSenderApp : boost::noncopyable {
 public:
  template <typename Context>
    VideoSenderApp(Context& context)
    : service_(context.service) {
    m_.camera.reset(new CameraDriver(context));
    m_.rtsp.reset(new RtspServer(context));

    m_.camera->AddFrameConsumer(m_.rtsp->get_frame_consumer());

    m_.camera->stats_signal()->connect(
       std::bind(&VideoSenderApp::HandleStats, this, std::placeholders::_1));
  }

  void AsyncStart(base::ErrorHandler handler) {
    parameters_.children.Start(handler);
  }

  struct Members {
    std::unique_ptr<CameraDriver> camera;
    std::unique_ptr<RtspServer> rtsp;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(camera));
      a->Visit(MJ_NVP(rtsp));
    }
  };

  struct Parameters {
    base::ComponentParameters<Members> children;

    // if non-zero, exit after that many stats messages are received
    int max_stats = 0;
    // if True, crash when stats do indicate the camera is not working.
    bool require_stats_good = false;

    template <typename Archive>
    void Serialize(Archive* a) {
      children.Serialize(a);
      a->Visit(MJ_NVP(max_stats));
      a->Visit(MJ_NVP(require_stats_good));
    }

    Parameters(Members* m) : children(m) {}
  };

  Parameters* parameters() { return &parameters_; }

 private:
  void HandleStats(const CameraDriver::CameraStats* stats) {
    stats_count_++;
    if (parameters_.require_stats_good) {
      std::ostringstream errors;
      if (stats->raw_frames < 5) {
        errors << "not enough raw frames; ";
      }
      if (stats->h264_frames < 5) {
        errors << "not enough h264 frames; ";
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
      log_.notice("Got required number of camera stats, quitting");
      service_.stop();
    }
  }

  boost::asio::io_service& service_;
  Members m_;
  Parameters parameters_{&m_};
  int stats_count_ = 0;
  base::LogRef log_ = base::GetLogInstance("video_sender_app");
};
}
}
