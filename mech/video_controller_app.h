// Copyright 2015-2016 Mikhail Afanasyev.  All rights reserved.
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

#include "base/component_archives.h"
#include "base/fail.h"
#include "base/fast_stream.h"
#include "base/logging.h"
#include "base/program_options_archive.h"
#include "base/telemetry_archive.h"

#include "gst_main_loop.h"
#include "mcast_video_link.h"
#include "mech_telemetry.h"
#include "mech_warfare_data.h"
#include "video_display.h"

namespace mjmech {
namespace mech {

class VideoControllerApp : boost::noncopyable {
 public:
  template <typename Context>
    VideoControllerApp(Context& context)
    : service_(context.service) {
    m_.gst_main.reset(new GstMainLoop(context));
    m_.display.reset(new VideoDisplay(context));
    m_.video_link.reset(new McastVideoLinkReceiver(context));

    m_.gst_main->ready_signal()->connect(
        std::bind(&VideoDisplay::HandleGstReady, m_.display.get(),
                  std::placeholders::_1));

    m_.video_link->frame_ready_signal()->connect(
      std::bind(&VideoDisplay::HandleIncomingFrame, m_.display.get(),
                std::placeholders::_1));
    m_.video_link->telemetry_ready_signal()->connect(
        std::bind(&VideoControllerApp::HandleTelemetry, this,
                  std::placeholders::_1, std::placeholders::_2));

    m_.display->stats_signal()->connect(
       std::bind(&VideoControllerApp::HandleStats, this,
                 std::placeholders::_1));

    base::ProgramOptionsArchive(&options_).Accept(&parameters_);
  }

  void AsyncStart(base::ErrorHandler handler) {
    parameters_.children.Start(handler);
  }

  struct Members {
    std::unique_ptr<GstMainLoop> gst_main;
    std::unique_ptr<VideoDisplay> display;
    std::unique_ptr<McastVideoLinkReceiver> video_link;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(gst_main));
      a->Visit(MJ_NVP(display));
      a->Visit(MJ_NVP(video_link));
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

    Parameters(Members* m) : children(m) {}
  };

  Parameters* parameters() { return &parameters_; }
  boost::program_options::options_description* options() { return &options_; }

  void SetTargetOffset(int x, int y) {
    m_.display->SetTargetOffset(x, y);
  }

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

  void HandleTelemetry(const std::string& name,
                       const std::string& data) {
    if (name == "mech") {
      base::FastIStringStream istr(data);

      MechTelemetry telemetry;
      try {
        base::TelemetrySimpleReadArchive<MechTelemetry>::Deserialize(
            &telemetry, istr);
      } catch (base::SystemError& se) {
        log_.warn("invalid telemetry: " + se.error_code().message());
        return;
      }

      const auto mode_map = MechWarfareData::ModeMapper();
      const auto it = mode_map.find(
          static_cast<MechWarfareData::Mode>(telemetry.mech_mode));
      const std::string mode_str =
          (it == mode_map.end()) ?
          "UNKNOWN" :
          it->second;

      m_.display->SetOsdText(
          fmt::format("Servo: {:.1f}/{:.1f}\n"
                      "Fire: {:.0f}(s)\n"
                      "Turret: {:.0f}(deg)\n"
                      "Mode: {}",
                      telemetry.servo_min_voltage_V,
                      telemetry.servo_max_voltage_V,
                      telemetry.total_fire_time_s,
                      telemetry.turret_absolute_deg,
                      mode_str));
      m_.display->SetTrackerTarget(telemetry.target_data);
    }
  }

  boost::asio::io_service& service_;
  Members m_;
  Parameters parameters_{&m_};
  boost::program_options::options_description options_;
  int stats_count_ = 0;
  base::LogRef log_ = base::GetLogInstance("video_controller_app");
};

}
}
