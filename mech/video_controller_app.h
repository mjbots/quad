// Copyright 2019-2020 Josh Pieper, jjp@pobox.com.
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

#include "mjlib/base/fail.h"
#include "mjlib/base/fast_stream.h"
#include "mjlib/base/system_error.h"
#include "mjlib/telemetry/binary_read_archive.h"

#include "base/component_archives.h"
#include "base/logging.h"

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
    : executor_(context.executor) {
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
  }

  void AsyncStart(mjlib::io::ErrorCallback handler) {
    base::StartArchive::Start(&m_, std::move(handler));
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
    // if non-zero, exit after that many stats messages are received
    int max_stats = 0;
    // if True, crash when stats do indicate the video is not working.
    bool require_stats_good = false;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(max_stats));
      a->Visit(MJ_NVP(require_stats_good));
    }
  };

  clipp::group program_options() {
    return (
        mjlib::base::ClippArchive().Accept(&parameters_).release(),
        base::ClippComponentArchive().Accept(&m_).release()
            );
  }

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
          mjlib::base::Fail("status report had errors:\n " + errors.str());
        };
      }
    }
    if (parameters_.max_stats && parameters_.max_stats <= stats_count_) {
      log_.notice("Got required number of stats, quitting");
      std::exit(0);
    }
  }

  void HandleTelemetry(const std::string& name,
                       const std::string& data) {
    if (name == "mech") {
      mjlib::base::FastIStringStream istr(data);

      MechTelemetry telemetry;
      try {
        mjlib::telemetry::BinaryReadArchive(istr).Accept(&telemetry);
      } catch (mjlib::base::system_error& se) {
        log_.warn("invalid telemetry: " + se.code().message());
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
          fmt::format("Servo: {:.1f}/{:.1f}V {:.1f}/{:.1f}C\n"
                      "Fire: {:.0f}(s)\n"
                      "Turret: {:4.0f}(deg)\n"
                      "x/y/r {:4.0f}/{:4.0f}/{:3.0f}\n"
                      "Mode: {}",
                      telemetry.servo_min_voltage_V,
                      telemetry.servo_max_voltage_V,
                      telemetry.servo_min_temp_C,
                      telemetry.servo_max_temp_C,
                      telemetry.total_fire_time_s,
                      telemetry.turret_absolute_deg,
                      telemetry.gait_x_mm_s,
                      telemetry.gait_y_mm_s,
                      telemetry.gait_rot_deg_s,
                      mode_str));
      m_.display->SetTrackerTarget(telemetry.target_data);
    }
  }

  boost::asio::executor executor_;
  Members m_;
  Parameters parameters_;
  int stats_count_ = 0;
  base::LogRef log_ = base::GetLogInstance("video_controller_app");
};

}
}
