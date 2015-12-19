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
#include "base/visitor.h"

#include "gst_main_loop.h"

struct _GstSample;
typedef struct _GstSample GstSample;

namespace mjmech {
namespace mech {

class VideoDisplay : boost::noncopyable {
 public:
  template <typename Context>
  VideoDisplay(Context& context)
      : VideoDisplay(context.service) {
    context.telemetry_registry->Register("video_stats", &stats_signal_);
  }

  VideoDisplay(boost::asio::io_service&);
  ~VideoDisplay();

  void AsyncStart(base::ErrorHandler handler);

  struct Parameters {
    double stats_interval_s = 1.0;

    // A display source:
    //  If empty, we assume some other component will provide video
    //  If "TEST", will use test source
    //  If starts with rtsp://, an RTSP URL
    //  Else, a gstreamer pipeline fragment
    std::string source;

    // If non-empty, write video file to this location
    std::string write_video;

    // If True, we pass the frames via our app to enable OSD
    // and missing video indication.
    bool process_frames = false;

    // If True, do not pop up X11 window
    bool hide_video = false;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(stats_interval_s));
      a->Visit(MJ_NVP(source));
      a->Visit(MJ_NVP(write_video));
      a->Visit(MJ_NVP(process_frames));
      a->Visit(MJ_NVP(hide_video));
    }
  };

  Parameters* parameters() { return &parameters_; }

  struct Stats {
    boost::posix_time::ptime timestamp;
    // raw frames from the source -- one may be an UDP packet, for example
    int raw_frames = 0;
    int raw_bytes = 0;
    // h264 frames (regular and key)
    int h264_frames = 0;
    int h264_key_frames = 0;
    // decoded frames
    int decoded_frames = 0;
    double decoded_max_interval_s = 0;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(timestamp));
      a->Visit(MJ_NVP(raw_frames));
      a->Visit(MJ_NVP(raw_bytes));
      a->Visit(MJ_NVP(h264_frames));
      a->Visit(MJ_NVP(h264_key_frames));
      a->Visit(MJ_NVP(decoded_frames));
      a->Visit(MJ_NVP(decoded_max_interval_s));
    }
  };

  boost::signals2::signal<void (const Stats*)>* stats_signal() {
    return &stats_signal_;
  }

  void HandleGstReady(GstMainLoopRef&);

  // Process a frame with h264 data. The argument must contain raw data.
  // WARNING: the argument to this function should never be changed afterwards!
  void HandleIncomingFrame(std::shared_ptr<std::string>&);

 private:
  boost::signals2::signal<void (const Stats*)> stats_signal_;
  Parameters parameters_;

  class Impl;
  std::unique_ptr<Impl> impl_;
};

}
}
