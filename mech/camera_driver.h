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

#include "gst_main_loop.h"

struct _GstSample;
typedef struct _GstSample GstSample;

namespace mjmech {
namespace mech {

class CameraFrameConsumer;

class CameraDriver : boost::noncopyable {
 public:
  CameraDriver(base::Context& context);
  ~CameraDriver();

  void AsyncStart(base::ErrorHandler handler);

  // Register a frame consumer. Pointer must be valid for the lifetime
  // of an object. Must be called before AsyncStart.
  void AddFrameConsumer(std::weak_ptr<CameraFrameConsumer>);

  struct Parameters {
    double stats_interval_s = 1.0;

    // may be set to TEST to use test source
    std::string device;

    // C920 camera is somewhat picky about framerate/framesize combinations,
    // so we provide a number of presets which define the default values of
    // the field.

    // Camera quality preset index. Values from 0 to (# of presets - 1).
    // Smaller preset indices indicate higher data rate.
    int preset = 1;
    // Bitrate scaling. The bandwith passed to the driver a product of this
    // value and a bitrate_mbps value (which may come from preset);
    double bitrate_scale = 1.0;

    /// Preset override fields. -1 / empty string for preset default.
    //{
    // Desired framerate.
    double framerate = -1;
    // The viewfinder framerate (0 is same as primary)
    double decoded_framerate = -1;
    // Desired bitrate, megabits/sec. 0 for camera-default (3), -1 for
    // preset-default.
    double bitrate_mbps = -1;
    // Partial caps for h264 stream. Only applicable to h264 camera.
    // Empty string for preset-default.
    std::string h264_caps;
    // Partial caps for decoded stream, specifies primary caps for dumb camera.
    // Empty string for preset-default.
    std::string decoded_caps;
    // Extra parameters to uvch264src
    std::string extra_uvch264;
    //}

    // Desired I-frame interval. 0 for camera-default. 1.0 seems to be the
    // smallest possible  -- any smaller numbers just cause the setting to
    // be ignored.
    double iframe_interval_s = 1.0;
    // ratio of peak bitrate to average bitrate. 1.0 makes them identical.
    double peak_bitrate_scale = 1.5;

    // If non-empty, write video file to this location
    std::string write_video;
    // If True, use regular v4l driver and encode h264 in software
    bool dumb_camera = false;
    // If True, treat "device" as custom gstreamer pipeline string.
    bool raw_gstreamer = false;
    // If non-empty, gstreamer pipeline that is fed h264 data
    std::string custom_h264_consumer;
    // If True, calculate and log frame properties (like average brightness)
    // (this operates off uncompressed camera output)
    bool analyze = false;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(stats_interval_s));
      a->Visit(MJ_NVP(device));

      a->Visit(MJ_NVP(preset));
      a->Visit(MJ_NVP(bitrate_scale));

      a->Visit(MJ_NVP(framerate));
      a->Visit(MJ_NVP(decoded_framerate));
      a->Visit(MJ_NVP(bitrate_mbps));
      a->Visit(MJ_NVP(h264_caps));
      a->Visit(MJ_NVP(decoded_caps));
      a->Visit(MJ_NVP(extra_uvch264));

      a->Visit(MJ_NVP(iframe_interval_s));
      a->Visit(MJ_NVP(peak_bitrate_scale));

      a->Visit(MJ_NVP(write_video));
      a->Visit(MJ_NVP(dumb_camera));
      a->Visit(MJ_NVP(raw_gstreamer));
      a->Visit(MJ_NVP(custom_h264_consumer));
      a->Visit(MJ_NVP(analyze));
    }
  };

  Parameters* parameters() { return &parameters_; }

  struct CameraStats {
    boost::posix_time::ptime timestamp;
    // raw frames from camera
    int raw_frames = 0;
    // h264 chunks
    int h264_frames = 0;
    int h264_key_frames = 0;
    double h264_max_interval_s = 0;
    // bytes in h264 chunks
    int h264_bytes = 0;
    int h264_max_bytes = 0;
    // h264 chunks which were sent to an active RTSP connection
    int h264_frames_rtsp = 0;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(timestamp));
      a->Visit(MJ_NVP(raw_frames));
      a->Visit(MJ_NVP(h264_frames));
      a->Visit(MJ_NVP(h264_key_frames));
      a->Visit(MJ_NVP(h264_max_interval_s));
      a->Visit(MJ_NVP(h264_bytes));
      a->Visit(MJ_NVP(h264_max_bytes));
      a->Visit(MJ_NVP(h264_frames_rtsp));
    }
  };

  boost::signals2::signal<void (const CameraStats*)>* stats_signal() {
    return &camera_stats_signal_;
  }

  void HandleGstReady(GstMainLoopRef&);

 private:
  boost::signals2::signal<void (const CameraStats*)> camera_stats_signal_;
  Parameters parameters_;

  class Impl;
  std::unique_ptr<Impl> impl_;
};

// This interface class can consume camera frames.
class CameraFrameConsumer : boost::noncopyable {
 public:
  // Consume an h264 frame. Invoked from an internal thread.
  virtual void ConsumeH264Sample(GstSample*) {};

  // Consume a raw frame. Invoked from an internal thread.
  virtual void ConsumeRawSample(GstSample*) {};

  // Going to emit stats soon. The argument is a mutable
  // stats pointer. Invoked from main gst thread.
  virtual void PreEmitStats(CameraDriver::CameraStats* stats) {};
};

}
}
