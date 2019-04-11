// Copyright 2014-2015 Mikhail Afanasyev.  All rights reserved.
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

#include <memory>

#include <boost/asio/io_service.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <boost/signals2/signal.hpp>

#include "mjlib/base/visitor.h"
#include "mjlib/io/async_types.h"

#include "base/context.h"

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

  void AsyncStart(mjlib::io::ErrorCallback handler);

  // Register a frame consumer. Pointer must be valid for the lifetime
  // of an object. Must be called before AsyncStart.
  void AddFrameConsumer(std::weak_ptr<CameraFrameConsumer>);

  struct Parameters {
    double stats_interval_s = 1.0;

    // The device to use.  This may have a number of special values:
    //  "TEST" - use the test source
    //  "JOULE" - use the joule camera
    //  "c920:" - a C920 webcam
    //  "gstreamer:" - use a custom gstreamer pipeline as the source
    //    it is required to emit a primary stream and a secondary
    //    decoded stream on the "dec-tee" pad.  For now, the primary
    //    stream must be un-encoded.
    //  "v4l2:" - a V4L2 device
    std::string device;

    // Camera quality preset index. Values from 0 to (# of presets - 1).
    // Smaller preset indices indicate higher data rate.
    int preset = 1;

    // Bitrate scaling. The bandwith passed to the driver a product of this
    // value and a bitrate_kbps value (which may come from preset);
    double bitrate_scale = 1.0;

    // For encoders that support it, the maximum instantaneous rate
    // relative to the standard rate.
    double bitrate_peak_scale = 1.5;

    // Desired I-frame interval.  0 for default.
    double iframe_interval_s = 1.0;

    /// Preset override fields. -1 / empty string for preset default.
    //{
    // Desired bitrate, kbits/sec. 0 for camera-default (3), -1 for
    // preset-default.
    int bitrate_kbps = -1;

    int h264_width = -1;
    int h264_height = -1;
    //}

    // The encoder to use.  This may have a number of special values:
    //  "X264" - a default x264enc based encoder
    //  "VAAPI" - a default VAAPI based encoder
    //  "gstreamer:" - use a custom gstreamer pipeline
    std::string h264_encoder = "X264";


    // If non-empty, write video file to this location
    std::string write_video;

    // If non-empty, gstreamer pipeline that is fed h264 data
    std::string custom_h264_consumer;
    // If True, calculate and log frame properties (like average brightness)
    // (this operates off uncompressed camera output)
    bool analyze = false;

    // If set, use a completely custom gstreamer pipeline.  This has
    // many undocumented constraints.
    std::string gstreamer_pipeline;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(stats_interval_s));
      a->Visit(MJ_NVP(device));


      a->Visit(MJ_NVP(preset));
      a->Visit(MJ_NVP(bitrate_scale));
      a->Visit(MJ_NVP(bitrate_peak_scale));
      a->Visit(MJ_NVP(iframe_interval_s));

      a->Visit(MJ_NVP(bitrate_kbps));
      a->Visit(MJ_NVP(h264_width));
      a->Visit(MJ_NVP(h264_height));

      a->Visit(MJ_NVP(h264_encoder));

      a->Visit(MJ_NVP(write_video));
      a->Visit(MJ_NVP(custom_h264_consumer));
      a->Visit(MJ_NVP(analyze));
      a->Visit(MJ_NVP(gstreamer_pipeline));
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
  virtual ~CameraFrameConsumer() {}

  // Consume an h264 frame. Invoked from an internal thread.
  virtual void ConsumeH264Sample(GstSample*) {};

  // Consume a raw frame. Invoked from an internal thread.
  virtual void ConsumeRawSample(GstSample*) {};

  // Going to emit stats soon. The argument is a mutable
  // stats pointer. Invoked from main gst thread.
  virtual void PreEmitStats(CameraDriver::CameraStats*) {};
};

}
}
