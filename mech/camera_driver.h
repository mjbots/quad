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

struct _GstSample;
typedef struct _GstSample GstSample;

namespace mjmech {
namespace mech {

class CameraFrameConsumer;

class CameraDriver : boost::noncopyable {
 public:
  template <typename Context>
    CameraDriver(Context& context)
    : CameraDriver(context.service,
                   &context.telemetry_registry) {}

  template <typename TelemetryRegistry>
    CameraDriver(boost::asio::io_service& service,
                 TelemetryRegistry* telemetry_registry)
    : CameraDriver(service) {
    telemetry_registry->Register("camera_stats", &camera_stats_signal_);
  }

  CameraDriver(boost::asio::io_service&);
  ~CameraDriver();

  void AsyncStart(base::ErrorHandler handler);

  // Register a frame consumer. Pointer must be valid for the lifetime
  // of an object. Must be called before AsyncStart.
  void AddFrameConsumer(std::weak_ptr<CameraFrameConsumer>);

  struct Parameters {
    // argv/argc to pass to gst
    std::string gst_options;

    double stats_interval_s = 1.0;

    // may be set to TEST to use test source
    std::string device;

    // Desired framerate
    double framerate = 30;
    // For h264 and dumb cameras, the viewfinder framerate (0 means same as
    // primary)
    double decoded_framerate = 0;

    // Desired I-frame interval. 0 for default.
    double iframe_interval_s = 1.0;
    // Desired bitrate, BYTES/sec. 0 for default (375000)
    int bitrate_Bps = 0;

    // Partial caps for h264 stream. Only applicable to h264 camera.
    std::string h264_caps = "width=1920,height=1080";
    // Partial caps for decoded stream, specifies primary caps for dumb camera.
    std::string decoded_caps = "width=640,height=480";
    // Extra parameters to uvch264src
    std::string extra_uvch264;
    // If non-empty, write video file to this location
    std::string write_video;

    // If True, use regular v4l driver and encode h264 in software
    bool dumb_camera = false;
    // If non-empty, gstreamer pipeline that is fed h264 data
    std::string custom_h264_consumer;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(gst_options));
      a->Visit(MJ_NVP(stats_interval_s));
      a->Visit(MJ_NVP(device));

      a->Visit(MJ_NVP(framerate));
      a->Visit(MJ_NVP(decoded_framerate));
      a->Visit(MJ_NVP(iframe_interval_s));
      a->Visit(MJ_NVP(bitrate_Bps));

      a->Visit(MJ_NVP(h264_caps));
      a->Visit(MJ_NVP(decoded_caps));
      a->Visit(MJ_NVP(extra_uvch264));

      a->Visit(MJ_NVP(write_video));
      a->Visit(MJ_NVP(dumb_camera));
      a->Visit(MJ_NVP(custom_h264_consumer));
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
    // h264 chunks which were sent to an active RTSP connection
    int h264_frames_rtsp = 0;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(timestamp));
      a->Visit(MJ_NVP(raw_frames));
      a->Visit(MJ_NVP(h264_frames));
      a->Visit(MJ_NVP(h264_key_frames));
      a->Visit(MJ_NVP(h264_bytes));
      a->Visit(MJ_NVP(h264_max_interval_s));
      a->Visit(MJ_NVP(h264_frames_rtsp));
    }
  };

  boost::signals2::signal<void (const CameraStats*)>* stats_signal() {
    return &camera_stats_signal_;
  }

 private:
  boost::signals2::signal<void (const CameraStats*)> camera_stats_signal_;
  Parameters parameters_;

  class Impl;
  std::unique_ptr<Impl> impl_;
};


// This interface class can consume camera frames.
class CameraFrameConsumer : boost::noncopyable {
 public:
  // Gstreamer has been initialized. The consumer can set up its
  // internal settings. Invoked from main gst thread.
  virtual void GstReady() {};

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
