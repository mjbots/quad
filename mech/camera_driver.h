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
  void AddFrameConsumer(CameraFrameConsumer*);

  struct Parameters {
    // argv/argc to pass to gst
    std::string gst_options;

    double stats_interval_ms = 1000;
    bool print_stats = false;
    std::string device;
    std::string h264_caps;
    std::string decoded_caps;
    std::string write_h264;
    bool dumb_camera = false;
    int verbosity = 0;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(gst_options));
      a->Visit(MJ_NVP(stats_interval_ms));
      a->Visit(MJ_NVP(print_stats));
      // may be set to TEST to use test source
      a->Visit(MJ_NVP(device));
      a->Visit(MJ_NVP(h264_caps));
      a->Visit(MJ_NVP(decoded_caps));
      a->Visit(MJ_NVP(write_h264));
      a->Visit(MJ_NVP(dumb_camera));
      a->Visit(MJ_NVP(verbosity));
    }
  };

  Parameters* parameters() { return &parameters_; }

  struct CameraStats {
    boost::posix_time::ptime timestamp;
    // raw frames from camera
    int raw_frames = 0;
    // h264 chunks
    int h264_frames = 0;
    // bytes in h264 chunks
    int h264_bytes = 0;
    // h264 chunks which were sent to an active RTSP connection
    int h264_frames_rtsp = 0;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(timestamp));
      a->Visit(MJ_NVP(raw_frames));
      a->Visit(MJ_NVP(h264_frames));
      a->Visit(MJ_NVP(h264_bytes));
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
