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

namespace mjmech {
namespace mech {

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

  struct Parameters {
    // argv/argc to pass to gst
    std::string gst_options;

    double stats_interval_ms = 1000;
    bool print_stats = false;
    std::string device;
    std::string resolution;
    std::string write_h264;
    bool dumb_camera = false;


    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(gst_options));
      a->Visit(MJ_NVP(stats_interval_ms));
      a->Visit(MJ_NVP(print_stats));
      // may be set to TEST to use test source
      a->Visit(MJ_NVP(device));
      // in 640x480 format
      a->Visit(MJ_NVP(resolution));
      a->Visit(MJ_NVP(write_h264));
      a->Visit(MJ_NVP(dumb_camera));
    }
  };

  Parameters* parameters() { return &parameters_; }

  struct CameraStats {
    boost::posix_time::ptime timestamp;
    int raw_frames = 0;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(timestamp));
      a->Visit(MJ_NVP(raw_frames));
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
}
}
