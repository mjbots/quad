// Copyright 2020 Josh Pieper, jjp@pobox.com.
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

#include <boost/signals2/signal.hpp>

#include <opencv2/core/core.hpp>

#include "mjlib/base/visitor.h"
#include "mjlib/io/async_types.h"

namespace mjmech {
namespace mech {

/// Read frames from a camera.
class CameraDriver {
 public:
  struct Options {
    int width = 1280;
    int height = 720;
    int mode = 6;
    int fps = 60;
    int rotation = 270;

    std::string record_path = "/tmp/mjbots-camera";
    int record_every = -1;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(width));
      a->Visit(MJ_NVP(height));
      a->Visit(MJ_NVP(mode));
      a->Visit(MJ_NVP(fps));
      a->Visit(MJ_NVP(rotation));
      a->Visit(MJ_NVP(record_path));
      a->Visit(MJ_NVP(record_every));
    }
  };

  CameraDriver(const Options& options);
  ~CameraDriver();

  using ImageSignal = boost::signals2::signal<void (const cv::Mat&)>;
  // This will be emitted from a background thread.
  ImageSignal* image_signal();

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}
}
