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

#include "mech/camera_driver.h"

#include <thread>

#include <boost/date_time/posix_time/posix_time.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/videoio/videoio.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>

#ifdef COM_GITHUB_MJBOTS_RASPBERRYPI
#include <raspicam_cv.h>
#endif

namespace mjmech {
namespace mech {

#ifdef COM_GITHUB_MJBOTS_RASPBERRYPI
class CameraDriver::Impl {
 public:
  Impl(const Options& options)
      : thread_(std::bind(&Impl::Run, this, options)) {
  }

  ~Impl() {
    done_.store(true);
    thread_.join();
  }

  void Run(Options options) {
    raspicam::RaspiCam_Cv camera;
    camera.set(cv::CAP_PROP_FRAME_WIDTH, options.width);
    camera.set(cv::CAP_PROP_FRAME_HEIGHT, options.height);
    camera.set(cv::CAP_PROP_MODE, options.mode);
    camera.set(cv::CAP_PROP_FPS, options.fps);
    camera.setRotation(options.rotation);
    camera.set(cv::CAP_PROP_FORMAT, CV_8UC3);
    camera.open();

    cv::Mat image;
    uint16_t count = 0;
    while (!done_.load()) {
      camera.grab();
      camera.retrieve(image);
      if (options.record_every >= 1) {
        if (count == 0) {
          cv::imwrite(MakePath(options), image);
          count = options.record_every - 1;
        } else {
          count--;
        }
      }
      image_signal_(image);
    }
  }

  std::string MakePath(const Options& options) const {
    const auto now = boost::posix_time::microsec_clock::universal_time();
    return options.record_path + to_iso_string(now) + ".png";
  }

  std::thread thread_;
  std::atomic<bool> done_{false};
  ImageSignal image_signal_;
};
#else
class CameraDriver::Impl {
 public:
  Impl(const Options&) {}

  ImageSignal image_signal_;
};
#endif

CameraDriver::CameraDriver(const Options& options)
    : impl_(std::make_unique<Impl>(options)) {}

CameraDriver::~CameraDriver() {}

CameraDriver::ImageSignal* CameraDriver::image_signal() {
  return &impl_->image_signal_;
}

}
}
