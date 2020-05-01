// Copyright 2016-2020 Josh Pieper, jjp@pobox.com.
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

#include "target_tracker.h"

#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

namespace mjmech {
namespace mech {

class TargetTracker::Impl {
 public:
  Impl(const Options& options) {
    cv::setNumThreads(1);
  }

  Result Track(const cv::Mat& image) {
    std::vector<int> marker_ids;
    std::vector<std::vector<cv::Point2f>> marker_corners;

    cv::aruco::detectMarkers(
        image, aruco_dictionary_,
        marker_corners, marker_ids,
        aruco_parameters_);

    Result result;
    for (const auto& corners : marker_corners) {
      Eigen::Vector2d total;
      for (const auto& corner : corners) {
        total += Eigen::Vector2d(corner.x, corner.y);
      }
      result.targets.push_back(total * (1.0 / corners.size()));
    }
    return result;
  }

  cv::Ptr<cv::aruco::Dictionary> aruco_dictionary_ =
      cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
  cv::Ptr<cv::aruco::DetectorParameters> aruco_parameters_ =
      cv::aruco::DetectorParameters::create();
};

TargetTracker::TargetTracker(const Options& options)
    : impl_(new Impl(options)) {}

TargetTracker::~TargetTracker() {}

TargetTracker::Result TargetTracker::Track(const cv::Mat& image) {
  return impl_->Track(image);
}

}
}
