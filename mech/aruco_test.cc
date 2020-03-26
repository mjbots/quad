// Copyright 2018-2019 Josh Pieper, jjp@pobox.com.
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

#include <iostream>

#include <fmt/format.h>

#include <boost/assert.hpp>
#include <boost/timer/timer.hpp>

#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

extern "C" {

int main(int argc, char** argv) {
  BOOST_VERIFY(argc >= 2);
  cv::Mat input = cv::imread(argv[1]);
  cv::Ptr<cv::aruco::Dictionary> dictionary =
      cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
  std::vector<int> markerIds;
  std::vector<std::vector<cv::Point2f>> markerCorners;

  auto parameters = cv::aruco::DetectorParameters::create();

  cv::setNumThreads(1);

  constexpr int width = 480;
  constexpr int height = 480;

  cv::Rect crop_area(input.cols / 2 - width / 2, input.rows / 2 - height / 2,
                     width, height);

  {
    boost::timer::auto_cpu_timer t;
    for (int i = 0; i < 100; i++) {
      cv::Mat subset = input(crop_area);
      cv::aruco::detectMarkers(subset, dictionary, markerCorners, markerIds, parameters);
    }
  }

  std::cout << markerIds.size() << "\n";
  for (const auto& marker : markerCorners) {
    for (const auto& corner : marker) {
      std::cout << fmt::format("({:f},{:f}) ", corner.x, corner.y);
    }
    std::cout << "\n";
  }

  return 0;
}

}
