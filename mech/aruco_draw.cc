// Copyright 2018-2019 Josh Pieper, jjp@pobox.com.  All rights reserved.
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

#include <boost/assert.hpp>
#include <boost/format.hpp>
#include <boost/timer/timer.hpp>

#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

extern "C" {

int main(int argc, char** argv) {
  BOOST_VERIFY(argc >= 3);

  cv::Mat markerImage;

  cv::Ptr<cv::aruco::Dictionary> dictionary =
      cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);

  cv::aruco::drawMarker(dictionary, std::stoi(argv[1]), 200, markerImage, 1);

  cv::imwrite(argv[2], markerImage);

  return 0;
}

}
