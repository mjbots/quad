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

#include "sophus/se3.hpp"

#include <boost/test/auto_unit_test.hpp>

BOOST_AUTO_TEST_CASE(DoIUnderstandFrames) {
  // I think that this means, that point (0, 0, 0) in frame B, should
  // be the same as point (1, 0, 0) in frame A.
  Sophus::SE3d pose_AB(Sophus::SO3d(), Eigen::Vector3d(1.0, 0, 0));

  const Eigen::Vector3d p_B = Eigen::Vector3d(0, 0, 0);
  const Eigen::Vector3d p_A = pose_AB * p_B;
  BOOST_TEST(p_A.x() == 1.0);
}
