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

#include "base/fit_plane.h"

#include <Eigen/Dense>

namespace mjmech {
namespace base {

Plane FitPlane(const std::vector<Eigen::Vector3d>& points) {
  Eigen::MatrixXd A(points.size(), 3);
  Eigen::MatrixXd B(points.size(), 1);

  for (size_t i = 0; i < points.size(); i++) {
    A(i, 0) = points[i].x();
    A(i, 1) = points[i].y();
    A(i, 2) = 1.0;
    B(i) = points[i].z();
  }

  Eigen::MatrixXd result = A.bdcSvd(
      Eigen::ComputeThinU | Eigen::ComputeThinV).solve(B);
  return Plane{result(0), result(1), result(2)};
}

}
}
