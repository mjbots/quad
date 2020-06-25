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

#include <vector>

#include <Eigen/Core>

namespace mjmech {
namespace base {

/// For a plane defined as a*x + b*y + c = z
///
/// Yes, this only works for planes that are mostly level.
struct Plane {
  double a = 0.0;
  double b = 0.0;
  double c = 0.0;
};

Plane FitPlane(const std::vector<Eigen::Vector3d>& points);

}
}
