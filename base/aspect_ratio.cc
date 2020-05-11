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

#include "base/aspect_ratio.h"

namespace mjmech {
namespace base {

Eigen::AlignedBox2i MaintainAspectRatio(Eigen::Vector2i source,
                                        Eigen::Vector2i dest) {
  int x = 0;
  int y = 0;
  int display_w = dest.x();
  int display_h = dest.y();

  // Enforce an aspect ratio.
  const double desired_aspect_ratio =
      static_cast<double>(std::abs(source.x())) /
      static_cast<double>(std::abs(source.y()));
  const double actual_ratio =
      static_cast<double>(display_w) /
      static_cast<double>(display_h);
  if (actual_ratio > desired_aspect_ratio) {
    const int w = display_h * desired_aspect_ratio;
    const int remaining = display_w - w;
    x = remaining / 2;
    display_w = w;
  } else if (actual_ratio < desired_aspect_ratio) {
    const int h = display_w / desired_aspect_ratio;
    const int remaining = display_h - h;
    y = remaining / 2;
    display_h = h;
  }

  return {
    Eigen::Vector2i(x, y),
        Eigen::Vector2i(x + display_w, y + display_h)
        };
}

}
}
