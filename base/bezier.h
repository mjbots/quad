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

namespace mjmech {
namespace base {

/// Interpolate over a cubic bezier function with fixed control points
/// such that velocity is 0 at start and end and the acceleration is
/// continuous within that range.
template <typename T>
class Bezier {
 public:
  Bezier(T start, T end)
      : start_(start),
        end_(end) {}

  T position(double phase) const {
    const double bezier =
        phase * phase * phase + 3.0 * (phase * phase * (1.0 - phase));
    return start_ + bezier * delta_;
  }

  T velocity(double phase) const {
    const double bezier = 6 * phase * (1.0 - phase);
    return bezier * delta_;
  }

  T acceleration(double phase) const {
    const double bezier = 6 - 12 * phase;
    return bezier * delta_;
  }

 private:
  T start_;
  T end_;
  T delta_{end_ - start_};
};

}
}
