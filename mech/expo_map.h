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

#include <cmath>

namespace mjmech {
namespace mech {

/// A two rate piecewise linear mapping with a central deadband.
///
/// For the positive side, the input-output mapping:
///
///  0     - 0.0
///  0.05  - 0.0
///  0.3   - 0.1
///  1.0   - 1.0
class ExpoMap {
 public:
  struct Options {
    double deadband = 0.05;
    double slow_range = 0.30;
    double slow_value = 0.10;

    Options() {}
  };

  ExpoMap(const Options& options = Options()) : options_(options) {}

  double operator()(double value) const {
    const auto& o = options_;

    if (std::abs(value) < o.deadband) { return 0.0; }
    const double sign = std::copysign(1.0, value);
    if (std::abs(value) < o.slow_range) {
      return o.slow_value * (value - o.deadband * sign) /
          (o.slow_range - o.deadband);
    }
    return (value - (sign * o.slow_range)) /
        (1.0 - o.slow_range) *
        (1.0 - o.slow_value) + sign * o.slow_value;
  }

 private:
  Options options_;
};

}
}
