// Copyright 2020 Josh Pieper, jjp@pobox.com.  All rights reserved.
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

#include "base/leg_force.h"

#include <fmt/format.h>

#include <unsupported/Eigen/LevenbergMarquardt>
#include <unsupported/Eigen/NumericalDiff>

#include "mjlib/base/assert.h"
#include "mjlib/base/system_error.h"

namespace mjmech {
namespace base {

namespace {
struct LegFunctor : public Eigen::DenseFunctor<double> {
  LegFunctor(const std::vector<Eigen::Vector2d>& legs)
      : Eigen::DenseFunctor<double>(legs.size(), 5 + legs.size()),
        legs_(legs) {}

  int operator()(const Eigen::VectorXd& x, Eigen::VectorXd& fvec) const {
    MJ_ASSERT(fvec.size() == 5 + legs_.size());
    MJ_ASSERT(x.size() == legs_.size());
    fvec(0) = 0.0;
    fvec(1) = 0.0;
    fvec(2) = -1.0;  // This is the total sum of all legs.
    fvec(3) = 0.0;
    fvec(4) = 0.0;

    for (size_t i = 0; i < legs_.size(); i++) {
      fvec(0) += legs_[i].x() * x(i);
      fvec(1) += 10 * legs_[i].y() * x(i);
      fvec(2) += x(i);
      if (x(i) < 0.0) { fvec(3) = x(i); }
      if (x(i) > 1.0) { fvec(4) = x(i); }

      fvec(5 + i) = balance_ratio_ * (x(i) - (1.0 / legs_.size()));
    }

    fvec(2) *= 1e6;

    return 0;
  }

  std::vector<Eigen::Vector2d> legs_;
  // Just some small value.
  const double balance_ratio_ = 0.001;
};
}

std::vector<double> OptimizeLegForce(const std::vector<Eigen::Vector2d>& legs) {
  if (legs.size() == 0) { return {}; }
  if (legs.size() == 1) { return { 1.0 }; }

  Eigen::VectorXd solution(legs.size());

  LegFunctor lf{legs};
  Eigen::NumericalDiff<LegFunctor> nf{lf};
  Eigen::LevenbergMarquardt<Eigen::NumericalDiff<LegFunctor>> lm{nf};
  lm.minimize(solution);

  std::vector<double> result;
  for (size_t i = 0; i < legs.size(); i++) {
    result.push_back(solution(i));
  }

  return result;
}

}
}
