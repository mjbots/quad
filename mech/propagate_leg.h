// Copyright 2019-2020 Josh Pieper, jjp@pobox.com.
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
namespace mech {

/// Propagate a leg given a linear and angular velocity of the CoM.
class PropagateLeg {
 public:
  PropagateLeg(const Eigen::Vector3d& v_R,
               const Eigen::Vector3d& w_R,
               double period_s)
      : v_R_(v_R),
        w_R_(w_R),
        pose_T2_T1_(
          Sophus::SO3d(
              Eigen::AngleAxisd(-period_s * w_R.z(), Eigen::Vector3d::UnitZ())
              .toRotationMatrix()),
          -v_R_ * period_s) {}

  struct Result {
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
  };

  Result operator()(const Eigen::Vector3d& position_R) const {
    Result result;
    result.position = pose_T2_T1_ * position_R;
    result.velocity = -v_R_ - w_R_.cross(position_R);
    return result;
  }

 private:
  Eigen::Vector3d v_R_;
  Eigen::Vector3d w_R_;
  Sophus::SE3d pose_T2_T1_;
};

}
}
