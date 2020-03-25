// Copyright 2014-2019 Josh Pieper, jjp@pobox.com.  All rights reserved.
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

#include <dart/dynamics/BodyNode.hpp>
#include <dart/dynamics/Skeleton.hpp>

#include "mjlib/base/visitor.h"

#include "base/point3d.h"

#include "mech/ik.h"

namespace mjmech {
namespace mech {

class MammalIk : public IkSolver {
 public:
  struct Config {
    struct Joint {
      // The shoulder is allowed to be unconstrained.  All other
      // joints must only have a positive Z value.
      //
      // The reference frame for each joint is the center of rotation
      // of the relevant joint, with +x being away from the rotation
      // and +z being down.
      base::Point3D pose_mm;

      int id = 0;

      template <typename Archive>
      void Serialize(Archive* a) {
        a->Visit(MJ_NVP(pose_mm));
        a->Visit(MJ_NVP(id));
      }
    };

    Joint shoulder;
    Joint femur;
    Joint tibia;

    // There are two possible solutions for most positions.  If
    // "invert" is false, then the one will be chosen for which the
    // tibia angle is negative.
    bool invert = false;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(shoulder));
      a->Visit(MJ_NVP(femur));
      a->Visit(MJ_NVP(tibia));
      a->Visit(MJ_NVP(invert));
    }
  };

  MammalIk(const Config& config);

  Effector Forward_G(const JointAngles& angles) const override;

  InverseResult Inverse(const Effector&,
                        const std::optional<JointAngles>&) const override;

  const Config config_;
  dart::dynamics::SkeletonPtr skel_;
  dart::dynamics::JointPtr shoulder_joint_;
  dart::dynamics::JointPtr femur_joint_;
  dart::dynamics::JointPtr tibia_joint_;
  dart::dynamics::BodyNodePtr shoulder_body_;
  dart::dynamics::BodyNodePtr femur_body_;
  dart::dynamics::BodyNodePtr tibia_body_;
  dart::dynamics::BodyNodePtr foot_body_;
};

}
}
