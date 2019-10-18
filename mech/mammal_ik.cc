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

#include "mech/mammal_ik.h"

#include <cmath>

#include <dart/dynamics/RevoluteJoint.hpp>
#include <dart/dynamics/WeldJoint.hpp>

#include "mjlib/base/fail.h"
#include "mjlib/base/limit.h"

#include "base/common.h"

namespace mjmech {
namespace mech {

MammalIk::MammalIk(const Config& config) : config_(config) {
  // Some sanity checks.
  BOOST_ASSERT(config_.femur.pose_mm.x() == 0.0);
  BOOST_ASSERT(config_.femur.pose_mm.y() == 0.0);
  BOOST_ASSERT(config_.femur.pose_mm.z() > 0.0);
  BOOST_ASSERT(config_.tibia.pose_mm.x() == 0.0);
  BOOST_ASSERT(config_.tibia.pose_mm.y() == 0.0);
  BOOST_ASSERT(config_.tibia.pose_mm.z() > 0.0);

  namespace dyn = dart::dynamics;
  skel_ = dyn::Skeleton::create("leg");
  skel_->setGravity(Eigen::Vector3d(0, 0, 0));

  {
    dyn::RevoluteJoint::Properties properties;
    properties.mName = "shoulder_joint";
    properties.mAxis = Eigen::Vector3d::UnitX();
    properties.mT_ChildBodyToJoint.translation() =
        -config_.shoulder.pose_mm * 0.001;

    std::tie(shoulder_joint_, shoulder_body_) =
        skel_->createJointAndBodyNodePair<dyn::RevoluteJoint>(
            nullptr, properties,
            dyn::BodyNode::AspectProperties("shoulder"));
    shoulder_body_->setMass(0.01);
  }
  {
    dyn::RevoluteJoint::Properties properties;
    properties.mName = "femur_joint";
    properties.mAxis = Eigen::Vector3d::UnitY();
    properties.mT_ChildBodyToJoint.translation() =
        -config_.femur.pose_mm * 0.001;

    std::tie(femur_joint_, femur_body_) =
        skel_->createJointAndBodyNodePair<dyn::RevoluteJoint>(
            shoulder_body_, properties,
            dyn::BodyNode::AspectProperties("femur"));
    femur_body_->setMass(0.01);
  }
  {
    dyn::RevoluteJoint::Properties properties;
    properties.mName = "tibia_joint";
    properties.mAxis = Eigen::Vector3d::UnitY();
    properties.mT_ChildBodyToJoint.translation() =
        -config_.tibia.pose_mm * 0.001;

    std::tie(tibia_joint_, tibia_body_) =
        skel_->createJointAndBodyNodePair<dyn::RevoluteJoint>(
            femur_body_, properties,
            dyn::BodyNode::AspectProperties("tibia"));
    tibia_body_->setMass(0.01);
  }

  {
    dyn::WeldJoint::Properties properties;
    properties.mName = "foot_joint";
    foot_body_ =
        skel_->createJointAndBodyNodePair<dyn::WeldJoint>(
            tibia_body_, properties,
            dyn::BodyNode::AspectProperties("foot")).second;
    // We set this to 1e-6 so that it swamps all other masses.
    // We're only using this to back-out how much force the
    // end-effector is producing.
    foot_body_->setMass(1e6);
  }
}

IkSolver::Effector MammalIk::Forward_G(const JointAngles& angles) const {
  auto get_id = [&](int id) {
    for (const auto& joint : angles) {
      if (joint.id == id) { return joint; }
    }
    mjlib::base::AssertNotReached();
  };

  const auto& shoulder = get_id(config_.shoulder.id);
  const auto& femur = get_id(config_.femur.id);
  const auto& tibia = get_id(config_.tibia.id);

  auto set_joint = [](auto& dart_joint, auto& mjoint) {
    dart_joint->setPosition(0, base::Radians(mjoint.angle_deg));
    dart_joint->setVelocity(0, base::Radians(mjoint.velocity_dps));
    dart_joint->setForce(0, 0.0);
  };
  set_joint(shoulder_joint_, shoulder);
  set_joint(femur_joint_, femur);
  set_joint(tibia_joint_, tibia);

  skel_->computeForwardKinematics();
  skel_->computeForwardDynamics();

  Effector result_G;
  result_G.pose_mm = foot_body_->getCOM() * 1000;
  result_G.velocity_mm_s = foot_body_->getCOMLinearVelocity() * 1000;

  // No torque acceleration.
  Eigen::Vector3d no_torque_accel =
      foot_body_->getCOMLinearAcceleration();


  shoulder_joint_->setForce(0, shoulder.torque_Nm);
  femur_joint_->setForce(0, femur.torque_Nm);
  tibia_joint_->setForce(0, tibia.torque_Nm);

  skel_->computeForwardKinematics();
  skel_->computeForwardDynamics();

  Eigen::Vector3d torque_accel =
      foot_body_->getCOMLinearAcceleration();

  result_G.force_N = (torque_accel - no_torque_accel) * 1e6;

  return result_G;
}

IkSolver::InverseResult MammalIk::Inverse(
    const Effector& effector_G,
    const std::optional<JointAngles>& current) const {
  const auto& point = effector_G.pose_mm;
  const double r = config_.shoulder.pose_mm.y();

  // Find the angle of the shoulder joint.  This will be the tangent
  // angle between the point and the circle with radius
  // femur_attachment_mm.y

  const auto lim1 = [](double value) {
    return mjlib::base::Limit(value, -1.0, 1.0);
  };

  const auto shoulder_rad = [&]() -> std::optional<double> {
    // Define a 2D coordinate system looking behind the shoulder joint
    // with +x to the right and +y up.
    const double x0 = point.y();
    const double y0 = -point.z();

    if (std::abs(r) < 1e-3) {
      // We are close enough to centered.  Just do the math directly.
      return -(std::atan2(y0, x0) + 0.5 * M_PI);
    }

    // From: http://mathworld.wolfram.com/CircleTangentLine.html

    const double squared = std::pow(x0, 2) + std::pow(y0, 2) - std::pow(r, 2);
    if (squared <= 0.0) {
      // The point is inside our shoulder's rotating radius.  We
      // cannot position ourselves.
      return {};
    }

    const double denom = (std::pow(x0, 2) + std::pow(y0, 2));
    const double subexp = y0 * std::sqrt(squared) / denom;

    const double t1 = std::acos(lim1(-r * x0 / denom + subexp));
    const double t2 = std::acos(lim1(-r * x0 / denom - subexp));

    // Possible solutions are +-t1 or +-t2, only of which 2 will
    // actually result in a tangent line.  We only need the tangent
    // line on the same side as our shoulder point.

    const double possible_lines[] = { t1, -t1, t2, -t2 };

    std::optional<double> best_theta;
    std::optional<double> best_distance_sq;

    for (auto theta : possible_lines) {
      // From: http://mathworld.wolfram.com/Circle-LineIntersection.html
      const double dx = std::sin(theta);
      const double dy = std::cos(theta);
      const double D = x0 * (y0 + dy) - (x0 + dx) * y0;
      // We don't need to square these, since dr == 1
      const double discriminant = std::abs(r) - std::abs(D);
      if (std::abs(discriminant) > 1e-4) { continue; }

      const double x_int = D * dy;
      const double y_int = -D * dx;
      const double distance = std::pow(x_int - r, 2) + std::pow(y_int, 2);
      if (!best_distance_sq || distance < *best_distance_sq) {
        best_distance_sq = distance;
        best_theta = std::atan2(0, r) - std::atan2(y_int, x_int);
      }
    }

    BOOST_ASSERT(!!best_theta);

    return base::WrapNegPiToPi(*best_theta);
  }();

  if (!shoulder_rad) { return {}; }

  const auto maybe_femur_tibia_rad =
      [&]() -> std::optional<std::pair<double, double>> {

    // Given this shoulder angle, find the center of the leg plane in
    // the joint reference system.
    const double leg_frame_y = r * std::cos(-*shoulder_rad);
    const double leg_frame_z = r * std::sin(-*shoulder_rad);

    // Now we project the the point into the leg plane.
    const double point_y =
        std::sqrt(std::pow(point.y() - leg_frame_y, 2) +
                  std::pow(-point.z() - leg_frame_z, 2)) *
        base::GetSign(-point.z() - leg_frame_z);

    // This 2D frame is looking through the femur along its +y axis,
    // with x pointed to the right, and y pointed up.
    const double prx = -(point.x()  - config_.shoulder.pose_mm.x());
    const double pry = point_y + config_.shoulder.pose_mm.z();

    // Now we have a simple triangle, with the femur attachment point
    // at the origin, and the two sides being the femur and tibia.
    //
    //                      O
    //                     /  <-- femur
    //                    /
    //                   /
    //                   -------P
    //
    //                      ^tibia
    //
    // We know the length of all three sides, which means we can find
    // all three angles.

    const double femur_length = config_.femur.pose_mm.z();
    const double tibia_length = config_.tibia.pose_mm.z();

    // Use the law of cosines to find the tibia angle first.
    const double op_sq = prx * prx + pry * pry;
    const double cos_tibia_sub =
        std::pow(femur_length, 2) +
        std::pow(tibia_length, 2) -
        op_sq;

    if (std::sqrt(op_sq) > (femur_length + tibia_length)) {
      return {};
    }

    const double cos_tibiainv =
        cos_tibia_sub /
        (2 * femur_length * tibia_length);
    const double tibiainv_rad = std::acos(lim1(cos_tibiainv));
    const double tibia_sign = config_.invert ? 1.0 : -1.0;
    const double logical_tibia_rad = tibia_sign * (M_PI - tibiainv_rad);

    // Now we can solve for the femur.
    const double cos_femur_sub =
        op_sq + std::pow(femur_length, 2) -
        std::pow(tibia_length, 2);

    const double cos_femur_1 = cos_femur_sub /
        (2 * std::sqrt(op_sq ) * femur_length);
    const double femur_1_rad = std::acos(lim1(cos_femur_1));

    const double femur_sign = config_.invert ? -1.0 : 1.0;
    const double femur_rad =
        base::WrapNegPiToPi(
            -(std::atan2(pry, prx) + 0.5 * M_PI) + femur_sign * femur_1_rad);

    return std::make_pair(femur_rad, logical_tibia_rad);
  }();

  if (!maybe_femur_tibia_rad) { return {}; }


  // Now that we have the position, lets run the DART forward dynamics
  // and get the Jacobian out so we can determine our velocities.
  auto set_joint = [](auto& dart_joint, double value) {
    dart_joint->setPosition(0, value);
    dart_joint->setVelocity(0, 0.0);
    dart_joint->setForce(0, 0.0);
  };
  set_joint(shoulder_joint_, *shoulder_rad);
  set_joint(femur_joint_, maybe_femur_tibia_rad->first);
  set_joint(tibia_joint_, maybe_femur_tibia_rad->second);

  skel_->computeForwardKinematics();
  skel_->computeForwardDynamics();

  auto linear_jacobian = foot_body_->getLinearJacobian();
  // Since we only have 3 joints, this should be a 3x3 matrix.
  BOOST_ASSERT(linear_jacobian.cols() == 3);

  // This is a tiny 3x3 matrix, so we'll just invert it directly.
  const Eigen::Vector3d joint_dps =
      linear_jacobian.inverse() * (effector_G.velocity_mm_s * 0.001);

  // Assemble our result with everything but torque.
  JointAngles result;

  result.push_back(
      Joint()
      .set_id(config_.shoulder.id)
      .set_angle_deg(base::Degrees(*shoulder_rad))
      .set_velocity_dps(base::Degrees(joint_dps.x()))
  );
  result.push_back(
      Joint()
      .set_id(config_.femur.id)
      .set_angle_deg(base::Degrees(maybe_femur_tibia_rad->first))
      .set_velocity_dps(base::Degrees(joint_dps.y()))
  );
  result.push_back(
      Joint()
      .set_id(config_.tibia.id)
      .set_angle_deg(base::Degrees(maybe_femur_tibia_rad->second))
      .set_velocity_dps(base::Degrees(joint_dps.z()))
  );

  // Now we do force.  If we have it, use the joint angles provided,
  // otherwise, use those we just calculated.
  const JointAngles* joints_for_force = (!!current ? &*current : &result);

  auto get_id = [&](int id) {
    for (const auto& joint : *joints_for_force) {
      if (joint.id == id) { return joint; }
    }
    mjlib::base::AssertNotReached();
  };

  set_joint(shoulder_joint_, base::Radians(get_id(config_.shoulder.id).angle_deg));
  set_joint(femur_joint_, base::Radians(get_id(config_.femur.id).angle_deg));
  set_joint(tibia_joint_, base::Radians(get_id(config_.tibia.id).angle_deg));

  // DART doesn't provide a way to get the acceleration Jacobian
  // w.r.t. joint torques.  Thus we numerically calculate it.  First,
  // get the baseline w/ no torque.

  skel_->computeForwardKinematics();
  skel_->computeForwardDynamics();

  const Eigen::Vector3d baseline = foot_body_->getCOMLinearAcceleration();

  // Then solve for each axis separately.
  Eigen::Matrix3d acceleration_force_jacobian;
  for (int axis = 0; axis < 3; axis++) {
    shoulder_joint_->setForce(0, axis == 0 ? 1.0 : 0);
    femur_joint_->setForce(0, axis == 1 ? 1.0 : 0);
    tibia_joint_->setForce(0, axis == 2 ? 1.0 : 0);

    skel_->computeForwardKinematics();
    skel_->computeForwardDynamics();

    acceleration_force_jacobian.col(axis) =
        (foot_body_->getCOMLinearAcceleration() - baseline);
  }

  // Now we invert that matrix to determine the joint torques required
  // for the force we want.  This is just a 3x3, so directly
  // calculating the inverse should be fine.
  const Eigen::Vector3d joint_torque =
      (acceleration_force_jacobian.inverse() * effector_G.force_N) * 1e-6;

  // Now stick our torques into our result vector.
  for (auto& rj : result) {
    if (rj.id == config_.shoulder.id) { rj.set_torque_Nm(joint_torque(0)); }
    if (rj.id == config_.femur.id) { rj.set_torque_Nm(joint_torque(1)); }
    if (rj.id == config_.tibia.id) { rj.set_torque_Nm(joint_torque(2)); }
  }

  return result;
}

}
}
