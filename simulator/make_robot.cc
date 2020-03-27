// Copyright 2015-2020 Josh Pieper, jjp@pobox.com.
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

#include "simulator/make_robot.h"

#include <dart/dynamics/BoxShape.hpp>
#include <dart/dynamics/CylinderShape.hpp>
#include <dart/dynamics/DegreeOfFreedom.hpp>
#include <dart/dynamics/FreeJoint.hpp>
#include <dart/dynamics/RevoluteJoint.hpp>
#include <dart/dynamics/WeldJoint.hpp>

namespace dd = dart::dynamics;

namespace mjmech {
namespace simulator {

namespace {
constexpr double kBodyMassKg = 2.0;

Sophus::SE3d ConvertSE3MmToM(const Sophus::SE3d& input) {
  auto result = input;
  result.translation() *= 0.001;
  return result;
}

Eigen::Vector3d ConvertMmToM(const Eigen::Vector3d& input) {
  return 0.001 * input;
}

dd::BodyNodePtr MakeLegJoint(
    dd::SkeletonPtr skel,
    dd::BodyNodePtr parent,
    const std::string& name,
    dd::ShapePtr shape,
    const Sophus::SE3d& shape_m,
    const Sophus::SE3d& pose_m,
    const Eigen::Vector3d& axis,
    double mass_kg) {
  dd::RevoluteJoint::Properties properties;
  properties.mName = name + "_joint";
  properties.mT_ParentBodyToJoint = pose_m.matrix();
  properties.mAxis = axis;

  auto joint = skel->createJointAndBodyNodePair<dd::RevoluteJoint>(
      parent, properties, dd::BodyNode::AspectProperties(name)).second;

  auto shape_node =
      joint->createShapeNodeWith<
        dd::VisualAspect, dd::CollisionAspect, dd::DynamicsAspect>(shape);
  shape_node->setRelativeTransform(Eigen::Isometry3d(shape_m.matrix()));
  shape_node->getVisualAspect()->setColor(dart::Color::Blue());

  // TODO: Set the location.
  dart::dynamics::Inertia inertia;
  inertia.setMass(mass_kg);
  inertia.setMoment(5 * shape->computeInertia(mass_kg));
  joint->setInertia(inertia);

  return joint;
}

dd::BodyNodePtr MakeLeg(dd::SkeletonPtr skel,
                        dd::BodyNodePtr parent,
                        const mech::QuadrupedConfig::Leg& leg_config,
                        const std::string& name) {
  Eigen::Vector3d shoulder_pose_m = ConvertMmToM(leg_config.ik.shoulder.pose_mm);
  auto shoulder = MakeLegJoint(
      skel,
      parent,
      name + "_shoulder",
      std::make_shared<dd::CylinderShape>(0.05, 0.11),
      {Eigen::Quaterniond(
            Eigen::AngleAxisd(0.5 * M_PI, Eigen::Vector3d::UnitX())),
            shoulder_pose_m +
            Eigen::DiagonalMatrix<double, 3>(0, -.5, 0) * shoulder_pose_m},
      ConvertSE3MmToM(leg_config.pose_mm_BG),
      {1.0, 0., 0.},
      0.5);

  auto femur = MakeLegJoint(
      skel,
      shoulder,
      name + "_femur",
      std::make_shared<dd::BoxShape>(
          Eigen::Vector3d(
              0.07, 0.04, 0.001 * leg_config.ik.femur.pose_mm.z())),
      {Eigen::Quaterniond::Identity(),
            Eigen::Vector3d(
                0.0, 0.0, 0.5 * 0.001 * leg_config.ik.femur.pose_mm.z())},
      ConvertSE3MmToM(
          {Eigen::Quaterniond(
                Eigen::AngleAxis(M_PI, Eigen::Vector3d::UnitX())),
                leg_config.ik.shoulder.pose_mm}),
      {0., 1., 0.},
      0.7);

  auto tibia = MakeLegJoint(
      skel,
      femur,
      name + "_tibia",
      std::make_shared<dd::CylinderShape>(
          0.01, 0.001 * leg_config.ik.tibia.pose_mm.z()),
      {Eigen::Quaterniond::Identity(),
            Eigen::Vector3d(
                0.0, 0.0, 0.5 * 0.001 * leg_config.ik.tibia.pose_mm.z())},
      ConvertSE3MmToM(
          {Eigen::Quaterniond::Identity(), leg_config.ik.femur.pose_mm}),
      {0., 1., 0.},
      0.3);

  {
    // Do the foot.
    dd::WeldJoint::Properties weld_properties;
    weld_properties.mName = name + "_foot_joint";
    weld_properties.mT_ParentBodyToJoint.translation() =
        0.001 * leg_config.ik.tibia.pose_mm;

    auto foot = skel->createJointAndBodyNodePair<dd::WeldJoint>(
        tibia, weld_properties,
        dd::BodyNode::AspectProperties(name + "_foot")).second;
    auto shape = std::make_shared<dd::EllipsoidShape>(
        Eigen::Vector3d(0.04, 0.04, 0.04));
    auto foot_shape_node = foot->createShapeNodeWith<
      dd::VisualAspect, dd::CollisionAspect, dd::DynamicsAspect>(shape);
    foot_shape_node->getVisualAspect()->setColor(dart::Color::Black());

    dart::dynamics::Inertia inertia;
    constexpr double foot_mass_kg = 0.1;
    inertia.setMass(foot_mass_kg);
    inertia.setMoment(5 * shape->computeInertia(foot_mass_kg));
    foot->setInertia(inertia);
    foot_shape_node->getDynamicsAspect()->setFrictionCoeff(0.25);
    foot_shape_node->getDynamicsAspect()->setRestitutionCoeff(0.4);
  }

  return shoulder;
}

}  // namespace

dd::SkeletonPtr MakeRobot(const mech::QuadrupedConfig& config) {
  auto result = dd::Skeleton::create("robot_skel");

  auto body =
      result->createJointAndBodyNodePair<dd::FreeJoint>(
          nullptr, dd::FreeJoint::Properties(),
          dd::BodyNode::AspectProperties("robot")).second;

  auto box =
      std::make_shared<dd::BoxShape>(Eigen::Vector3d(0.230, 0.245, 0.140));
  body->createShapeNodeWith<
    dd::VisualAspect, dd::CollisionAspect, dd::DynamicsAspect>(box);

  dart::dynamics::Inertia inertia;
  inertia.setMass(kBodyMassKg);
  inertia.setMoment(box->computeInertia(kBodyMassKg));
  body->setInertia(inertia);

  result->getDof("Joint_pos_z")->setPosition(0.5);

  auto leg0 = MakeLeg(result, body, config.legs.at(0), "leg0");
  auto leg1 = MakeLeg(result, body, config.legs.at(1), "leg1");
  auto leg2 = MakeLeg(result, body, config.legs.at(2), "leg2");
  auto leg3 = MakeLeg(result, body, config.legs.at(3), "leg3");

  return result;
}

dd::SkeletonPtr MakeFloor() {
  auto floor = dd::Skeleton::create("floor");
  auto body = floor->createJointAndBodyNodePair<dd::WeldJoint>(nullptr).second;

  double floor_width = 10.0;
  double floor_height = 0.01;
  auto box = std::make_shared<dd::BoxShape>(
      Eigen::Vector3d(floor_width, floor_width, floor_height));
  auto shape_node = body->createShapeNodeWith<
    dd::VisualAspect, dd::CollisionAspect, dd::DynamicsAspect>(box);
  shape_node->getVisualAspect()->setColor(dart::Color::Black());

  // Put the body into position.
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = Eigen::Vector3d(0., 0., -0.5 * floor_height);
  body->getParentJoint()->setTransformFromParentBodyNode(tf);

  return floor;
}

}
}
