// Copyright 2015 Josh Pieper, jjp@pobox.com.  All rights reserved.
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

#include "simulator_window.h"

using namespace dart::dynamics;
using namespace dart::simulation;

namespace {
SkeletonPtr createFloor() {
  SkeletonPtr floor = Skeleton::create("floor");

  // Give the floor a body
  BodyNodePtr body =
      floor->createJointAndBodyNodePair<WeldJoint>(nullptr).second;

  // Give the body a shape
  double floor_width = 3.0;
  double floor_height = 0.01;
  std::shared_ptr<BoxShape> box(
        new BoxShape(Eigen::Vector3d(floor_width, floor_width, floor_height)));
  box->setColor(dart::Color::Black());

  body->addVisualizationShape(box);
  body->addCollisionShape(box);

  // Put the body into position
  Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
  tf.translation() = Eigen::Vector3d(0.0, 0.0, -floor_height / 2.0);
  body->getParentJoint()->setTransformFromParentBodyNode(tf);

  return floor;
}

void setShape(const BodyNodePtr& bn, const BoxShapePtr& box,
              const Eigen::Vector3d& axis) {
  // Always create a visualization shape with no local transform so we
  // can figure out where things are.
  auto joint = std::make_shared<BoxShape>(
      Eigen::Vector3d(0.001, 0.001, 0.001));
  joint->setColor(dart::Color::Red());
  bn->addVisualizationShape(joint);


  box->setColor(dart::Color::Blue());

  // Set the location of the Box
  Eigen::Isometry3d box_tf(Eigen::Isometry3d::Identity());
  auto size = box->getSize();
  Eigen::Vector3d center = 0.5 * size.cwiseProduct(axis);
  box_tf.translation() = center;
  box->setLocalTransform(box_tf);

  // Add it as a visualization and collision shape
  bn->addVisualizationShape(box);
  bn->addCollisionShape(box);

  // Move the center of mass to the center of the object
  bn->setLocalCOM(center);
}

BodyNodePtr makeLegJoint(SkeletonPtr skel, BodyNodePtr parent,
                         const std::string& name,
                         const Eigen::Vector3d& shape,
                         const Eigen::Vector3d& axis,
                         const Eigen::Vector3d& offset) {
  RevoluteJoint::Properties properties;
  properties.mName = name + "_joint";
  properties.mT_ParentBodyToJoint.translation() = offset;

  auto joint = skel->createJointAndBodyNodePair<RevoluteJoint>(
      parent, properties,
      BodyNode::Properties(name)).second;

  auto box = std::make_shared<BoxShape>(shape);
  setShape(joint, box, axis);

  return joint;
}

BodyNodePtr makeLeg(SkeletonPtr skel, BodyNodePtr parent,
                    const Eigen::Vector3d& offset,
                    int left, const std::string& name) {
  auto coxa = makeLegJoint(skel, parent, name + "_coxa",
                           Eigen::Vector3d(0.04, 0.02, 0.02),
                           Eigen::Vector3d(0., -1. * left, 0.),
                           offset);
  auto femur = makeLegJoint(skel, coxa, name + "_femur",
                            Eigen::Vector3d(0.025, 0.041, 0.095),
                            Eigen::Vector3d(0.0, 0.0, 1.0),
                            Eigen::Vector3d(0.00, -0.04 * left, 0.015));
  auto tibia = makeLegJoint(skel, femur, name + "_tibia",
                            Eigen::Vector3d(0.025, 0.041, 0.105),
                            Eigen::Vector3d(0.0, 0.0, 1.0),
                            Eigen::Vector3d(0.0, 0.0, 0.095));

  return coxa;
}

SkeletonPtr createMech() {
  BallJoint::Properties properties;
  properties.mName = "mech_joint";

  SkeletonPtr result = Skeleton::create("mech");

  auto body = result->createJointAndBodyNodePair<BallJoint>(
      nullptr, properties,
      BodyNode::Properties(std::string("body"))).second;

  auto box = std::make_shared<BoxShape>(
      Eigen::Vector3d(0.210, 0.128, .0332));

  setShape(body, box, Eigen::Vector3d(0., 0., 0.));

  auto leg_lf = makeLeg(
      result, body, Eigen::Vector3d(0.09, -0.062, 0.0), 1, "lf");
  auto leg_rf = makeLeg(
      result, body, Eigen::Vector3d(0.09, 0.062, 0.0), -1, "rf");
  auto leg_lr = makeLeg(
      result, body, Eigen::Vector3d(-0.09, -0.062, 0.0), 1, "lr");
  auto leg_rr = makeLeg(
      result, body, Eigen::Vector3d(-0.09, 0.062, 0.0), -1, "rr");

  return result;
}
}

namespace mjmech {
namespace simulator {

SimulatorWindow::SimulatorWindow() {
  auto world = std::make_shared<World>();

  world->addSkeleton(createFloor());

  mech_ = createMech();
  world->addSkeleton(mech_);

  setWorld(world);
}

SimulatorWindow::~SimulatorWindow() {}

}
}
