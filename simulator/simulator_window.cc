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


// TODO jpieper:
// * Set correct masses for each joint and element, along with
//   appropriate servo constants.
// * Link in mech C++ class
// * Implement HerkuleX protocol simulator

#include "simulator_window.h"

#include "base/common.h"
#include "base/concrete_comm_factory.h"
#include "base/context.h"
#include "base/fail.h"

#include "herkulex_protocol.h"

using namespace dart::dynamics;
using namespace dart::simulation;
using namespace mjmech;

namespace {
const double kServo_kp = 100.0;
const double kServo_kd = 10.0;

class ServoController {
 public:
  ServoController(const SkeletonPtr& skel,
                  int joint_number) :
      skeleton_(skel),
      joint_number_(joint_number) {
  }

  void SetPosition(double angle_rad) {
    desired_rad_ = angle_rad * sign_;
  }

  void SetSign(double sign) {
    sign_ = sign;
  }

  void Update() {
    const double position = skeleton_->getPosition(joint_number_);
    const double velocity = skeleton_->getVelocity(joint_number_);

    const double error = base::WrapNegPiToPi(position - desired_rad_);
    const double control = error * kServo_kp + velocity * kServo_kd;

    skeleton_->setForce(joint_number_, -control);
  }

 private:
  SkeletonPtr skeleton_;
  const int joint_number_;
  double desired_rad_ = 0.0;
  double sign_ = 1.0;
};

typedef std::shared_ptr<ServoController> ServoControllerPtr;

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

void setShape(const BodyNodePtr& bn, const ShapePtr& box,
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
  auto size = box->getBoundingBoxDim();
  const Eigen::Vector3d ref_point = 0.5 * size.cwiseProduct(axis);
  box_tf.translation() = ref_point;
  box->setLocalTransform(box_tf);

  // Add it as a visualization and collision shape
  bn->addVisualizationShape(box);
  bn->addCollisionShape(box);

  const Eigen::Vector3d center = 0.5 * size;

  // Move the center of mass to the center of the object
  bn->setLocalCOM(center);
}

BodyNodePtr makeLegJoint(SkeletonPtr skel, BodyNodePtr parent,
                         const std::string& name,
                         const ShapePtr& shape,
                         const Eigen::Vector3d& axis,
                         const Eigen::Vector3d& rotation_axis,
                         const Eigen::Vector3d& offset) {
  RevoluteJoint::Properties properties;
  properties.mName = name + "_joint";
  properties.mT_ParentBodyToJoint.translation() = offset;
  properties.mAxis = rotation_axis;

  auto joint = skel->createJointAndBodyNodePair<RevoluteJoint>(
      parent, properties,
      BodyNode::Properties(name)).second;

  setShape(joint, shape, axis);

  return joint;
}

mjmech::simulator::SimulatorWindow::Impl* g_impl = nullptr;

}

static void HandleGlutTimer(int);

namespace mjmech {
namespace simulator {

class SimulatorWindow::Impl {
 public:
  Impl() {
    g_impl = this;
  }

  ~Impl() {
    g_impl = nullptr;
  }

  BodyNodePtr MakeLeg(SkeletonPtr skel, BodyNodePtr parent,
                      const Eigen::Vector3d& offset,
                      int left, const std::string& name) {
    auto coxa = makeLegJoint(
        skel, parent, name + "_coxa",
        std::make_shared<BoxShape>(Eigen::Vector3d(0.04, 0.02, 0.02)),
        Eigen::Vector3d(0., -1. * left, 0.),
        Eigen::Vector3d(1.0, 0.0, 0.0),
        offset);
    auto femur = makeLegJoint(
        skel, coxa, name + "_femur",
        std::make_shared<BoxShape>(Eigen::Vector3d(0.025, 0.041, 0.095)),
        Eigen::Vector3d(0.0, 0.0, 1.0),
        Eigen::Vector3d(0.0, 1.0, 0.0),
        Eigen::Vector3d(0.00, -0.04 * left, 0.015));
    auto tibia = makeLegJoint(
        skel, femur, name + "_tibia",
        std::make_shared<BoxShape>(Eigen::Vector3d(0.01, 0.01, 0.090)),
        Eigen::Vector3d(0.0, 0.0, 1.0),
        Eigen::Vector3d(0.0, 1.0, 0.0),
        Eigen::Vector3d(0.0, 0.0, 0.095));

    WeldJoint::Properties weld_properties;
    weld_properties.mName = name + "_foot_joint";
    weld_properties.mT_ParentBodyToJoint.translation() =
        Eigen::Vector3d(0., 0., 0.090);

    auto foot = skel->createJointAndBodyNodePair<WeldJoint>(
        tibia, weld_properties, BodyNode::Properties(name + "_foot")).second;
    setShape(
        foot,
        std::make_shared<EllipsoidShape>(Eigen::Vector3d(0.03, 0.03, 0.03)),
        Eigen::Vector3d(0., 0., 0.));

    const std::size_t count = skel->getNumDofs();
    BOOST_ASSERT(count >= 3);
    const std::size_t snum_start = servos_.size();
    servos_[snum_start + 0] =
        std::make_shared<ServoController>(skel, count - 3);
    servos_[snum_start + 1] =
        std::make_shared<ServoController>(skel, count - 2);
    servos_[snum_start + 2] =
        std::make_shared<ServoController>(skel, count - 1);

    return coxa;
  }

  void CreateMech() {
    FreeJoint::Properties properties;
    properties.mName = "mech_joint";

    SkeletonPtr result = Skeleton::create("mech");

    auto body = result->createJointAndBodyNodePair<FreeJoint>(
        nullptr, properties,
        BodyNode::Properties(std::string("body"))).second;

    auto box = std::make_shared<BoxShape>(
        Eigen::Vector3d(0.210, 0.128, .0332));

    setShape(body, box, Eigen::Vector3d(0., 0., 0.));

    auto leg_rf = MakeLeg(
        result, body, Eigen::Vector3d(0.09, 0.062, 0.0), -1, "rf");
    auto leg_rr = MakeLeg(
        result, body, Eigen::Vector3d(-0.09, 0.062, 0.0), -1, "rr");
    auto leg_lr = MakeLeg(
        result, body, Eigen::Vector3d(-0.09, -0.062, 0.0), 1, "lr");
    auto leg_lf = MakeLeg(
        result, body, Eigen::Vector3d(0.09, -0.062, 0.0), 1, "lf");

    // Fix up some signs so that the servos match what the actual mech
    // does.
    servos_[0]->SetSign(1.0);
    servos_[1]->SetSign(1.0);
    servos_[2]->SetSign(-1.0);

    servos_[3]->SetSign(-1.0);
    servos_[4]->SetSign(1.0);
    servos_[5]->SetSign(-1.0);

    servos_[6]->SetSign(-1.0);
    servos_[7]->SetSign(-1.0);
    servos_[8]->SetSign(1.0);

    servos_[9]->SetSign(1.0);
    servos_[10]->SetSign(-1.0);
    servos_[11]->SetSign(1.0);

    Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
    tf.rotate(Eigen::AngleAxisd(M_PI, Eigen::Vector3d(1., 0., 0.)));
    tf.translation() = Eigen::Vector3d(0, 0, .4);
    result->getJoint(0)->setTransformFromParentBodyNode(tf);

    mech_ = result;
  }

  void timeStepping() {
    for (auto& pair: servos_) {
      pair.second->Update();
    }
  }

  void Start() {
    base::StreamHandler handler =
        std::bind(&Impl::HandleStart, this,
                  std::placeholders::_1, std::placeholders::_2);
    context_.factory->AsyncCreate(stream_config_, handler);
  }

  void StartGlutTimer() {
    glutTimerFunc(50, &HandleGlutTimer, 0);
  }

  void Timer() {
    StartGlutTimer();
    context_.service.poll();
  }

  void HandleStart(base::ErrorCode ec,
                   base::SharedStream stream) {
    base::FailIf(ec);

    stream_ = stream;
    herkulex_protocol_.reset(new HerkulexProtocol(*stream_, operations_));
    herkulex_protocol_->AsyncStart([](base::ErrorCode ec) {
        FailIf(ec);
      });
  }

  class HerkulexOperations : public HerkulexProtocol::Operations {
   public:
    HerkulexOperations(Impl* parent) : parent_(parent) {}
    virtual ~HerkulexOperations() {}

    bool address_valid(int) const override { return true; }

    void SJog(const std::vector<ServoAngle>& angles) override {
      for (const auto& angle: angles) {
        auto it = parent_->servos_.find(angle.first);
        if (it != parent_->servos_.end()) {
          const double angle_deg = (angle.second - 512) * 0.325;
          const double angle_rad = base::Radians(angle_deg);
          it->second->SetPosition(angle_rad);
        }
      }
    }

    void Reboot(int servo) override {
      // Noop.
    }

    void WriteRam(int servo, uint8_t addr, uint8_t data) override {
      // Noop
    }

    uint8_t ReadRam(int servo, uint8_t addr) override {
      return 0;
    }

   private:
    Impl* const parent_;
  };

  dart::dynamics::SkeletonPtr mech_;
  int current_joint_ = 0;
  std::map<int, ServoControllerPtr> servos_;

  base::Context context_;
  base::ConcreteStreamFactory::Parameters stream_config_;
  base::SharedStream stream_;

  char buffer_[256] = {};
  HerkulexOperations operations_{this};
  std::unique_ptr<HerkulexProtocol> herkulex_protocol_;
};

SimulatorWindow::SimulatorWindow() : impl_(new Impl()) {
  auto world = std::make_shared<World>();

  world->addSkeleton(createFloor());

  impl_->CreateMech();
  world->addSkeleton(impl_->mech_);

  setWorld(world);
}

SimulatorWindow::~SimulatorWindow() {}

void SimulatorWindow::keyboard(unsigned char key, int x, int y) {
  auto move_joint = [&](double val) {
    impl_->mech_->setPosition(
        impl_->current_joint_,
        impl_->mech_->getPosition(impl_->current_joint_) + val);
    glutPostRedisplay();
  };
  if ((key >= '0' && key <= '9') ||
      (key >= 'a' && key <= 'h')) {
    const int joint =
        (key >= 'a' && key <= 'h') ? (key - 'a' + 10) : (key - '0');
    impl_->current_joint_ = joint;
  } else if (key == 'z') {
    move_joint(0.1);
  } else if (key == 'x') {
    move_joint(-0.1);
  } else {
    SimWindow::keyboard(key, x, y);
  }
}

void SimulatorWindow::timeStepping() {
  impl_->timeStepping();
  SimWindow::timeStepping();
}

void SimulatorWindow::render() {
  SimWindow::render();
}

boost::program_options::options_description*
SimulatorWindow::options_description() {
  return impl_->stream_config_.options_description();
}

void SimulatorWindow::Start() {
  impl_->Start();
  impl_->StartGlutTimer();
}

}
}

static void HandleGlutTimer(int value) {
  g_impl->Timer();
}
