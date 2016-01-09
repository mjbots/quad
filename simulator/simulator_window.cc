// Copyright 2015-2016 Josh Pieper, jjp@pobox.com.  All rights reserved.
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
// * Link in mech C++ class
//    * simulate IMU
// * Add turret model
// * Refine mass and moment of inertia of each joint with real robot
// * Simulate torque on/off for each servo
// * Strip out the unecessary tutorial-level SimWindow save/replay
//   functionality.

#include "simulator_window.h"

#include <boost/filesystem.hpp>

#include "base/common.h"
#include "base/concrete_comm_factory.h"
#include "base/context_full.h"
#include "base/debug_deadline_service.h"
#include "base/fail.h"
#include "base/now.h"
#include "base/program_options.h"
#include "base/program_options_archive.h"

#include "mech/mech_warfare.h"

#include "herkulex_protocol.h"

using namespace dart::dynamics;
using namespace dart::simulation;
using namespace mjmech;
namespace po = boost::program_options;

namespace {
const double kConvert_kgf_cm_to_N_m = 10.197162;
const double kServo_kp = 10.0;
const double kServo_kd = 0.1;

double Limit(double value, double limit) {
  return (value > limit) ? limit : ((value < -limit) ? -limit : value);
}

class ServoController {
 public:
  ServoController(const SkeletonPtr& skel,
                  Joint* joint,
                  int dof_number) :
      skeleton_(skel),
      dof_number_(dof_number) {

    joint->setVelocityUpperLimit(0, base::Radians(360));
    joint->setDampingCoefficient(0, 0.2);
    joint->setCoulombFriction(0, 0.2);
  }

  void SetPosition(double angle_rad) {
    desired_rad_ = angle_rad * sign_;
  }

  void SetSign(double sign) {
    sign_ = sign;
  }

  void Update() {
    const double position = skeleton_->getPosition(dof_number_);
    const double velocity = skeleton_->getVelocity(dof_number_);

    const double error = base::WrapNegPiToPi(position - desired_rad_);
    const double kMaxForce_kgf_cm = 16.0;
    const double kLimit = kMaxForce_kgf_cm / kConvert_kgf_cm_to_N_m;
    const double control =
        Limit(error * kServo_kp + velocity * kServo_kd, kLimit);

    skeleton_->setForce(dof_number_, -control);
  }

 private:
  SkeletonPtr skeleton_;
  const int dof_number_;
  double desired_rad_ = 0.0;
  double sign_ = 1.0;
};

typedef std::shared_ptr<ServoController> ServoControllerPtr;

void SetMass(BodyNodePtr body, double mass_kg) {
  BOOST_ASSERT(body->getNumCollisionShapes() == 1);

  Inertia inertia;
  inertia.setMass(mass_kg);

  auto shape = body->getCollisionShape(0);
  auto size = shape->getBoundingBoxDim();
  const Eigen::Vector3d center = 0.5 * size;

  inertia.setLocalCOM(center);

  // TODO jpieper: Most of our solids are actually hollow inside, or
  // have uneven weight distribution because they have a heavy servo
  // at one end.  Rather than actually model that at the moment, just
  // add a fudge factor.
  const double kFudge = 5.0;

  inertia.setMoment(kFudge * shape->computeInertia(inertia.getMass()));

  body->setInertia(inertia);
}

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
              const Eigen::Vector3d& axis,
              double mass_kg) {
  // Always create a visualization shape with no local transform so we
  // can figure out where things are.
  auto joint = std::make_shared<BoxShape>(
      Eigen::Vector3d(0.001, 0.001, 0.001));
  joint->setColor(dart::Color::Red());
  bn->addVisualizationShape(joint);


  box->setColor(dart::Color::Blue());

  // Set the location of the Shape.
  Eigen::Isometry3d box_tf = box->getLocalTransform();
  auto size = box_tf.rotation() * box->getBoundingBoxDim();
  const Eigen::Vector3d ref_point = 0.5 * size.cwiseProduct(axis);
  box_tf.translation() += ref_point;
  box->setLocalTransform(box_tf);

  // Add it as a visualization and collision shape
  bn->addVisualizationShape(box);
  bn->addCollisionShape(box);

  SetMass(bn, mass_kg);
}

BodyNodePtr makeLegJoint(SkeletonPtr skel, BodyNodePtr parent,
                         const std::string& name,
                         const ShapePtr& shape,
                         const Eigen::Vector3d& axis,
                         const Eigen::Vector3d& rotation_axis,
                         const Eigen::Vector3d& offset,
                         double mass_kg) {
  RevoluteJoint::Properties properties;
  properties.mName = name + "_joint";
  properties.mT_ParentBodyToJoint.translation() = offset;
  properties.mAxis = rotation_axis;

  auto joint = skel->createJointAndBodyNodePair<RevoluteJoint>(
      parent, properties,
      BodyNode::Properties(name)).second;

  setShape(joint, shape, axis, mass_kg);

  return joint;
}

mjmech::simulator::SimulatorWindow::Impl* g_impl = nullptr;

}

static void HandleGlutTimer(int);

namespace mjmech {
namespace simulator {

class SimulatorWindow::Impl {
 public:
  Impl()
      : context_(),
        debug_deadline_service_(
            base::DebugDeadlineService::Install(context_.service)) {

    world_ = std::make_shared<World>();

    debug_deadline_service_->SetTime(
        boost::posix_time::microsec_clock::universal_time());
    g_impl = this;

    mech_warfare_.reset(new mech::MechWarfare(context_));

    options_description_.add_options()
        ("disable-mech", po::bool_switch(&disable_mech_),
         "do not start the mech instance")
        ("start-disabled", po::bool_switch(&start_disabled_),
         "begin in paused mode")
        ("turret-enabled", po::bool_switch(&turret_enabled_),
         "include turret model")
        ;

    base::MergeProgramOptions(stream_config_.options_description(),
                              "stream.",
                              &options_description_);
    base::ProgramOptionsArchive archive(&options_description_, "mech.");
    archive.Accept(mech_warfare_->parameters());

    // Prepare our default configuration.
    auto key = [&](const std::string& value) {
      return options_description_.find(value, false).semantic();
    };

    key("stream.type")->notify(std::string("pipe"));
    key("stream.pipe.key")->notify(std::string("bus"));
    key("stream.pipe.mode")->notify(1u);
    key("mech.servo_base.stream.type")->notify(std::string("pipe"));
    key("mech.servo_base.stream.pipe.key")->notify(std::string("bus"));
    boost::filesystem::path this_file(__FILE__);
    boost::filesystem::path root = this_file.parent_path().parent_path();
    key("mech.gait_config")->notify((root / "configs" / "real.cfg").string());
    key("mech.imu_enable")->notify(false);
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
        offset,
        0.02);
    auto femur = makeLegJoint(
        skel, coxa, name + "_femur",
        std::make_shared<BoxShape>(Eigen::Vector3d(0.025, 0.041, 0.095)),
        Eigen::Vector3d(0.0, 0.0, 1.0),
        Eigen::Vector3d(0.0, 1.0, 0.0),
        Eigen::Vector3d(0.00, -0.04 * left, 0.015),
        0.09);
    auto tibia = makeLegJoint(
        skel, femur, name + "_tibia",
        std::make_shared<BoxShape>(Eigen::Vector3d(0.01, 0.01, 0.090)),
        Eigen::Vector3d(0.0, 0.0, 1.0),
        Eigen::Vector3d(0.0, 1.0, 0.0),
        Eigen::Vector3d(0.0, 0.0, 0.095),
        0.09);

    WeldJoint::Properties weld_properties;
    weld_properties.mName = name + "_foot_joint";
    weld_properties.mT_ParentBodyToJoint.translation() =
        Eigen::Vector3d(0., 0., 0.090);

    auto foot = skel->createJointAndBodyNodePair<WeldJoint>(
        tibia, weld_properties, BodyNode::Properties(name + "_foot")).second;
    setShape(
        foot,
        std::make_shared<EllipsoidShape>(Eigen::Vector3d(0.03, 0.03, 0.03)),
        Eigen::Vector3d(0., 0., 0.),
        0.02);
    foot->setRestitutionCoeff(0.4);
    foot->setFrictionCoeff(0.5);

    const std::size_t count = skel->getNumDofs();
    BOOST_ASSERT(count >= 3);
    const std::size_t snum_start = servos_.size();
    servos_[snum_start + 0] =
        std::make_shared<ServoController>(
            skel, coxa->getParentJoint(), count - 3);
    servos_[snum_start + 1] =
        std::make_shared<ServoController>(
            skel, femur->getParentJoint(), count - 2);
    servos_[snum_start + 2] =
        std::make_shared<ServoController>(
            skel, tibia->getParentJoint(), count - 1);

    return coxa;
  }

  void CreateTurret(SkeletonPtr skeleton, BodyNodePtr body) {
    auto turret_yaw =
        makeLegJoint(skeleton, body, "turret_yaw",
                     std::make_shared<CylinderShape>(0.021, 0.021),
                     Eigen::Vector3d(0.0, 0.0, -1.0),
                     Eigen::Vector3d(0.0, 0.0, 1.0),
                     Eigen::Vector3d(0.0, 0.0, -0.0157),
                     0.02);

    const std::size_t snum_start = servos_.size();
    servos_[snum_start + 0] =
        std::make_shared<ServoController>(
            skeleton, turret_yaw->getParentJoint(),
            skeleton->getNumDofs() - 1);

    auto gimbal_base =
        std::make_shared<BoxShape>(Eigen::Vector3d(0.040, 0.0695, 0.003));
    gimbal_base->setLocalTransform(
        Eigen::Isometry3d(
            Eigen::Translation3d(
                Eigen::Vector3d(0.0, 0.0, -0.021 - 0.5 * 0.003))));

    turret_yaw->addVisualizationShape(gimbal_base);

    auto add_arm = [&](double side) {
      auto arm = std::make_shared<BoxShape>(Eigen::Vector3d(0.040, 0.003, 0.117));
      arm->setLocalTransform(
          Eigen::Isometry3d(
              Eigen::Translation3d(
                  Eigen::Vector3d(
                      0.0, side * 0.5 * 0.0695, -0.0225 - 0.003 - 0.5 * .117))));
      turret_yaw->addVisualizationShape(arm);
    };

    add_arm(-1.0);
    add_arm(1.0);

    auto pitch_cylinder =
        std::make_shared<CylinderShape>(0.021, 0.021);
    Eigen::Isometry3d tf = pitch_cylinder->getLocalTransform();
    tf.rotate(Eigen::AngleAxisd(0.5 * M_PI, Eigen::Vector3d(1., 0., 0.)));
    pitch_cylinder->setLocalTransform(tf);
    auto turret_pitch =
        makeLegJoint(skeleton, turret_yaw, "turret_pitch",
                     pitch_cylinder,
                     Eigen::Vector3d(0.0, -1.0, 0.0),
                     Eigen::Vector3d(0.0, 1.0, 0.0),
                     Eigen::Vector3d(0.0, -0.5 * 0.0695, -0.128),
                     0.02);

    // TODO jpieper: Put the turret mass at the correct location,
    // rather than off to the side and make it be the correct
    // magnitude.

    servos_[snum_start + 1] =
        std::make_shared<ServoController>(
            skeleton, turret_pitch->getParentJoint(),
            skeleton->getNumDofs() - 1);
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

    setShape(body, box, Eigen::Vector3d(0., 0., 0.), 1.0);

    // TODO jpieper: This is just a random fudge for the center of
    // mass of the mech, it kind of makes things work in simulation,
    // but I don't know how accurate it is in real life.
    auto center = 0.5 * box->getBoundingBoxDim();
    body->setLocalCOM(center + Eigen::Vector3d(-0.08, 0.0, 0.0));

    auto leg_rf = MakeLeg(
        result, body, Eigen::Vector3d(0.09, 0.062, 0.0), -1, "rf");
    auto leg_rr = MakeLeg(
        result, body, Eigen::Vector3d(-0.09, 0.062, 0.0), -1, "rr");
    auto leg_lr = MakeLeg(
        result, body, Eigen::Vector3d(-0.09, -0.062, 0.0), 1, "lr");
    auto leg_lf = MakeLeg(
        result, body, Eigen::Vector3d(0.09, -0.062, 0.0), 1, "lf");

    if (turret_enabled_) {
      CreateTurret(result, body);
    }

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
    tf.translation() = Eigen::Vector3d(0, 0, .21);
    result->getJoint(0)->setTransformFromParentBodyNode(tf);

    mech_ = result;
  }

  void timeStepping() {
    for (auto& pair: servos_) {
      pair.second->Update();
    }

    debug_deadline_service_->SetTime(
        base::Now(context_.service) +
        base::ConvertSecondsToDuration(world_->getTimeStep()));
    context_.service.poll();
    context_.service.reset();
  }

  void Start() {
    base::StreamHandler handler =
        std::bind(&Impl::HandleStart, this,
                  std::placeholders::_1, std::placeholders::_2);
    context_.factory->AsyncCreate(stream_config_, handler);

    if (!disable_mech_) {
      mech_warfare_->AsyncStart([](base::ErrorCode ec) {
          base::FailIf(ec);
        });
    }
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

  std::shared_ptr<dart::simulation::World> world_;
  dart::dynamics::SkeletonPtr mech_;
  int current_joint_ = 0;
  std::map<int, ServoControllerPtr> servos_;

  base::Context context_;
  base::DebugDeadlineService* const debug_deadline_service_;
  base::ConcreteStreamFactory::Parameters stream_config_;
  base::SharedStream stream_;

  char buffer_[256] = {};
  HerkulexOperations operations_{this};
  std::unique_ptr<HerkulexProtocol> herkulex_protocol_;

  boost::program_options::options_description options_description_;

  std::unique_ptr<mech::MechWarfare> mech_warfare_;
  bool disable_mech_ = false;
  bool start_disabled_ = false;
  bool turret_enabled_ = false;
};

SimulatorWindow::SimulatorWindow() : impl_(new Impl()) {
  impl_->world_->addSkeleton(createFloor());

  setWorld(impl_->world_);
}

SimulatorWindow::~SimulatorWindow() {
}

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
  return &impl_->options_description_;
}

void SimulatorWindow::Start() {
  impl_->CreateMech();
  impl_->world_->addSkeleton(impl_->mech_);

  impl_->Start();
  impl_->StartGlutTimer();

  if (!impl_->start_disabled_) {
    // Send a space bar to get us simulating.
    SimWindow::keyboard(' ', 0, 0);
  }
}

}
}

static void HandleGlutTimer(int value) {
  g_impl->Timer();
}
