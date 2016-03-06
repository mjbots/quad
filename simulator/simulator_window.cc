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
#include "base/debug_i2c_generator.h"
#include "base/fail.h"
#include "base/now.h"
#include "base/program_options.h"
#include "base/program_options_archive.h"

#include "mech/mech_warfare.h"

#include "herkulex_protocol.h"
#include "imu_simulation.h"

using namespace dart::dynamics;
using namespace dart::simulation;
using namespace mjmech;
namespace po = boost::program_options;

namespace {
const double kConvert_kgf_cm_to_N_m = 10.197162;
const double kServo_kp = 10.0;
const double kServo_kd = 0.1;

const double kGimbal_kp = 0.1;
const double kGimbalLimit = 10.0;

double Limit(double value, double limit) {
  return (value > limit) ? limit : ((value < -limit) ? -limit : value);
}

class ServoInterface : boost::noncopyable {
 public:
  virtual ~ServoInterface() {}

  virtual void SetPosition(double angle_rad) = 0;
  virtual void Reboot() = 0;
  virtual void WriteRam(uint8_t addr, uint8_t data) = 0;
  virtual uint8_t ReadRam(uint8_t addr) = 0;
  virtual void Update(double dt_s) = 0;
};

class ServoController : public ServoInterface {
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

  virtual ~ServoController() {}

  void SetPosition(double angle_rad) override {
    desired_rad_ = angle_rad * sign_;
  }

  void Reboot() override {
    // noop for now
  }

  void WriteRam(uint8_t addr, uint8_t data) override {
    if (addr == 52) {
      if (data == 0x40) {
        // brake
        SetFree();
      } else if (data == 0x60) {
        // on
        SetTorque();
      } else {
        // off
        SetFree();
      }
    };
  }

  uint8_t ReadRam(uint8_t addr) override {
    switch (addr) {
      case 48: {
        // status error
        return 0;
      }
      case 49: {
        return power_factor_ != 0.0 ? 0x40 : 0x00;
      }
      case 54: {
        // Voltage:
        return 100;
      }
    }

    return 0;
  }

  void SetSign(double sign) {
    sign_ = sign;
  }

  void Update(double dt_s) override {
    const double position = skeleton_->getPosition(dof_number_);
    const double velocity = skeleton_->getVelocity(dof_number_);

    const double error = base::WrapNegPiToPi(position - desired_rad_);
    const double kMaxForce_kgf_cm = 16.0;
    const double kLimit = kMaxForce_kgf_cm / kConvert_kgf_cm_to_N_m;
    const double control =
        Limit(error * kServo_kp + velocity * kServo_kd, kLimit);

    skeleton_->setForce(dof_number_, -control * power_factor_);
  }

  void SetFree() { power_factor_ = 0.0; }
  void SetTorque() { power_factor_ = 1.0; }

 private:
  SkeletonPtr skeleton_;
  const int dof_number_;
  double desired_rad_ = 0.0;
  double sign_ = 1.0;
  double power_factor_ = 0.0;
};

typedef std::shared_ptr<ServoInterface> ServoInterfacePtr;
typedef std::shared_ptr<ServoController> ServoControllerPtr;

class GimbalController : public ServoInterface {
 public:
  GimbalController(const SkeletonPtr& skel,
                   Joint* yaw,
                   int yaw_dof,
                   Joint* pitch,
                   int pitch_dof)
      : skeleton_(skel),
        yaw_dof_(yaw_dof),
        pitch_dof_(pitch_dof),
        pitch_node_(pitch->getChildBodyNode()) {
    yaw->setVelocityUpperLimit(0, base::Radians(360));
    yaw->setDampingCoefficient(0, 0.2);
    yaw->setCoulombFriction(0, 0.2);

    pitch->setVelocityUpperLimit(0, base::Radians(360));
    pitch->setDampingCoefficient(0, 0.2);
    pitch->setCoulombFriction(0, 0.2);
  }

  virtual ~GimbalController() {}

  void SetPosition(double angle_rad) override {
  }

  void Reboot() override {
  }

  void WriteRam(uint8_t addr, uint8_t data) override {
    switch (addr) {
      case 52: {
        if (data == 0x60) {
          torque_on_ = true;
        } else {
          torque_on_ = false;
        }
        break;
      }
      // pitch rate
      case 0x6c:
      case 0x6d:
      case 0x6e:
      case 0x6f: {
        const int shift = (addr - 0x6c) * 7;
        const int mask = 0x7f << shift;
        pitch_rate_mdps_ = (pitch_rate_mdps_ & (~mask)) | (data << shift);
        break;
      }

        //  yaw rate
      case 0x70:
      case 0x71:
      case 0x72:
      case 0x73: {
        const int shift = (addr - 0x70) * 7;
        const int mask = 0x7f << shift;
        yaw_rate_mdps_ = (yaw_rate_mdps_ & (~mask)) | (data << shift);

        break;
      }

      case 0x7e: { // agitator pwm
        agitator_pwm_ = data;
        break;
      }
    }
  }

  uint8_t ReadRam(uint8_t addr) override {
    switch (addr) {
      case 49: {
        return torque_on_ ? 0x40 : 0x00;
      }
      case 0x60:
      case 0x61:
      case 0x62:
      case 0x63: {
        const int shift = (addr - 0x60) * 7;
        const int yaw_mdps = static_cast<int>(absolute_yaw_deg_ * 1000.0);
        return (yaw_mdps >> shift) & 0x7f;
      }
    }
    return 0;
  }

  static double MakeDps(int mdeg) {
    if (mdeg < (1 << 27)) {
      return static_cast<double>(mdeg) / 1000.0;
    } else {
      return static_cast<double>(mdeg - (1 << 28)) / 1000.0;
    }
  }

  void Update(double dt_s) override {
    const double pitch_rate_dps = MakeDps(pitch_rate_mdps_);
    const double yaw_rate_dps = MakeDps(yaw_rate_mdps_);

    pitch_deg_ += pitch_rate_dps * dt_s;
    yaw_deg_ += yaw_rate_dps * dt_s;

    if (pitch_deg_ < -20.0) { pitch_deg_ = -20.0; }
    if (pitch_deg_ > 20.0) { pitch_deg_ = 20.0; }

    const auto transform = pitch_node_->getWorldTransform();
    const auto rotation =
        transform.rotation() *
        Eigen::AngleAxisd(0.5 * M_PI, Eigen::Vector3d::UnitX());
    const auto euler = rotation.eulerAngles(0, 1, 2);

    // TODO jpieper: pitch and roll are not correct here.  However,
    // for the moment, I really only care about yaw so am not
    // investigating any further.

    //const double actual_roll_deg = base::Degrees(euler[0]) - 90;
    const double actual_yaw_deg = -(base::Degrees(euler[1]) - 180);
    const double actual_pitch_deg =
        base::WrapNeg180To180(base::Degrees(euler[2]) + 180);

    const double change = actual_yaw_deg - last_actual_yaw_deg_;
    if (std::abs(change) > 90.0) {
      if (actual_yaw_deg > last_actual_yaw_deg_) {
        yaw_wrap_deg_ -= 360.0;
      } else {
        yaw_wrap_deg_ += 360.0;
      }
    }
    last_actual_yaw_deg_ = actual_yaw_deg;

    const double unwrapped_actual_yaw_deg = actual_yaw_deg + yaw_wrap_deg_;

    const double yaw_error =
        unwrapped_actual_yaw_deg - yaw_deg_;
    const double pitch_error =
        base::WrapNeg180To180(actual_pitch_deg - pitch_deg_);

    const double power = torque_on_ ? 1.0 : 0.0;

    const double kMaxForce_kgf_cm = 12.0;
    const double kLimit = kMaxForce_kgf_cm / kConvert_kgf_cm_to_N_m;
    const double yaw_control = Limit(yaw_error * kGimbal_kp, kLimit);
    const double pitch_control = Limit(pitch_error * kGimbal_kp, kLimit);

    skeleton_->setForce(yaw_dof_, -yaw_control * power);
    skeleton_->setForce(pitch_dof_, -pitch_control * power);

    absolute_yaw_deg_ = base::WrapNeg180To180(
        base::Degrees(skeleton_->getPosition(yaw_dof_)));
  }

 private:
  SkeletonPtr skeleton_;
  const int yaw_dof_;
  const int pitch_dof_;
  const BodyNode* pitch_node_;

  bool torque_on_ = false;
  uint8_t agitator_pwm_ = 0;

  int pitch_rate_mdps_ = 0;
  int yaw_rate_mdps_ = 0;

  double pitch_deg_ = 0.0;
  double yaw_deg_ = 0.0;

  double last_actual_yaw_deg_ = 0.0;
  double yaw_wrap_deg_ = 0.0;

  double absolute_yaw_deg_ = 0.0;
};

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
  class ImuI2C : public base::AsyncI2C {
   public:
    ImuI2C(Impl* parent) : parent_(parent) {}
    virtual ~ImuI2C() {}

    boost::asio::io_service& get_io_service() override {
      return parent_->context_.service;
    }

    void AsyncRead(uint8_t address, uint8_t reg,
                   base::MutableBufferSequence buffers,
                   base::ReadHandler handler) override {
      for (const auto& device: parent_->i2c_devices_) {
        if (device->address() == address) {
          const auto size = boost::asio::buffer_size(buffers);
          auto it = boost::asio::buffers_begin(buffers);
          for (std::size_t i = 0; i < size; i++) {
            *it = device->Read(reg + i);
            ++it;
          }
          parent_->PostDelayed(std::bind(handler, base::ErrorCode(), size),
                               boost::posix_time::milliseconds(1));
          return;
        }
      }
      get_io_service().post(
          std::bind(handler,
                    base::ErrorCode::einval("unknown I2C address"), 0));
    }

    void AsyncWrite(uint8_t address, uint8_t reg,
                    base::ConstBufferSequence buffers,
                    base::WriteHandler handler) override {
      for (const auto& device: parent_->i2c_devices_) {
        if (device->address() == address) {
          const auto size = boost::asio::buffer_size(buffers);
          auto it = boost::asio::buffers_begin(buffers);
          for (std::size_t i = 0; i < size; i++) {
            device->Write(reg + i, *it);
            ++it;
          }
          parent_->PostDelayed(std::bind(handler, base::ErrorCode(), size),
                               boost::posix_time::milliseconds(1));
          return;
        }
      }
      get_io_service().post(
          std::bind(handler,
                    base::ErrorCode::einval("unknown I2C address"), 0));
    }

    Impl* const parent_;
  };

  Impl()
      : context_(),
        debug_deadline_service_(
            base::DebugDeadlineService::Install(context_.service)) {

    debug_i2c_generator_ = new base::DebugI2CGenerator(context_.service);
    context_.i2c_factory->Register(
        std::unique_ptr<base::I2CFactory::Generator>(debug_i2c_generator_));
    imu_i2c_ = std::make_shared<ImuI2C>(this);
    debug_i2c_generator_->InstallHandler("imu", imu_i2c_);

    world_ = std::make_shared<World>();

    debug_deadline_service_->SetTime(
        boost::posix_time::microsec_clock::universal_time());
    g_impl = this;

    mech_warfare_.reset(new mech::MechWarfare(context_));

    options_description_.add_options()
        ("log,l", po::value(&log_file_),
         "record a log file")
        ("disable-mech", po::bool_switch(&disable_mech_),
         "do not start the mech instance")
        ("start-disabled", po::bool_switch(&start_disabled_),
         "begin in paused mode")
        ("disable-turret", po::bool_switch(&disable_turret_),
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

    for (int i = 0; i < 3; i++) {
      devices_[snum_start + i] = servos_[snum_start + i];
    }

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

    auto gimbal_base =
        std::make_shared<BoxShape>(Eigen::Vector3d(0.040, 0.0695, 0.003));
    gimbal_base->setLocalTransform(
        Eigen::Isometry3d(
            Eigen::Translation3d(
                Eigen::Vector3d(0.0, 0.0, -0.021 - 0.5 * 0.003))));

    turret_yaw->addVisualizationShape(gimbal_base);

    auto add_arm = [&](double side) {
      auto arm = std::make_shared<BoxShape>(
          Eigen::Vector3d(0.040, 0.003, 0.117));
      arm->setLocalTransform(
          Eigen::Isometry3d(
              Eigen::Translation3d(
                  Eigen::Vector3d(
                      0.0,
                      side * 0.5 * 0.0695,
                      -0.0225 - 0.003 - 0.5 * .117))));
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

    auto barrel =
        std::make_shared<BoxShape>(
            Eigen::Vector3d(0.200, 0.03, 0.03));
    barrel->setLocalTransform(
        Eigen::Isometry3d(
            Eigen::Translation3d(
                Eigen::Vector3d(
                    0.03, 0.04, 0.0))));
    turret_pitch->addVisualizationShape(barrel);

    // TODO jpieper: Put the turret mass at the correct location,
    // rather than off to the side and make it be the correct
    // magnitude.

    devices_[0x62] = std::make_shared<GimbalController>(
        skeleton,
        turret_yaw->getParentJoint(),
        skeleton->getNumDofs() - 2,
        turret_pitch->getParentJoint(),
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

    if (!disable_turret_) {
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

    i2c_devices_.push_back(
        std::make_shared<MAX21000>(context_.service, body));
    i2c_devices_.push_back(
        std::make_shared<MMA8451Q>(context_.service,
                                   body, Eigen::Vector3d(0., 0., 0.)));
  }

  void timeStepping() {
    const double dt_s = world_->getTimeStep();

    for (auto& pair: devices_) {
      pair.second->Update(dt_s);
    }

    debug_deadline_service_->SetTime(
        base::Now(context_.service) +
        base::ConvertSecondsToDuration(dt_s));
    context_.service.poll();
    context_.service.reset();
  }

  void Start() {
    if (!log_file_.empty()) {
      context_.telemetry_log->SetRealtime(false);
      context_.telemetry_log->Open(log_file_);
    }

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

    template <typename Functor>
    void DoServo(int servo_addr, Functor functor) {
      if (servo_addr == 0xfe) {
        for (auto& device: parent_->devices_) {
          functor(device.second.get());
        }
      } else {
        auto it = parent_->devices_.find(servo_addr);
        if (it != parent_->devices_.end()) {
          return functor(it->second.get());
        }
      }
    }

    void SJog(const std::vector<ServoAngle>& angles) override {
      for (const auto& angle: angles) {
        const double angle_deg = (angle.second - 512) * 0.325;
        const double angle_rad = base::Radians(angle_deg);

        DoServo(angle.first,
                [&](ServoInterface* s) { s->SetPosition(angle_rad); });
      }
    }

    void Reboot(int servo) override {
      DoServo(servo, [](ServoInterface* s) { s->Reboot(); });
    }

    void WriteRam(int servo_addr, uint8_t addr, uint8_t data) override {
      DoServo(servo_addr, [&](ServoInterface* s) {
          s->WriteRam(addr, data);
        });
    }

    uint8_t ReadRam(int servo, uint8_t addr) override {
      uint8_t result = 0;
      DoServo(servo, [&](ServoInterface* s) {
          result = s->ReadRam(addr);
        });
      return result;
    }

   private:
    Impl* const parent_;
  };

  void PostDelayed(base::NullHandler handler,
                   boost::posix_time::time_duration delay) {
    auto next = base::Now(context_.service) + delay;
    if (callbacks_.empty() || next < callbacks_.begin()->first) {
      // This will be our new first item.
      callback_timer_.expires_at(next);
      callback_timer_.async_wait(
          std::bind(&Impl::HandleCallbackTimer, this));
    } else {
      // We already should have a timer going when we want it.
    }

    callbacks_.insert(std::make_pair(next, handler));
  }

  void HandleCallbackTimer() {
    auto now = base::Now(context_.service);
    while (!callbacks_.empty() && callbacks_.begin()->first <= now) {
      context_.service.post(callbacks_.begin()->second);
      callbacks_.erase(callbacks_.begin());
    }

    if (!callbacks_.empty()) {
      callback_timer_.expires_at(callbacks_.begin()->first);
      callback_timer_.async_wait(
          std::bind(&Impl::HandleCallbackTimer, this));
    }
  }

  std::shared_ptr<dart::simulation::World> world_;
  dart::dynamics::SkeletonPtr floor_;
  dart::dynamics::SkeletonPtr mech_;
  int current_joint_ = 0;
  std::map<int, ServoControllerPtr> servos_;
  std::map<int, ServoInterfacePtr> devices_;

  base::Context context_;
  base::DebugDeadlineService* const debug_deadline_service_;
  base::ConcreteStreamFactory::Parameters stream_config_;
  base::SharedStream stream_;

  base::DebugI2CGenerator* debug_i2c_generator_ = nullptr;
  base::SharedI2C imu_i2c_;

  char buffer_[256] = {};
  HerkulexOperations operations_{this};
  std::unique_ptr<HerkulexProtocol> herkulex_protocol_;

  boost::program_options::options_description options_description_;

  std::unique_ptr<mech::MechWarfare> mech_warfare_;
  std::string log_file_;
  bool disable_mech_ = false;
  bool start_disabled_ = false;
  bool disable_turret_ = false;

  std::list<std::shared_ptr<I2CDevice> > i2c_devices_;
  std::multimap<boost::posix_time::ptime, base::NullHandler> callbacks_;
  base::DeadlineTimer callback_timer_{context_.service};
};

SimulatorWindow::SimulatorWindow() : impl_(new Impl()) {
  impl_->floor_ = createFloor();
  impl_->world_->addSkeleton(impl_->floor_);

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
