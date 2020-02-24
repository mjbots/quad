// Copyright 2019 Josh Pieper, jjp@pobox.com.  All rights reserved.
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

#include "mech/quadruped_control.h"

#include <fstream>

#include <fmt/format.h>

#include "mjlib/base/json5_read_archive.h"
#include "mjlib/base/program_options_archive.h"

#include "mjlib/io/now.h"
#include "mjlib/io/repeating_timer.h"

#include "base/common.h"
#include "base/logging.h"
#include "base/sophus.h"

#include "mech/mammal_ik.h"
#include "mech/moteus.h"

namespace pl = std::placeholders;

namespace mjmech {
namespace mech {

namespace {
using QC = QuadrupedCommand;
using QM = QC::Mode;

template <typename Iter, typename Functor>
auto Average(Iter begin, Iter end, Functor f) -> decltype(f(*begin)) {
  using T = decltype(f(*begin));

  T sum = {};
  T count = {};
  for (Iter it = begin; it != end; ++it) {
    count++;
    sum += f(*it);
  }
  return sum / count;
}

constexpr double kGravity = 9.81;

// This represents the JSON used to configure the geometry of the
// robot.
struct Config {
  struct Joint {
    int id = 0;
    double sign = 1.0;
    double min_deg = -360.0;
    double max_deg = 360.0;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(id));
      a->Visit(MJ_NVP(sign));
      a->Visit(MJ_NVP(min_deg));
      a->Visit(MJ_NVP(max_deg));
    }
  };

  std::vector<Joint> joints;

  struct Leg {
    int leg = 0;
    Sophus::SE3d pose_mm_BG;
    MammalIk::Config ik;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(leg));
      a->Visit(MJ_NVP(pose_mm_BG));
      a->Visit(MJ_NVP(ik));
    }
  };

  std::vector<Leg> legs;

  struct Bounds {
    double min_z_B = 0.0;
    double max_z_B = 300.0;
    double max_acceleration_mm_s2 = 50000;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(min_z_B));
      a->Visit(MJ_NVP(max_z_B));
      a->Visit(MJ_NVP(max_acceleration_mm_s2));
    }
  };

  Bounds bounds;

  double mass_kg = 10.0;

  struct MammalJoint {
    double shoulder_deg = 0.0;
    double femur_deg = 125.0;
    double tibia_deg = -135.0;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(shoulder_deg));
      a->Visit(MJ_NVP(femur_deg));
      a->Visit(MJ_NVP(tibia_deg));
    }
  };

  struct StandUp {
    MammalJoint pose;
    double velocity_dps = 30.0;
    double velocity_mm_s = 100.0;
    double max_preposition_torque_Nm = 3.0;
    double timeout_s = 10.0;
    double tolerance_deg = 1.0;
    double tolerance_mm = 1;
    double force_scale_window_mm = 100;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(pose));
      a->Visit(MJ_NVP(velocity_dps));
      a->Visit(MJ_NVP(velocity_mm_s));
      a->Visit(MJ_NVP(max_preposition_torque_Nm));
      a->Visit(MJ_NVP(timeout_s));
      a->Visit(MJ_NVP(tolerance_deg));
      a->Visit(MJ_NVP(tolerance_mm));
      a->Visit(MJ_NVP(force_scale_window_mm));
    }
  };

  StandUp stand_up;

  struct Rest {
    double velocity_mm_s = 100.0;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(velocity_mm_s));
    }
  };

  Rest rest;

  double stand_height_mm = 200.0;
  double rb_filter_constant_Hz = 2.0;
  double lr_acceleration_mm_s2 = 1000.0;
  double lr_alpha_rad_s2 = 0.5;

  struct Jump {
    double lower_velocity_mm_s = 100.0;
    double retract_velocity_mm_s = 1000.0;
    double land_threshold_mm_s = 100.0;
    double land_threshold_s = 0.02;
    double land_kp = 0.05;
    double land_kd = 0.1;
    double lower_height_mm = 100.0;
    double upper_height_mm = 220.0;
    double retract_height_mm = 190.0;
    double landing_force_scale = 1.0;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(lower_velocity_mm_s));
      a->Visit(MJ_NVP(retract_velocity_mm_s));
      a->Visit(MJ_NVP(land_threshold_mm_s));
      a->Visit(MJ_NVP(land_threshold_s));
      a->Visit(MJ_NVP(land_kp));
      a->Visit(MJ_NVP(land_kd));
      a->Visit(MJ_NVP(lower_height_mm));
      a->Visit(MJ_NVP(upper_height_mm));
      a->Visit(MJ_NVP(retract_height_mm));
      a->Visit(MJ_NVP(landing_force_scale));
    }
  };

  Jump jump;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(joints));
    a->Visit(MJ_NVP(legs));
    a->Visit(MJ_NVP(bounds));
    a->Visit(MJ_NVP(mass_kg));
    a->Visit(MJ_NVP(stand_up));
    a->Visit(MJ_NVP(rest));
    a->Visit(MJ_NVP(stand_height_mm));
    a->Visit(MJ_NVP(rb_filter_constant_Hz));
    a->Visit(MJ_NVP(lr_acceleration_mm_s2));
    a->Visit(MJ_NVP(lr_alpha_rad_s2));
    a->Visit(MJ_NVP(jump));
  }
};

struct Leg {
  int leg = 0;
  Config::Leg config;
  Sophus::SE3d pose_mm_BG;
  MammalIk ik;

  base::Point3D stand_up_R;
  base::Point3D idle_R;

  Leg(const Config::Leg& config_in,
      const Config::StandUp& stand_up,
      double stand_height_mm)
      : leg(config_in.leg),
        config(config_in),
        pose_mm_BG(config_in.pose_mm_BG),
        ik(config_in.ik) {
    IkSolver::JointAngles joints;

    auto make_joint = [&](int id, double angle_deg) {
      IkSolver::Joint joint;
      joint.id = id;
      joint.angle_deg = angle_deg;
      return joint;
    };

    joints.push_back(
        make_joint(config.ik.shoulder.id,
                   stand_up.pose.shoulder_deg));
    joints.push_back(
        make_joint(config.ik.femur.id,
                   stand_up.pose.femur_deg));
    joints.push_back(
        make_joint(config.ik.tibia.id,
                   stand_up.pose.tibia_deg));

    const auto pose_mm_G = ik.Forward_G(joints);
    // The idle and standup poses assume a null RB transform
    const Sophus::SE3d pose_mm_RB;
    const auto pose_mm_R = pose_mm_RB * (config.pose_mm_BG * pose_mm_G);

    stand_up_R = pose_mm_R.pose_mm;
    idle_R = base::Point3D(pose_mm_R.pose_mm.x(),
                           pose_mm_R.pose_mm.y(),
                           stand_height_mm);
  }
};

struct CommandLog {
  boost::posix_time::ptime timestamp;

  const QC* command = &ignored_command;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(timestamp));
    const_cast<QC*>(command)->Serialize(a);
  }

  static QC ignored_command;
};

QC CommandLog::ignored_command;

struct ControlLog {
  boost::posix_time::ptime timestamp;
  std::vector<QC::Joint> joints;
  std::vector<QC::Leg> legs_B;
  std::vector<QC::Leg> legs_R;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(timestamp));
    a->Visit(MJ_NVP(joints));
    a->Visit(MJ_NVP(legs_B));
    a->Visit(MJ_NVP(legs_R));
  }
};

}

class QuadrupedControl::Impl {
 public:
  Impl(base::Context& context)
      : executor_(context.executor),
        timer_(executor_) {
    context.telemetry_registry->Register("qc_status", &status_signal_);
    context.telemetry_registry->Register("qc_command", &command_signal_);
    context.telemetry_registry->Register("qc_control", &control_signal_);

    mjlib::base::ProgramOptionsArchive(&options_).Accept(&parameters_);
  }

  void AsyncStart(mjlib::io::ErrorCallback callback) {
    // Load our configuration.
    std::ifstream inf(parameters_.config);
    mjlib::base::system_error::throw_if(
        !inf.is_open(),
        fmt::format("could not open config file '{}'", parameters_.config));

    mjlib::base::Json5ReadArchive(inf).Accept(&config_);

    if (config_.legs.size() != 4 ||
        config_.joints.size() != 12) {
      mjlib::base::Fail(
          fmt::format(
              "Incorrect number of legs/joints configured: {}/{} != 4/12",
              config_.legs.size(), config_.joints.size()));
    }

    Configure();

    PopulateStatusRequest();

    timer_.start(mjlib::base::ConvertSecondsToDuration(parameters_.period_s),
                 std::bind(&Impl::HandleTimer, this, pl::_1));

    boost::asio::post(
        executor_,
        std::bind(std::move(callback), mjlib::base::error_code()));
  }

  void Command(const QC& command) {
    CommandLog command_log;
    command_log.timestamp = Now();
    command_log.command = &command;

    current_command_ = command;

    command_signal_(&command_log);
  }

  void Configure() {
    for (const auto& leg : config_.legs) {
      legs_.emplace_back(leg, config_.stand_up, config_.stand_height_mm);
    }
  }

  void PopulateStatusRequest() {
    status_request_ = {};
    for (const auto& joint : config_.joints) {
      status_request_.push_back({});
      auto& current = status_request_.back();
      current.id = joint.id;

      // Read mode, position, velocity, and torque.
      current.request.ReadMultiple(moteus::Register::kMode, 4, 1);
      current.request.ReadMultiple(moteus::Register::kVoltage, 3, 0);
    }
  }

  void HandleTimer(const mjlib::base::error_code& ec) {
    if (ec == boost::asio::error::operation_aborted) { return; }
    mjlib::base::FailIf(ec);

    if (!client_) { return; }
    if (outstanding_) { return; }

    timestamps_.last_cycle_start = timestamps_.cycle_start;
    timestamps_.cycle_start = Now();

    timestamps_.delta_s = base::ConvertDurationToSeconds(
        timestamps_.cycle_start - timestamps_.last_cycle_start);

    outstanding_ = true;

    status_reply_ = {};
    client_->AsyncRegisterMultiple(status_request_, &status_reply_,
                                   std::bind(&Impl::HandleStatus, this, pl::_1));
  }

  void HandleStatus(const mjlib::base::error_code& ec) {
    mjlib::base::FailIf(ec);

    timestamps_.status_done = Now();

    // If we don't have all 12 servos, then skip this cycle.
    status_.missing_replies = 12 - status_reply_.replies.size();

    if (status_reply_.replies.size() != 12) {
      if (status_.state.joints.size() != 12) {
        // We have to get at least one full set before we can start
        // updating.
        outstanding_ = false;
        return;
      }
    }

    // Fill in the status structure.
    UpdateStatus();

    // Now run our control loop and generate our command.
    std::swap(control_log_, old_control_log_);
    *control_log_ = {};
    RunControl();

    timestamps_.control_done = Now();

    if (!client_command_.empty()) {
      client_command_reply_ = {};
      client_->AsyncRegisterMultiple(client_command_, &client_command_reply_,
                                     std::bind(&Impl::HandleCommand, this, pl::_1));
    } else {
      HandleCommand({});
    }
  }

  void HandleCommand(const mjlib::base::error_code& ec) {
    mjlib::base::FailIf(ec);
    outstanding_ = false;

    const auto now = Now();
    timestamps_.command_done = now;

    status_.timestamp = now;
    status_.time_status_s =
        mjlib::base::ConvertDurationToSeconds(
            timestamps_.status_done - timestamps_.cycle_start);
    status_.time_control_s =
        mjlib::base::ConvertDurationToSeconds(
            timestamps_.control_done - timestamps_.status_done);
    status_.time_command_s =
        mjlib::base::ConvertDurationToSeconds(
            timestamps_.command_done - timestamps_.control_done);
    status_.time_cycle_s =
        mjlib::base::ConvertDurationToSeconds(
            timestamps_.command_done - timestamps_.cycle_start);
    status_.time_delta_s = timestamps_.delta_s;

    status_signal_(&status_);
  }

  double GetSign(int id) const {
    for (const auto& joint : config_.joints) {
      if (joint.id == id) { return joint.sign; }
    }
    mjlib::base::AssertNotReached();
  }

  void UpdateStatus() {
    std::vector<QuadrupedState::Link> links;

    auto find_or_make_joint = [&](int id) -> QuadrupedState::Joint& {
      for (auto& joint : status_.state.joints) {
        if (joint.id == id) { return joint; }
      }
      status_.state.joints.push_back({});
      auto& result = status_.state.joints.back();
      result.id = id;
      return result;
    };

    for (const auto& reply : status_reply_.replies) {
      QuadrupedState::Joint& out_joint = find_or_make_joint(reply.id);
      QuadrupedState::Link out_link;

      out_joint.id = out_link.id = reply.id;

      const double sign = GetSign(reply.id);

      for (const auto& pair : reply.reply) {
        const auto* maybe_value = std::get_if<moteus::Value>(&pair.second);
        if (!maybe_value) { continue; }
        const auto& value = *maybe_value;
        switch (static_cast<moteus::Register>(pair.first)) {
          case moteus::kMode: {
            out_joint.mode = moteus::ReadInt(value);
            break;
          }
          case moteus::kPosition: {
            out_joint.angle_deg = sign * moteus::ReadPosition(value);
            out_link.angle_deg = out_joint.angle_deg;
            break;
          }
          case moteus::kVelocity: {
            out_joint.velocity_dps = sign * moteus::ReadPosition(value);
            out_link.velocity_dps = out_joint.velocity_dps;
            break;
          }
          case moteus::kTorque: {
            out_joint.torque_Nm = sign * moteus::ReadTorque(value);
            out_link.torque_Nm = out_joint.torque_Nm;
            break;
          }
          case moteus::kVoltage: {
            out_joint.voltage = moteus::ReadVoltage(value);
            break;
          }
          case moteus::kTemperature: {
            out_joint.temperature_C = moteus::ReadTemperature(value);
            break;
          }
          case moteus::kFault: {
            out_joint.fault = moteus::ReadInt(value);
            break;
          }
          default: {
            break;
          }
        }
      }

      links.push_back(out_link);
    }

    std::sort(status_.state.joints.begin(), status_.state.joints.end(),
              [](const auto& lhs, const auto& rhs) {
                return lhs.id < rhs.id;
              });

    if (status_.mode != QM::kFault) {
      std::string fault;

      for (const auto& joint : status_.state.joints) {
        if (joint.fault) {
          if (!fault.empty()) {
            fault += ", ";
          }
          fault += fmt::format("servo {} fault: {}", joint.id, joint.fault);
        }
      }
      if (!fault.empty()) {
        Fault(fault);
      }
    }

    IkSolver::JointAngles joint_angles;
    for (const auto& joint : status_.state.joints) {
      IkSolver::Joint ik_joint;

      ik_joint.id = joint.id;
      ik_joint.angle_deg = joint.angle_deg;
      ik_joint.velocity_dps = joint.velocity_dps;
      ik_joint.torque_Nm = joint.torque_Nm;

      joint_angles.push_back(ik_joint);
    }

    status_.state.legs_B.clear();

    auto get_link = [&](int id) {
      for (const auto& link : links) {
        if (link.id == id) { return link; }
      }
      return QuadrupedState::Link();
    };

    auto find_or_make_leg = [&](int id) -> QuadrupedState::Leg& {
      for (auto& leg : status_.state.legs_B) {
        if (leg.leg == id) { return leg; }
      }

      status_.state.legs_B.push_back({});
      auto& result = status_.state.legs_B.back();
      result.leg = id;
      return result;
    };

    for (const auto& leg : legs_) {
      QuadrupedState::Leg& out_leg_B = find_or_make_leg(leg.leg);
      const auto effector_G = leg.ik.Forward_G(joint_angles);
      const auto effector_B = leg.pose_mm_BG * effector_G;

      out_leg_B.leg = leg.leg;
      out_leg_B.position_mm = effector_B.pose_mm;
      out_leg_B.velocity_mm_s = effector_B.velocity_mm_s;
      out_leg_B.force_N = effector_B.force_N;

      out_leg_B.links.clear();
      out_leg_B.links.push_back(get_link(leg.config.ik.shoulder.id));
      out_leg_B.links.push_back(get_link(leg.config.ik.femur.id));
      out_leg_B.links.push_back(get_link(leg.config.ik.tibia.id));
    }

    std::sort(status_.state.legs_B.begin(), status_.state.legs_B.end(),
              [](const auto& lhs, const auto& rhs) {
                return lhs.leg < rhs.leg;
              });

    // Now update the robot values.

    // pose_mm_RB isn't sensed, but is just a commanded value.
  }

  void RunControl() {
    if (current_command_.mode != status_.mode) {
      MaybeChangeMode();
    }

    switch (status_.mode) {
      case QM::kStopped: {
        DoControl_Stopped();
        break;
      }
      case QM::kFault: {
        DoControl_Fault();
        break;
      }
      case QM::kZeroVelocity: {
        DoControl_ZeroVelocity();
        break;
      }
      case QM::kJoint: {
        DoControl_Joint();
        break;
      }
      case QM::kLeg: {
        DoControl_Leg();
        break;
      }
      case QM::kStandUp: {
        DoControl_StandUp();
        break;
      }
      case QM::kRest: {
        DoControl_Rest();
        break;
      }
      case QM::kJump: {
        DoControl_Jump();
        break;
      }
      case QM::kNumModes: {
        mjlib::base::AssertNotReached();
      }
    }
  }

  void MaybeChangeMode() {
    const auto old_mode = status_.mode;
    switch (current_command_.mode) {
      case QM::kNumModes:
      case QM::kFault: {
        mjlib::base::AssertNotReached();
      }
      case QM::kStopped: {
        // It is always valid (although I suppose not always a good
        // idea) to enter the stopped mode.
        status_.mode = QM::kStopped;
        break;
      }
      case QM::kZeroVelocity:
      case QM::kJoint:
      case QM::kLeg: {
        // We can always do these if not faulted.
        if (status_.mode == QM::kFault) { return; }
        status_.mode = current_command_.mode;
        break;
      }
      case QM::kStandUp: {
        // We can only do this from stopped.
        if (!(status_.mode == QM::kStopped ||
              status_.mode == QM::kZeroVelocity)) {
          return;
        }
        status_.mode = current_command_.mode;

        // Since we're just switching to this mode, start from
        // scratch.
        status_.state.stand_up = {};
        break;
      }
      case QM::kRest: {
        // This can only be done from standing up.  If we're stopped,
        // then we should just go ahead and stand up first.
        if (status_.mode == QM::kStopped ||
            status_.mode == QM::kZeroVelocity) {
          status_.mode = QM::kStandUp;
          status_.state.stand_up = {};
        } else if (status_.mode == QM::kStandUp &&
                   status_.state.stand_up.mode ==
                   QuadrupedState::StandUp::Mode::kDone) {
          status_.mode = current_command_.mode;
        } else if (status_.mode == QM::kJump &&
                   status_.state.jump.mode ==
                   QuadrupedState::Jump::Mode::kDone) {
          status_.mode = current_command_.mode;
        }

        // TODO(jpieper): When we have a moving mode, we should be
        // able to enter the Rest state as long as we are moving
        // slowly enough and all four legs are on the ground, although
        // perhaps we will require a zero-velocity step cycle to get
        // the legs into the regular idle position.
        break;
      }
      case QM::kJump: {
        if (status_.mode == QM::kStopped ||
            status_.mode == QM::kZeroVelocity) {
          status_.mode = QM::kStandUp;
          status_.state.stand_up = {};
        } else if (status_.mode == QM::kRest ||
                   (status_.mode == QM::kStandUp &&
                    status_.state.stand_up.mode ==
                    QuadrupedState::StandUp::Mode::kDone)) {
          status_.mode = current_command_.mode;
          status_.state.jump.command = current_command_.jump.value();
        }

        break;
      }
    }

    if (status_.mode != old_mode) {
      switch (old_mode) {
        case QM::kStandUp: {
          status_.state.stand_up = {};
          break;
        }
        case QM::kJump: {
          status_.state.jump = {};
          break;
        }
        case QM::kStopped:
        case QM::kFault:
        case QM::kZeroVelocity:
        case QM::kJoint:
        case QM::kLeg:
        case QM::kNumModes:
        case QM::kRest: {
          break;
        }
      }
      status_.mode_start = Now();
    }
  }

  void DoControl_Stopped() {
    status_.fault = "";
    std::vector<QC::Joint> out_joints;
    for (const auto& joint : config_.joints) {
      QC::Joint out_joint;
      out_joint.id = joint.id;
      out_joint.power = false;
      out_joints.push_back(out_joint);
    }

    ControlJoints(std::move(out_joints));
  }

  void Fault(std::string_view message) {
    status_.mode = QM::kFault;
    status_.fault = message;
    status_.mode_start = Now();

    log_.warn("Fault: " + std::string(message));

    DoControl_Fault();
  }

  void DoControl_Fault() {
    DoControl_ZeroVelocity();
  }

  void DoControl_ZeroVelocity() {
    std::vector<QC::Joint> out_joints;
    for (const auto& joint : config_.joints) {
      QC::Joint out_joint;
      out_joint.id = joint.id;
      out_joint.power = true;
      out_joint.zero_velocity = true;
      out_joints.push_back(out_joint);
    }

    ControlJoints(std::move(out_joints));
  }

  void DoControl_Joint() {
    ControlJoints(current_command_.joints);
  }

  void DoControl_Leg() {
    ControlLegs_B(current_command_.legs_B);
  }

  void DoControl_StandUp() {
    // While we are standing up, the RB transform is always nil.
    status_.state.robot.pose_mm_RB = Sophus::SE3d();
    ClearDesiredMotion();

    using M = QuadrupedState::StandUp::Mode;
    // See if we can advance to the next state.

    const double elapsed_s =
        base::ConvertDurationToSeconds(Now() - status_.mode_start);
    if (status_.state.stand_up.mode != M::kDone &&
        elapsed_s > config_.stand_up.timeout_s) {
      Fault("timeout");
      return;
    }

    switch (status_.state.stand_up.mode) {
      case M::kPrepositioning: {
        const bool done = CheckPrepositioning();
        if (done) {
          DoControl_StandUp_StartStanding();
          status_.state.stand_up.mode = M::kStanding;
        }
        break;
      }
      case M::kStanding: {
        const bool done = CheckStanding();
        if (done) {
          status_.state.stand_up.mode = M::kDone;
        }
        break;
      }
      case M::kDone: {
        // We never leave this state automatically.
        break;
      }
    }


    // Now execute our control.
    switch (status_.state.stand_up.mode) {
      case M::kPrepositioning: {
        DoControl_StandUp_Prepositioning();
        break;
      }
      case M::kStanding:
      case M::kDone: {
        DoControl_StandUp_Standing();
        break;
      }
    };
  }

  bool CheckStanding() const {
    for (const auto& leg : status_.state.stand_up.legs) {
      if ((leg.pose_mm_R - leg.target_mm_R).norm() >
          config_.stand_up.tolerance_mm) {
        return false;
      }
    }

    return true;
  }

  bool CheckPrepositioning() const {
    // We're done when all our joints are close enough.
    const std::map<int, double> current_deg = [&]() {
      std::map<int, double> result;
      for (const auto& joint : status_.state.joints) {
        result[joint.id] = joint.angle_deg;
      }
      return result;
    }();

    for (const auto& leg : legs_) {
      auto check = [&](int id, int expected_deg) {
        const auto it = current_deg.find(id);
        BOOST_ASSERT(it != current_deg.end());
        if (std::abs(it->second - expected_deg) > config_.stand_up.tolerance_deg) {
          return false;
        }
        return true;
      };

      if (!check(leg.config.ik.shoulder.id, config_.stand_up.pose.shoulder_deg)) {
        return false;
      }
      if (!check(leg.config.ik.femur.id, config_.stand_up.pose.femur_deg)) {
        return false;
      }
      if (!check(leg.config.ik.tibia.id, config_.stand_up.pose.tibia_deg)) {
        return false;
      }
    }
    return true;
  }

  void DoControl_StandUp_Prepositioning() {
    std::vector<QC::Joint> joints;
    for (const auto& leg : legs_) {
      QC::Joint joint;
      joint.power = true;
      joint.angle_deg = std::numeric_limits<double>::quiet_NaN();
      joint.velocity_dps = config_.stand_up.velocity_dps;
      joint.max_torque_Nm = config_.stand_up.max_preposition_torque_Nm;

      auto add_joint = [&](int id, double angle_deg) {
        joint.id = id;
        joint.stop_angle_deg = angle_deg;
        joints.push_back(joint);
      };

      add_joint(leg.config.ik.shoulder.id, config_.stand_up.pose.shoulder_deg);
      add_joint(leg.config.ik.femur.id, config_.stand_up.pose.femur_deg);
      add_joint(leg.config.ik.tibia.id, config_.stand_up.pose.tibia_deg);
    }
    ControlJoints(joints);
  }

  void DoControl_StandUp_StartStanding() {
    status_.state.stand_up.legs = {};

    // This is called right before we begin the "standing" phase.  It
    // figures out what target leg position we want for each leg.
    for (const auto& leg : legs_) {
      QuadrupedState::StandUp::Leg sleg;
      sleg.leg = leg.leg;
      sleg.pose_mm_R = leg.stand_up_R;
      sleg.target_mm_R = leg.idle_R;
      status_.state.stand_up.legs.push_back(sleg);
    }
  }

  void DoControl_StandUp_Standing() {
    std::vector<QC::Leg> legs_R;

    for (auto& leg : status_.state.stand_up.legs) {
      const base::Point3D delta = leg.target_mm_R - leg.pose_mm_R;
      const double distance =
          std::min(delta.norm(),
                   timestamps_.delta_s * config_.stand_up.velocity_mm_s);
      if (distance != 0.0) {
        leg.pose_mm_R += distance * (delta / delta.norm());
      }

      QC::Leg leg_cmd_R;
      leg_cmd_R.leg_id = leg.leg;
      leg_cmd_R.power = true;
      leg_cmd_R.position_mm = leg.pose_mm_R;
      leg_cmd_R.velocity_mm_s = base::Point3D(0, 0, config_.stand_up.velocity_mm_s);

      const double force_scale =
          std::max(0.0, config_.stand_up.force_scale_window_mm - delta.norm()) /
          config_.stand_up.force_scale_window_mm;

      leg_cmd_R.force_N = base::Point3D(
          0, 0, force_scale * config_.mass_kg * kGravity / legs_.size());

      legs_R.push_back(leg_cmd_R);
    }

    ControlLegs_R(std::move(legs_R));
  }

  void DoControl_Rest() {
    UpdateCommandedRB();
    ClearDesiredMotion();

    std::vector<QC::Leg> legs_R;

    if (!old_control_log_->legs_R.empty()) {
      legs_R = old_control_log_->legs_R;
      MoveLegsZ(
          &legs_R,
          config_.rest.velocity_mm_s,
          config_.stand_height_mm,
          0.0);
    } else {
      for (const auto& leg : legs_) {
        QC::Leg leg_R;
        leg_R.leg_id = leg.leg;
        leg_R.power = true;
        leg_R.position_mm = leg.idle_R;
        leg_R.force_N = base::Point3D(
            0, 0, kGravity * config_.mass_kg / legs_.size());
        legs_R.push_back(leg_R);
      }
    }

    ControlLegs_R(std::move(legs_R));
  }

  void DoControl_Jump() {
    using JM = QuadrupedState::Jump::Mode;

    UpdateCommandedRB();
    // We can only accelerate in one of the states where our legs are
    // actually in contact with the ground.
    const bool in_contact = [&]() {
      switch (status_.state.jump.mode) {
        case JM::kPushing:
        case JM::kLanding: {
          return true;
        }
        // We technically are in contact w/ the ground during
        // lowering, but since we can only enter here with zero
        // velocity, we'll just wait until the first jump before
        // starting to move.
        case JM::kLowering:
        case JM::kRetracting:
        case JM::kFalling:
        case JM::kDone: {
          return false;
        }
      }
    }();
    if (in_contact) {
      UpdateCommandedLR();
    }

    auto& js = status_.state.jump;

    auto legs_R = old_control_log_->legs_R;

    switch (status_.state.jump.mode) {
      case JM::kLowering: {
        // Move legs to take into account LR rates.
        MoveLegsForLR(&legs_R);

        // Lower all legs until they reach the lower_height.
        const bool done = MoveLegsZ(
            &legs_R,
            config_.jump.lower_velocity_mm_s,
            config_.jump.lower_height_mm,
            0);
        if (done) {
          status_.state.jump.mode = JM::kPushing;
          status_.state.jump.velocity_mm_s = 0.0;
          status_.state.jump.acceleration_mm_s2 =
              js.command.acceleration_mm_s2;
        }
        break;
      }
      case JM::kPushing: {
        MoveLegsForLR(&legs_R);

        status_.state.jump.velocity_mm_s +=
            js.command.acceleration_mm_s2 * timestamps_.delta_s;
        const bool done = MoveLegsZ(
            &legs_R,
            js.velocity_mm_s,
            config_.jump.upper_height_mm,
            config_.mass_kg * js.acceleration_mm_s2 * 0.001);
        if (done) {
          js.mode = JM::kRetracting;
          js.velocity_mm_s = 0.0;
          js.acceleration_mm_s2 = 0.0;
          for (auto& leg_R : legs_R) {
            leg_R.stance = false;
          }
        }
        break;
      }
      case JM::kRetracting: {
        // We need to get our legs back into a lateral position
        // suitable for the landing phase.

        const bool done = MoveLegs(
            &legs_R,
            config_.jump.retract_velocity_mm_s,
            [&]() {
              std::vector<std::pair<int, base::Point3D>> result;
              for (const auto& leg : legs_) {
                result.push_back(std::make_pair(leg.leg, leg.idle_R));
              }
              return result;
            }(),
            0.0);

        const double average_velocity_mm_s = Average(
            status_.state.legs_B.begin(),
            status_.state.legs_B.end(),
            [](const auto& leg_B) {
              return leg_B.velocity_mm_s.z();
            });
        if (done &&
            std::abs(average_velocity_mm_s) <
            config_.jump.land_threshold_mm_s) {
          js.mode = JM::kFalling;
          js.falling = Now();
        }
        break;
      }
      case JM::kFalling: {
        // Keep the legs moving while falling so that relative
        // velocity will be minimal when we do land.  We aren't trying
        // to accelerate during this phase, so that should mostly work
        // out.
        MoveLegsForLR(&legs_R);

        // Set our gains to be much lower while falling.
        for (auto& leg_R : legs_R) {
          leg_R.kp_scale = base::Point3D(config_.jump.land_kp,
                                         config_.jump.land_kp,
                                         config_.jump.land_kp);
          leg_R.kd_scale = base::Point3D(config_.jump.land_kd,
                                         config_.jump.land_kd,
                                         config_.jump.land_kd);
        }

        // Wait for our average velocity of all legs to exceed our
        // threshold, which indicates we have landed.
        const double average_velocity_mm_s = Average(
            status_.state.legs_B.begin(),
            status_.state.legs_B.end(),
            [](const auto& leg_B) {
              return leg_B.velocity_mm_s.z();
            });
        // Also, if we're already applying more than half of our total
        // weight, that must mean we have landed also.
        const double total_force_z_N = Average(
            status_.state.legs_B.begin(),
            status_.state.legs_B.end(),
            [](const auto& leg_B) {
              return leg_B.force_N.z();
            }) * status_.state.legs_B.size();
        if (js.falling.is_not_a_date_time() ||
            (average_velocity_mm_s >= -config_.jump.land_threshold_mm_s &&
             total_force_z_N < (0.75 * kGravity * config_.mass_kg))) {
          js.falling = Now();
        }
        const double moving_duration_s =
            base::ConvertDurationToSeconds(Now() -
                                           js.falling);

        if (moving_duration_s > config_.jump.land_threshold_s) {
          js.velocity_mm_s = average_velocity_mm_s;
          // Pick an acceleration that will ensure we reach stopped
          // before hitting our lower height.
          const double average_height_mm = Average(
              status_.state.legs_B.begin(),
              status_.state.legs_B.end(),
              [](const auto& leg_B) {
                return leg_B.position_mm.z();
              });
          const double error_mm =
              average_height_mm - config_.jump.lower_height_mm;
          if (error_mm < 0.0) {
            // Whoops, we're already below our allowed limit.  Pick a
            // safe maximum acceleration and hope for the best.
            js.acceleration_mm_s2 =
                config_.bounds.max_acceleration_mm_s2;
          } else {
            js.acceleration_mm_s2 =
                std::max(
                    js.command.acceleration_mm_s2,
                    std::min(
                        config_.bounds.max_acceleration_mm_s2,
                        0.5 * std::pow(average_velocity_mm_s, 2) / error_mm));
          }

          js.mode = JM::kLanding;
          for (auto& leg_R : legs_R) {
            const auto& pose_mm_RB = status_.state.robot.pose_mm_RB;
            auto desired_B = pose_mm_RB.inverse() * leg_R.position_mm;
            desired_B.z() = average_height_mm;

            leg_R.position_mm = pose_mm_RB * desired_B;
            leg_R.stance = true;
          }
        }
        break;
      }
      case JM::kLanding: {
        MoveLegsForLR(&legs_R);

        // Turn our gains back up.
        for (auto& leg_R : legs_R) {
          leg_R.kp_scale = {};
          leg_R.kd_scale = {};
        }

        // Work to zero our velocity.
        js.velocity_mm_s =
            std::min(
                0.0,
                js.velocity_mm_s +
                js.acceleration_mm_s2 * timestamps_.delta_s);
        if (js.velocity_mm_s == 0.0) {
          // We are either done, or going to start jumping again.

          // TODO(jpieper): Condition switching to rest on us having a
          // sufficiently low R frame velocity.

          if (js.command.repeat &&
              current_command_.mode == QM::kJump) {
            js.mode = JM::kPushing;
          } else {
            js.mode = JM::kDone;

            // Switch our top level command back to rest to make life
            // more convenient.
            current_command_.mode = QM::kRest;
          }
        } else {
          MoveLegsZ(
              &legs_R,
              js.velocity_mm_s,
              config_.jump.upper_height_mm,
              (config_.jump.landing_force_scale * config_.mass_kg *
               js.acceleration_mm_s2 * 0.001));
        }
        break;
      }
      case JM::kDone: {
        DoControl_Rest();
        return;
      }
    }

    ControlLegs_R(std::move(legs_R));
  }

  void ClearDesiredMotion() {
    status_.state.robot.desired_v_mm_s_R = {};
    status_.state.robot.desired_w_LR = {};
  }

  void UpdateCommandedRB() {
    // Smoothly filter in any commanded RB transform.
    const base::Point3D translation =
        current_command_.pose_mm_RB.translation() -
        status_.state.robot.pose_mm_RB.translation();
    status_.state.robot.pose_mm_RB.translation() +=
        config_.rb_filter_constant_Hz * timestamps_.delta_s * translation;
    status_.state.robot.pose_mm_RB.so3() =
        Sophus::SO3d(
            status_.state.robot.pose_mm_RB.so3().unit_quaternion().slerp(
                config_.rb_filter_constant_Hz * timestamps_.delta_s,
                current_command_.pose_mm_RB.so3().unit_quaternion()));
  }

  void UpdateCommandedLR() {
    const base::Point3D input_delta_mm_s = (
        current_command_.v_mm_s_R - status_.state.robot.desired_v_mm_s_R);
    const double input_delta_norm_mm_s = input_delta_mm_s.norm();
    const double max_delta_mm_s =
        config_.lr_acceleration_mm_s2 * timestamps_.delta_s;
    const base::Point3D delta_mm_s =
        (input_delta_norm_mm_s < max_delta_mm_s) ?
        input_delta_mm_s :
        input_delta_mm_s.normalized() * max_delta_mm_s;

    status_.state.robot.desired_v_mm_s_R += delta_mm_s;
    // We require this.
    status_.state.robot.desired_v_mm_s_R.z() = 0;

    const base::Point3D input_delta_rad_s = (
        current_command_.w_LR - status_.state.robot.desired_w_LR);
    const double input_delta_norm_rad_s = input_delta_rad_s.norm();
    const double max_delta_rad_s =
        config_.lr_alpha_rad_s2 * timestamps_.delta_s;
    const base::Point3D delta_rad_s =
        (input_delta_norm_rad_s < max_delta_rad_s) ?
        input_delta_rad_s :
        input_delta_rad_s.normalized() * max_delta_rad_s;

    status_.state.robot.desired_w_LR += delta_rad_s;
    // We only allow a z value.
    status_.state.robot.desired_w_LR.x() =
        status_.state.robot.desired_w_LR.y() = 0.0;
  }

  void MoveLegsForLR(std::vector<QC::Leg>* legs_R) {
    const double dt = timestamps_.delta_s;
    const auto& desired_w_LR = status_.state.robot.desired_w_LR;
    const auto& desired_v_mm_s_R = status_.state.robot.desired_v_mm_s_R;

    const auto& v_mm_s = desired_v_mm_s_R;

    // For now, we'll just do the dumb zeroth order integration.

    const Sophus::SE3d pose_T2_T1(
        Sophus::SO3d(
            Eigen::AngleAxisd(-dt * desired_w_LR.z(), Eigen::Vector3d::UnitZ())
            .toRotationMatrix()),
        -v_mm_s * dt);

    for (auto& leg_R : *legs_R) {
      if (!leg_R.stance) { continue; }

      leg_R.position_mm = pose_T2_T1 * leg_R.position_mm;

      // We don't want to change the Z velocity, but do want to force
      // the X and Y, since the LR frame movement is the only thing
      // that should be happening for a leg in stance configuration.
      leg_R.velocity_mm_s.head<2>() =
          -v_mm_s.head<2>() - desired_w_LR.cross(leg_R.position_mm).head<2>();
    }
  }

  void ControlLegs_R(std::vector<QC::Leg> legs_R) {
    control_log_->legs_R = std::move(legs_R);
    std::sort(control_log_->legs_R.begin(),
              control_log_->legs_R.end(),
              [](const auto& lhs, const auto& rhs) {
                return lhs.leg_id < rhs.leg_id;
              });

    const Sophus::SE3d pose_mm_BR = status_.state.robot.pose_mm_RB.inverse();

    std::vector<QC::Leg> legs_B;
    for (const auto& leg_R : control_log_->legs_R) {
      legs_B.push_back(pose_mm_BR * leg_R);
    }

    ControlLegs_B(std::move(legs_B));
  }

  void ControlLegs_B(std::vector<QC::Leg> legs_B) {
    control_log_->legs_B = std::move(legs_B);
    std::sort(control_log_->legs_B.begin(),
              control_log_->legs_B.end(),
              [](const auto& lhs, const auto& rhs) {
                return lhs.leg_id < rhs.leg_id;
              });

    // Apply Z bounds.
    for (auto& leg_B : control_log_->legs_B) {
      leg_B.position_mm.z() = std::max(
          config_.bounds.min_z_B,
          std::min(config_.bounds.max_z_B, leg_B.position_mm.z()));
    }

    std::vector<QC::Joint> out_joints;

    const std::vector<IkSolver::Joint> current_joints = [&]() {
      std::vector<IkSolver::Joint> result;
      for (const auto& joint : status_.state.joints) {
        IkSolver::Joint ik_joint;
        ik_joint.id = joint.id;
        ik_joint.angle_deg = joint.angle_deg;
        ik_joint.velocity_dps = joint.velocity_dps;
        ik_joint.torque_Nm = joint.torque_Nm;
        result.push_back(ik_joint);
      }
      return result;
    }();

    for (const auto& leg_B : control_log_->legs_B) {
      const auto& qleg = GetLeg(leg_B.leg_id);

      auto add_joints = [&](auto base) {
        base.id = qleg.config.ik.shoulder.id;
        out_joints.push_back(base);
        base.id = qleg.config.ik.femur.id;
        out_joints.push_back(base);
        base.id = qleg.config.ik.tibia.id;
        out_joints.push_back(base);
      };
      if (!leg_B.power) {
        QC::Joint out_joint;
        out_joint.power = false;
        add_joints(out_joint);
      } else if (leg_B.zero_velocity) {
        QC::Joint out_joint;
        out_joint.power = true;
        out_joint.zero_velocity = true;
        add_joints(out_joint);
      } else {
        const Sophus::SE3d pose_mm_GB = qleg.pose_mm_BG.inverse();

        IkSolver::Effector effector_B;

        effector_B.pose_mm = leg_B.position_mm;
        effector_B.velocity_mm_s = leg_B.velocity_mm_s;
        effector_B.force_N = leg_B.force_N;

        const auto effector_G = pose_mm_GB * effector_B;

        const auto result = qleg.ik.Inverse(effector_G, current_joints);

        if (!result) {
          // Hmmm, for now, we'll just command all zero velocity, but
          // in the future we should probably just stick to the
          // command we had the last cycle?
          QC::Joint out_joint;
          out_joint.power = true;
          out_joint.zero_velocity = true;
          add_joints(out_joint);
        } else {
          for (const auto& joint_angle : *result) {
            QC::Joint out_joint;
            out_joint.id = joint_angle.id;
            out_joint.power = true;
            out_joint.angle_deg = joint_angle.angle_deg;
            out_joint.torque_Nm = joint_angle.torque_Nm;
            out_joint.velocity_dps = joint_angle.velocity_dps;

            // TODO: Propagate kp and kd from 3D into joints.
            out_joint.kp_scale = leg_B.kp_scale ? leg_B.kp_scale->x() :
                std::optional<double>();
            out_joint.kd_scale = leg_B.kd_scale ? leg_B.kd_scale->x() :
                std::optional<double>();
            out_joints.push_back(out_joint);
          }
        }
      }
    }

    ControlJoints(out_joints);
  }

  void ControlJoints(std::vector<QC::Joint> joints) {
    control_log_->joints = joints;
    std::sort(control_log_->joints.begin(),
              control_log_->joints.end(),
              [](const auto& lhs, const auto& rhs) {
                return lhs.id < rhs.id;
              });

    EmitControl();
  }

  void EmitControl() {
    control_log_->timestamp = Now();
    control_signal_(control_log_);

    client_command_ = {};
    for (const auto& joint : control_log_->joints) {
      client_command_.push_back({});
      auto& request = client_command_.back();
      request.id = joint.id;

      constexpr double kInf = std::numeric_limits<double>::infinity();
      std::optional<double> max_torque_Nm =
          (parameters_.max_torque_Nm >= 0.0 || !!joint.max_torque_Nm) ?
          std::min(parameters_.max_torque_Nm < 0.0 ?
                   kInf : parameters_.max_torque_Nm,
                   joint.max_torque_Nm.value_or(kInf)) :
          std::optional<double>();

      const auto mode = [&]() {
        if (joint.power == false) {
          return moteus::Mode::kStopped;
        } else if (joint.zero_velocity) {
          return moteus::Mode::kZeroVelocity;
        } else {
          return moteus::Mode::kPosition;
        }
      }();

      request.request.WriteSingle(moteus::kMode, static_cast<int8_t>(mode));

      auto& values = values_cache_;
      values.resize(0);

      if (mode == moteus::Mode::kPosition) {
        const double sign = GetSign(joint.id);

        if (joint.angle_deg != 0.0) { values.resize(1); }
        if (joint.velocity_dps != 0.0) { values.resize(2); }
        if (joint.torque_Nm != 0.0) { values.resize(3); }
        if (joint.kp_scale) { values.resize(4); }
        if (joint.kd_scale) { values.resize(5); }
        if (max_torque_Nm) { values.resize(6); }
        if (joint.stop_angle_deg) { values.resize(7); }

        for (size_t i = 0; i < values.size(); i++) {
          switch (i) {
            case 0: {
              values[i] = moteus::WritePosition(
                  sign * joint.angle_deg, moteus::kInt16);
              break;
            }
            case 1: {
              values[i] = moteus::WriteVelocity(
                  sign * joint.velocity_dps, moteus::kInt16);
              break;
            }
            case 2: {
              values[i] = moteus::WriteTorque(
                  sign * joint.torque_Nm, moteus::kInt16);
              break;
            }
            case 3: {
              values[i] = moteus::WritePwm(
                  joint.kp_scale.value_or(1.0), moteus::kInt16);
              break;
            }
            case 4: {
              values[i] = moteus::WritePwm(
                  joint.kd_scale.value_or(1.0), moteus::kInt16);
              break;
            }
            case 5: {
              values[i] = moteus::WriteTorque(
                  max_torque_Nm.value_or(kInf),
                  moteus::kInt16);
              break;
            }
            case 6: {
              values[i] = moteus::WritePosition(
                  sign * joint.stop_angle_deg.value_or(
                      std::numeric_limits<double>::quiet_NaN()),
                  moteus::kInt16);
              break;
            }

          }
        }

        if (!values.empty()) {
          request.request.WriteMultiple(moteus::kCommandPosition, values);
        }
      }

    }
  }

  boost::posix_time::ptime Now() {
    return mjlib::io::Now(executor_.context());
  }

  const Leg& GetLeg(int id) const {
    for (const auto& leg : legs_) {
      if (leg.leg == id) { return leg; }
    }
    mjlib::base::AssertNotReached();
  }

  bool MoveLegs(
      std::vector<QC::Leg>* legs_R,
      double desired_velocity_mm_s,
      const std::vector<std::pair<int, base::Point3D>>& command_pose_mm_R,
      double extra_z_N,
      base::Point3D velocity_mask = base::Point3D(1., 1., 1),
      base::Point3D velocity_inverse_mask = base::Point3D(0., 0., 0.)) const {

    bool done = true;

    const int stance_legs =
        std::count_if(legs_R->begin(), legs_R->end(),
                      [](auto& leg_R) { return leg_R.stance; });

    // We do each leg independently.
    for (const auto& pair : command_pose_mm_R) {
      auto& leg_R = [&]() -> QC::Leg& {
        for (auto& leg_R : *legs_R) {
          if (leg_R.leg_id == pair.first) { return leg_R; }
        }
        mjlib::base::AssertNotReached();
      }();

      const base::Point3D error_mm = pair.second - leg_R.position_mm;
      const double error_norm_mm = error_mm.norm();
      if (error_norm_mm > 1.0) {
        // If any leg has distance left to move, then we're not done.
        done = false;
      }
      const double max_velocity_mm_s =
          config_.bounds.max_acceleration_mm_s2 *
          std::sqrt(2 * error_norm_mm /
                    config_.bounds.max_acceleration_mm_s2);

      const double velocity_mm_s =
          std::min(desired_velocity_mm_s, max_velocity_mm_s);

      const double delta_mm =
          std::min(
              error_norm_mm,
              velocity_mm_s * timestamps_.delta_s);
      leg_R.position_mm += error_mm.normalized() * delta_mm;
      leg_R.velocity_mm_s =
          velocity_inverse_mask.asDiagonal() * leg_R.velocity_mm_s  +
          velocity_mask.asDiagonal() * error_mm.normalized() * velocity_mm_s;

      const double gravity_N =
          leg_R.stance ?
          config_.mass_kg * kGravity :
          0.0;
      const double force_z_N =
          stance_legs == 0 ?
          0.0 :
          (gravity_N + extra_z_N) / stance_legs;
      leg_R.force_N = base::Point3D(0, 0, force_z_N);
    }

    return done;
  }

  bool MoveLegsZ(
      std::vector<QC::Leg>* legs_R,
      double desired_velocity_mm_s,
      double desired_height_mm,
      double extra_z_N) const {
    std::vector<std::pair<int, base::Point3D>> desired_poses_mm_R;
    for (const auto& leg : *legs_R) {
      base::Point3D pose_mm_R = leg.position_mm;
      pose_mm_R.z() = desired_height_mm;
      desired_poses_mm_R.push_back(std::make_pair(leg.leg_id, pose_mm_R));
    }

    return MoveLegs(
        legs_R,
        desired_velocity_mm_s,
        desired_poses_mm_R,
        extra_z_N,
        base::Point3D(0, 0, 1),
        base::Point3D(1, 1, 0));
  }

  boost::asio::executor executor_;
  Parameters parameters_;
  boost::program_options::options_description options_;

  base::LogRef log_ = base::GetLogInstance("QuadrupedControl");

  Config config_;
  std::deque<Leg> legs_;

  QuadrupedControl::Status status_;
  QC current_command_;

  std::array<ControlLog, 2> control_logs_;
  ControlLog* control_log_ = &control_logs_[0];
  ControlLog* old_control_log_ = &control_logs_[1];

  mjlib::io::RepeatingTimer timer_;
  using Client = MultiplexClient::Client;

  Client* client_ = nullptr;

  using Request = std::vector<Client::IdRequest>;
  Request status_request_;
  Client::Reply status_reply_;

  Request client_command_;
  Client::Reply client_command_reply_;

  bool outstanding_ = false;

  struct Timestamps {
    boost::posix_time::ptime last_cycle_start;
    double delta_s = 0.0;

    boost::posix_time::ptime cycle_start;
    boost::posix_time::ptime status_done;
    boost::posix_time::ptime control_done;
    boost::posix_time::ptime command_done;
  } timestamps_;

  boost::signals2::signal<void (const Status*)> status_signal_;
  boost::signals2::signal<void (const CommandLog*)> command_signal_;
  boost::signals2::signal<void (const ControlLog*)> control_signal_;

  std::vector<moteus::Value> values_cache_;
};

QuadrupedControl::QuadrupedControl(base::Context& context)
    : impl_(std::make_unique<Impl>(context)) {}

QuadrupedControl::~QuadrupedControl() {}

void QuadrupedControl::AsyncStart(mjlib::io::ErrorCallback callback) {
  impl_->AsyncStart(std::move(callback));
}

void QuadrupedControl::SetClient(MultiplexClient::Client* client) {
  impl_->client_ = client;
}

void QuadrupedControl::Command(const QC& command) {
  impl_->Command(command);
}

const QuadrupedControl::Status& QuadrupedControl::status() const {
  return impl_->status_;
}

QuadrupedControl::Parameters* QuadrupedControl::parameters() {
  return &impl_->parameters_;
}

boost::program_options::options_description* QuadrupedControl::options() {
  return &impl_->options_;
}

}
}
