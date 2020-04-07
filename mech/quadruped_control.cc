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

#include "mech/quadruped_control.h"

#include <fstream>

#include <fmt/format.h>

#include "mjlib/base/clipp_archive.h"
#include "mjlib/base/json5_read_archive.h"

#include "mjlib/io/now.h"
#include "mjlib/io/repeating_timer.h"

#include "base/common.h"
#include "base/logging.h"
#include "base/sophus.h"

#include "mech/attitude_data.h"
#include "mech/mammal_ik.h"
#include "mech/moteus.h"
#include "mech/quadruped_config.h"
#include "mech/trajectory.h"

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

template <typename Iter, typename Functor>
auto Max(Iter begin, Iter end, Functor f) -> decltype(f(*begin)) {
  using T = decltype(f(*begin));

  MJ_ASSERT(begin != end);

  T current = f(*begin);
  Iter it = begin;
  ++it;
  for (; it != end; ++it) {
    const auto value = f(*it);
    if (value > current) { current = value; }
  }
  return current;
}

template <typename Iter, typename Functor>
auto Min(Iter begin, Iter end, Functor f) -> decltype(f(*begin)) {
  using T = decltype(f(*begin));

  MJ_ASSERT(begin != end);

  T current = f(*begin);
  Iter it = begin;
  ++it;
  for (; it != end; ++it) {
    const auto value = f(*it);
    if (value < current) { current = value; }
  }
  return current;
}

constexpr double kGravity = 9.81;

struct ReportedServoConfig {
  boost::posix_time::ptime timestamp;

  struct Servo {
    int id = 0;
    int rezero_state = 0;
    int register_map_version = -1;
    std::array<uint8_t, 12> serial_number = {};

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(id));
      a->Visit(MJ_NVP(rezero_state));
      a->Visit(MJ_NVP(register_map_version));
      a->Visit(MJ_NVP(serial_number));
    }
  };

  std::vector<Servo> servos;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(timestamp));
    a->Visit(MJ_NVP(servos));
  }
};

using Config = QuadrupedConfig;

struct MammalJoint {
  double shoulder_deg = 0.0;
  double femur_deg = 0.0;
  double tibia_deg = 0.0;
};

struct Leg {
  int leg = 0;
  Config::Leg config;
  Sophus::SE3d pose_mm_BG;
  MammalIk ik;

  base::Point3D stand_up_R;
  base::Point3D idle_R;
  MammalJoint resolved_stand_up_joints;

  Leg(const Config::Leg& config_in,
      const Config::StandUp& stand_up,
      double stand_height_mm,
      double idle_x_mm,
      double idle_y_mm)
      : leg(config_in.leg),
        config(config_in),
        pose_mm_BG(config_in.pose_mm_BG),
        ik(config_in.ik) {
    // The idle and standup poses assume a null RB transform
    const Sophus::SE3d pose_mm_RB;
    const auto pose_mm_RG = pose_mm_RB * config.pose_mm_BG;

    const base::Point3D pose_mm_R = [&]() {
      auto result = stand_up.pose_mm_R;
      if (config.pose_mm_BG.translation().x() < 0.0) { result.x() *= -1; }
      if (config.pose_mm_BG.translation().y() < 0.0) { result.y() *= -1; }
      return result;
    }();

    const base::Point3D pose_mm_G = pose_mm_RG.inverse() * pose_mm_R;

    IkSolver::Effector effector_G;
    effector_G.pose_mm = pose_mm_G;
    const auto resolved = ik.Inverse(effector_G, {});
    auto get_resolved = [&](int id) {
      MJ_ASSERT(!!resolved);
      for (const auto& joint_angle : *resolved) {
        if (joint_angle.id == id) { return joint_angle.angle_deg; }
      }
      mjlib::base::AssertNotReached();
    };
    resolved_stand_up_joints.shoulder_deg = get_resolved(config.ik.shoulder.id);
    resolved_stand_up_joints.femur_deg = get_resolved(config.ik.femur.id);
    resolved_stand_up_joints.tibia_deg = get_resolved(config.ik.tibia.id);

    stand_up_R = pose_mm_R;
    base::Point3D tf = config.pose_mm_BG.translation();
    idle_R = base::Point3D(
        idle_x_mm * ((tf.x() > 0.0) ? 1.0 : -1.0),
        idle_y_mm * ((tf.y() > 0.0) ? 1.0 : -1.0),
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
  Impl(base::Context& context,
       ClientGetter client_getter,
       ImuGetter imu_getter)
      : executor_(context.executor),
        timer_(executor_),
        client_getter_(client_getter),
        imu_getter_(imu_getter) {
    context.telemetry_registry->Register("qc_status", &status_signal_);
    context.telemetry_registry->Register("qc_command", &command_signal_);
    context.telemetry_registry->Register("qc_control", &control_signal_);
    context.telemetry_registry->Register("imu", &imu_signal_);
    context.telemetry_registry->Register("servo_config", &servo_config_signal_);
  }

  void AsyncStart(mjlib::io::ErrorCallback callback) {
    client_ = client_getter_();
    if (parameters_.enable_imu) {
      imu_client_ = imu_getter_();
    }

    BOOST_ASSERT(!!client_);

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
    const auto now = Now();
    const bool higher_priority = command.priority >= current_command_.priority;
    const bool stale =
        !current_command_timestamp_.is_not_a_date_time() &&
        (mjlib::base::ConvertDurationToSeconds(
            now - current_command_timestamp_) > parameters_.command_timeout_s);
    if (!higher_priority && !stale) {
      return;
    }

    CommandLog command_log;
    command_log.timestamp = now;
    command_log.command = &command;

    current_command_ = command;
    current_command_timestamp_ = now;

    command_signal_(&command_log);
  }

  void Configure() {
    for (const auto& leg : config_.legs) {
      legs_.emplace_back(leg, config_.stand_up, config_.stand_height_mm,
                         config_.idle_x_mm, config_.idle_y_mm);
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

    config_status_request_ = {};
    for (const auto& joint : config_.joints) {
      config_status_request_.push_back({});
      auto& current = config_status_request_.back();
      current.id = joint.id;

      // While configuring, we request a few more things.
      current.request.ReadMultiple(moteus::Register::kMode, 4, 1);
      current.request.ReadMultiple(moteus::Register::kRezeroState, 4, 0);
      current.request.ReadMultiple(moteus::Register::kRegisterMapVersion, 1, 2);
      current.request.ReadMultiple(moteus::Register::kSerialNumber, 3, 2);
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

    // Ask for the IMU and the servo data simultaneously.
    outstanding_status_requests_ = 1;
    auto* request = [&]() {
      if (status_.mode == QM::kConfiguring) {
        return &config_status_request_;
      }
      return &status_request_;
    }();
    client_->AsyncRegisterMultiple(*request, &status_reply_,
                                   std::bind(&Impl::HandleStatus, this, pl::_1));
    if (imu_client_) {
      imu_client_->ReadImu(
          &imu_data_, std::bind(&Impl::HandleStatus, this, pl::_1));
      outstanding_status_requests_++;
    }
  }

  void HandleStatus(const mjlib::base::error_code& ec) {
    mjlib::base::FailIf(ec);

    outstanding_status_requests_--;
    if (outstanding_status_requests_ > 0) { return; }

    timestamps_.status_done = Now();

    if (!!imu_client_) {
      imu_signal_(&imu_data_);
    }

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

    if (status_.mode == QM::kConfiguring) {
      // Try to update our config structure.
      UpdateConfiguringStatus();
    }

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

    // We should only be here if we have something for all our joints.
    MJ_ASSERT(status_.state.joints.size() == 12);

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

  void UpdateConfiguringStatus() {
    auto& reported = reported_servo_config_;

    auto find_or_make_servo = [&](int id) -> ReportedServoConfig::Servo& {
      for (auto& servo : reported.servos) {
        if (servo.id == id) { return servo; }
      }
      reported.servos.push_back({});
      auto& result = reported.servos.back();
      result.id = id;
      return result;
    };

    for (const auto& reply : status_reply_.replies) {
      auto& out_servo = find_or_make_servo(reply.id);

      for (const auto& pair : reply.reply) {
        const auto* maybe_value = std::get_if<moteus::Value>(&pair.second);
        if (!maybe_value) { continue; }
        const auto& value = *maybe_value;
        switch (static_cast<moteus::Register>(pair.first)) {
          case moteus::kRezeroState: {
            out_servo.rezero_state = moteus::ReadInt(value);
            break;
          }
          case moteus::kRegisterMapVersion: {
            out_servo.register_map_version = moteus::ReadInt(value);
            break;
          }
          case moteus::kSerialNumber1: {
            const auto sn = static_cast<uint32_t>(moteus::ReadInt(value));
            std::memcpy(&out_servo.serial_number[0], &sn, sizeof(sn));
            break;
          }
          case moteus::kSerialNumber2: {
            const auto sn = static_cast<uint32_t>(moteus::ReadInt(value));
            std::memcpy(&out_servo.serial_number[4], &sn, sizeof(sn));
            break;
          }
          case moteus::kSerialNumber3: {
            const auto sn = static_cast<uint32_t>(moteus::ReadInt(value));
            std::memcpy(&out_servo.serial_number[8], &sn, sizeof(sn));
            break;
          }
          default: {
            break;
          }
        }
      }
    }

    reported.timestamp = Now();
    servo_config_signal_(&reported);
  }

  void RunControl() {
    if (current_command_.mode != status_.mode) {
      MaybeChangeMode();
    }

    switch (status_.mode) {
      case QM::kConfiguring: {
        DoControl_Configuring();
        break;
      }
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
      case QM::kWalk: {
        DoControl_Walk();
        break;
      }
      case QM::kNumModes: {
        mjlib::base::AssertNotReached();
      }
    }
  }

  void MaybeChangeMode() {
    const auto old_mode = status_.mode;
    if (status_.mode == QM::kConfiguring) {
      // We can only leave this mode after we have heard from all the
      // servos, they are all rezeroed, and have an appropriate
      // register map version.
      if (!IsConfiguringDone()) { return; }
    }

    switch (current_command_.mode) {
      case QM::kConfiguring:
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
      case QM::kRest:
      case QM::kJump:
      case QM::kWalk: {
        // This can only be done from certain configurations, where we
        // know all four legs are on the ground.  Modify our command
        // to try and get into that state.
        //
        // TODO(jpieper): Add another layer of control which cycles
        // between these lower level ones to reduce confusion rather
        // than tucking this logic into state transitions.
        if (status_.mode == QM::kStopped ||
            status_.mode == QM::kZeroVelocity) {
          status_.mode = QM::kStandUp;
          status_.state.stand_up = {};
        } else if ((status_.mode == QM::kRest &&
                    status_.state.rest.done) ||
                   (status_.mode == QM::kStandUp &&
                    status_.state.stand_up.mode ==
                    QuadrupedState::StandUp::Mode::kDone)) {
          status_.mode = current_command_.mode;

          status_.state.jump.command = current_command_.jump.value_or(
              QuadrupedCommand::Jump());

        } else if (status_.mode == QM::kJump &&
                   status_.state.jump.mode ==
                   QuadrupedState::Jump::Mode::kDone) {
          status_.mode = current_command_.mode;
        } else if (status_.mode == QM::kWalk) {
          // We can only leave the walk state when our desired
          // velocities are all 0 and all four legs are on the ground.
          if (status_.state.walk.idle_count >= 2 &&
              all_legs_stance()) {
            status_.state.jump.command = current_command_.jump.value_or(
                QuadrupedCommand::Jump());
            status_.mode = current_command_.mode;
          } else {
            current_command_.v_mm_s_R = {};
            current_command_.w_LR = {};
          }
        } else {
          // We can't switch, just wait I guess.
        }

        break;
      }
    }

    if (status_.mode != old_mode) {
      log_.warn(fmt::format("Changed mode from {} to {}",
                            old_mode, status_.mode));
      switch (old_mode) {
        case QM::kStandUp: {
          status_.state.stand_up = {};
          break;
        }
        case QM::kJump: {
          status_.state.jump = {};
          break;
        }
        case QM::kWalk: {
          status_.state.walk = {};
          break;
        }
        case QM::kRest: {
          status_.state.rest = {};
        }
        case QM::kConfiguring:
        case QM::kStopped:
        case QM::kFault:
        case QM::kZeroVelocity:
        case QM::kJoint:
        case QM::kLeg:
        case QM::kNumModes: {
          break;
        }
      }
      status_.mode_start = Now();
    }
  }

  bool IsConfiguringDone() {
    // We must have heard from all 12 servos.
    if (reported_servo_config_.servos.size() != 12) {
      status_.fault = "missing servos";
      return false;
    }

    // All of them must have been rezerod and have the current
    // register map.
    for (const auto& servo : reported_servo_config_.servos) {
      if (servo.rezero_state == 0) {
        status_.fault = fmt::format("servo {} not rezerod", servo.id);
        return false;
      }
      if (servo.register_map_version != moteus::kCurrentRegisterMapVersion) {
        status_.fault = fmt::format("servo {} has incorrect register version {}",
                                    servo.id, servo.register_map_version);
        return false;
      }
    }
    status_.fault = "";
    return true;
  }

  void DoControl_Configuring() {
    EmitStop();
  }

  void DoControl_Stopped() {
    status_.fault = "";

    EmitStop();
  }

  void EmitStop() {
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

      if (!check(leg.config.ik.shoulder.id,
                 leg.resolved_stand_up_joints.shoulder_deg)) {
        return false;
      }
      if (!check(leg.config.ik.femur.id,
                 leg.resolved_stand_up_joints.femur_deg)) {
        return false;
      }
      if (!check(leg.config.ik.tibia.id,
                 leg.resolved_stand_up_joints.tibia_deg)) {
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

      add_joint(leg.config.ik.shoulder.id,
                leg.resolved_stand_up_joints.shoulder_deg);
      add_joint(leg.config.ik.femur.id,
                leg.resolved_stand_up_joints.femur_deg);
      add_joint(leg.config.ik.tibia.id,
                leg.resolved_stand_up_joints.tibia_deg);
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
      sleg.target_mm_R = sleg.pose_mm_R;
      sleg.target_mm_R.z() = config_.stand_height_mm;
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

      // Ensure all gains are back to their default and that
      // everything is marked as in stance.
      for (auto& leg_R : legs_R) {
        leg_R.kp_scale = {};
        leg_R.kd_scale = {};
        leg_R.stance = 1.0;
        leg_R.landing = false;
      }
      status_.state.rest.done = MoveLegsFixedSpeedZ(
          all_leg_ids_,
          &legs_R,
          config_.rest.velocity_mm_s,
          config_.stand_height_mm);
    } else {
      for (const auto& leg : legs_) {
        QC::Leg leg_R;
        leg_R.leg_id = leg.leg;
        leg_R.power = true;
        // TODO: This is unlikely to be a good idea.
        leg_R.position_mm = leg.idle_R;
        legs_R.push_back(leg_R);
      }
      status_.state.rest.done = true;
    }

    UpdateLegsStanceForce(&legs_R, 0.0);

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

    double extra_z_N = 0.0;
    auto& js = status_.state.jump;
    std::optional<JM> previous_jump_mode;

    while (true) {
      // We should only loop here if our jumping state is different
      // from what it was the previous time.
      auto legs_R = old_control_log_->legs_R;

      if (!!previous_jump_mode) {
        MJ_ASSERT(status_.state.jump.mode != *previous_jump_mode);
      }
      previous_jump_mode = status_.state.jump.mode;

      switch (status_.state.jump.mode) {
        case JM::kLowering: {
          // Move legs to take into account LR rates.
          MoveLegsForLR(&legs_R);

          // Lower all legs until they reach the lower_height.
          const bool done = MoveLegsFixedSpeedZ(
              all_leg_ids_,
              &legs_R,
              config_.jump.lower_velocity_mm_s,
              config_.jump.lower_height_mm);
          if (done) {
            status_.state.jump.mode = JM::kPushing;
            status_.state.jump.velocity_mm_s = 0.0;
            status_.state.jump.acceleration_mm_s2 =
                js.command.acceleration_mm_s2;
            // Loop around and do the pushing behavior.
            continue;
          }
          break;
        }
        case JM::kPushing: {
          MoveLegsForLR(&legs_R);

          status_.state.jump.velocity_mm_s +=
              js.command.acceleration_mm_s2 * timestamps_.delta_s;
          const bool done = MoveLegsFixedSpeedZ(
              all_leg_ids_,
              &legs_R,
              js.velocity_mm_s,
              config_.jump.upper_height_mm);
          extra_z_N = config_.mass_kg * js.acceleration_mm_s2 * 0.001;
          if (done) {
            js.mode = JM::kRetracting;
            js.velocity_mm_s = 0.0;
            js.acceleration_mm_s2 = 0.0;
            for (auto& leg_R : legs_R) {
              leg_R.stance = 0.0;
            }
            // Loop around and do the retracting behavior.
            old_control_log_->legs_R = legs_R;
            continue;
          }
          break;
        }
        case JM::kRetracting: {
          // We need to get our legs back into a lateral position
          // suitable for the landing phase.

          // TODO: This could be updated to reach at a given time (with
          // a configurable time), instead of a fixed speed.
          const bool done = MoveLegsFixedSpeed(
              &legs_R,
              config_.jump.retract_velocity_mm_s,
              [&]() {
                std::vector<std::pair<int, base::Point3D>> result;
                for (const auto& leg : legs_) {
                  result.push_back(std::make_pair(leg.leg, leg.idle_R));
                }
                return result;
              }());

          if (done) {
            js.mode = JM::kFalling;
            for (auto& leg_R : legs_R) {
              leg_R.velocity_mm_s = base::Point3D();
              leg_R.landing = true;
            }
            // Loop around and do the falling behavior.
            old_control_log_->legs_R = legs_R;
            continue;
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
            const auto kp = config_.jump.land_kp;
            leg_R.kp_scale = base::Point3D(kp, kp, kp);
            const auto kd = config_.jump.land_kd;
            leg_R.kd_scale = base::Point3D(kd, kd, kd);
          }

          // Wait for our legs to be pushed up by a certain distance,
          // which will indicate we have made contact with the ground.
          const double average_height_mm = Average(
              status_.state.legs_B.begin(),
              status_.state.legs_B.end(),
              [](const auto& leg_B) {
                return leg_B.position_mm.z();
              });
          const double max_height_mm = Max(
              status_.state.legs_B.begin(),
              status_.state.legs_B.end(),
              [](const auto& leg_B) {
                return leg_B.position_mm.z();
              });
          // We require the average to be below a threshold, and for
          // all legs to have made contact of some kind.
          const double average_error_mm =
              average_height_mm - config_.stand_height_mm;
          const double max_error_mm =
              max_height_mm - config_.stand_height_mm;
          if (average_error_mm < -config_.jump.land_threshold_mm &&
              max_error_mm < -0.5 * config_.jump.land_threshold_mm) {
            js.mode = JM::kLanding;
            // Loop around and start landing.
            continue;
          }
          break;
        }
        case JM::kLanding: {
          MoveLegsForLR(&legs_R);

          const auto landing_result = SetLandingParameters(&legs_R);
          extra_z_N = landing_result.extra_z_N;

          if (landing_result.done) {
            // We are either done, or going to start jumping again.

            // TODO(jpieper): Condition switching to rest on us having a
            // sufficiently low R frame velocity.

            if (js.command.repeat &&
                current_command_.mode == QM::kJump) {
              js.mode = JM::kPushing;

              // First, latch in our current jump command.
              if (!!current_command_.jump) {
                status_.state.jump.command = *current_command_.jump;
              }

              // Then, for the pushing phase, switch back to whatever
              // acceleration we were commanded.
              js.acceleration_mm_s2 = js.command.acceleration_mm_s2;
              continue;
            } else {
              js.mode = JM::kDone;

              // Switch our top level command back to rest to make life
              // more convenient.
              current_command_.mode = QM::kRest;
            }
          }
          break;
        }
        case JM::kDone: {
          DoControl_Rest();
          return;
        }
      }

      // If we make it here, then we haven't skipped back to redo our
      // loop.  Thus we can actually emit our control.
      UpdateLegsStanceForce(&legs_R, extra_z_N);

      ControlLegs_R(std::move(legs_R));
      return;
    }
  }

  struct LandingResult {
    double extra_z_N = 0.0;
    bool done = false;
  };

  LandingResult SetLandingParameters(std::vector<QC::Leg>* legs_R) {
    auto increase_gain = [&](auto& gain) {
      if (!!gain) {
        const auto old_value = gain->x();
        const auto new_value = (
            old_value + timestamps_.delta_s * config_.jump.land_gain_increase);
        if (new_value > 1.0) {
          gain = {};
        } else {
          gain = base::Point3D(new_value, new_value, new_value);
        }
      }
    };

    // Turn our gains back up.
    for (auto& leg_R : *legs_R) {
      increase_gain(leg_R.kp_scale);
      increase_gain(leg_R.kd_scale);
    }

    auto get_vel = [](const auto& leg_B) { return leg_B.velocity_mm_s.z(); };
    auto& js = status_.state.jump;

    const double average_velocity_mm_s = Average(
        status_.state.legs_B.begin(),
        status_.state.legs_B.end(),
        get_vel);
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
        std::max(
            10.0,
            average_height_mm - config_.jump.lower_height_mm);

    constexpr double kFudge = 0.7;
    js.acceleration_mm_s2 =
        std::max(
            config_.jump.min_acceleration_mm_s2,
            std::min(
                config_.bounds.max_acceleration_mm_s2,
                kFudge * 0.5 * std::pow(average_velocity_mm_s, 2) / error_mm));

    const double command_height_mm =
        std::max(average_height_mm, config_.jump.lower_height_mm);
    const double limited_velocity_mm_s =
        std::min(average_velocity_mm_s, -config_.jump.lower_velocity_mm_s);

    const double command_velocity_mm_s =
        (average_height_mm > config_.jump.lower_height_mm) ?
        limited_velocity_mm_s : 0.0;

    for (auto& leg_R : *legs_R) {
      const auto& pose_mm_RB = status_.state.robot.pose_mm_RB;
      auto desired_B = pose_mm_RB.inverse() * leg_R.position_mm;
      desired_B.z() = command_height_mm;

      leg_R.position_mm = pose_mm_RB * desired_B;
      leg_R.velocity_mm_s.z() = command_velocity_mm_s;
      leg_R.stance = 1.0;
      leg_R.landing = false;
    }

    LandingResult result;
    result.extra_z_N = config_.jump.landing_force_scale * config_.mass_kg *
        js.acceleration_mm_s2 * 0.001;

    const double min_velocity_mm_s = Min(
        status_.state.legs_B.begin(),
        status_.state.legs_B.end(),
        [](const auto& leg_B) {
          return leg_B.velocity_mm_s.z();
        });

    // We are done if we are low enough, or if no leg is still moving
    // down (that can happen when part of the leg begins to rest on
    // the ground).
    result.done = (
        (average_height_mm < config_.jump.lower_height_mm) ||
        (min_velocity_mm_s > -10));

    return result;
  }

  void DoControl_Walk() {
    // We always have some legs on the ground, so can nominally always
    // accelerate.
    UpdateCommandedLR();

    auto legs_R = old_control_log_->legs_R;

    // Update our phase.
    auto& ws = status_.state.walk;
    const auto old_phase = ws.phase;
    auto& phase = ws.phase;
    phase = std::fmod(
        phase + timestamps_.delta_s / config_.walk.cycle_time_s, 1.0);
    if (phase < old_phase) {
      // We have wrapped around.
      if (status_.state.robot.desired_v_mm_s_R.norm() == 0.0 &&
          status_.state.robot.desired_w_LR.norm() == 0.0) {
        ws.idle_count++;
      } else {
        ws.idle_count = 0;
      }
    }

    // For now, we are hard-coding 2 leg movement.

    // Check to see if we are in a step or not.
    const auto step_phase = [&]() -> std::optional<double> {
      if (phase > 0.0 && phase < config_.walk.step_phase) {
        return phase / config_.walk.step_phase;
      } else if (phase > 0.5 && phase < (0.5 + config_.walk.step_phase)) {
        return (phase - 0.5) / config_.walk.step_phase;
      }
      return {};
    }();

    const auto [ground_legs, step_legs] = [&]()
        -> std::pair<std::vector<int>, std::vector<int>> {
      if (!step_phase) { return {{0, 1, 2, 3}, {}}; }
      if (phase < 0.5) { return {{0, 3}, {1, 2}};}
      return {{1, 2}, {0, 3}};
    }();

    // All ground legs should be at the correct height and in stance.
    for (int ground_leg : ground_legs) {
      auto& leg_R = GetLeg_R(&legs_R, ground_leg);
      leg_R.stance = 1.0;
    }

    std::vector<std::pair<int, base::Point3D>> moving_legs_R;
    ws.moving_target_remaining_s = 0.0;

    // Accumulate our various control times.
    const auto step_time_s =
        config_.walk.cycle_time_s * config_.walk.step_phase;

    const auto start = 0.0;
    const auto release_end = start + config_.walk.step.release_time;
    const auto lift_end = release_end + config_.walk.step.lift_time;
    const auto lift_time_s = step_time_s * config_.walk.step.lift_time;
    const auto move_end =
        1.0 - (config_.walk.step.lower_time + config_.walk.step.load_time);
    const auto move_time = move_end - lift_end;
    const auto move_time_s = step_time_s * move_time;
    const auto lower_end = 1.0 - config_.walk.step.load_time;
    const auto lower_time_s = step_time_s * config_.walk.step.lower_time;

    const auto swing_targets_R = GetSwingTarget_R(
        (1.0 - config_.walk.step_phase) * config_.walk.cycle_time_s);

    // Move our legs that are in step.
    for (int step_leg_id : step_legs) {
      const auto sp = step_phase.value();

      auto& leg_R = GetLeg_R(&legs_R, step_leg_id);

      if (sp < release_end) {
        // Here, we are still in stance, but decreasing it.
        leg_R.stance = 1.0 - sp / config_.walk.step.release_time;
        leg_R.landing = false;
      } else if (sp < lift_end) {
        // We are lifting.
        leg_R.stance = 0.0;

        // We mark ourselves as landing so that we will keep
        // stationary w.r.t. the ground.
        leg_R.landing = true;

        leg_R.force_N = base::Point3D(0, 0, 0);

        // Use MoveLegsFixedSpeedZ to get a controlled deceleration
        // profile.
        const double speed = -0.5 * (
            std::sqrt(
                std::pow(config_.bounds.max_acceleration_mm_s2, 2.0) *
                std::pow(lift_time_s, 2.0) -
                4 * config_.bounds.max_acceleration_mm_s2 * config_.walk.lift_height_mm) -
            config_.bounds.max_acceleration_mm_s2 * lift_time_s);

        // If the speed isn't finite, then we cannot lift this far in
        // the time allotted. :(  Instead, lift as far as we can.
        if (!std::isfinite(speed)) {
          const double achievable_lift_height_mm =
              std::pow(0.5 * lift_time_s, 2) *
              config_.bounds.max_acceleration_mm_s2;
          MJ_ASSERT(achievable_lift_height_mm <= config_.walk.lift_height_mm);
          MoveLegsFixedSpeedZ(
              {step_leg_id},
              &legs_R,
              .5 * lift_time_s * config_.bounds.max_acceleration_mm_s2,
              config_.stand_height_mm - achievable_lift_height_mm);
        } else {
          MoveLegsFixedSpeedZ(
              {step_leg_id},
              &legs_R,
              speed,
              config_.stand_height_mm - config_.walk.lift_height_mm);
        }
      } else if (sp < move_end) {
        // Move to the target position (for now the idle position),
        // with an infinite acceleration profile attempting to reach
        // it exactly at the end of our travel time.
        ws.moving_target_remaining_s =
            move_time_s - (sp - lift_end) * step_time_s;
        auto target_R = [&]() {
          for (const auto& pair : swing_targets_R) {
            if (pair.first == step_leg_id) { return pair.second; }
          }
          mjlib::base::AssertNotReached();
        }();

        // We are moving to our end point at the lift height.
        target_R.z() =
            config_.stand_height_mm - config_.walk.lift_height_mm;

        moving_legs_R.push_back(std::make_pair(step_leg_id, target_R));

        leg_R.stance = 0.0;
        leg_R.landing = false;
      } else if (sp < lower_end) {
        leg_R.stance = 0.0;
        leg_R.landing = true;
        const auto kp = config_.walk.lower_kp;
        leg_R.kp_scale = base::Point3D(kp, kp, kp);
        const auto kd = config_.walk.lower_kd;
        leg_R.kd_scale = base::Point3D(kd, kd, kd);

        // TODO: Use the speed calculation from above instead of a fudge.
        constexpr double kSpeedFudge = 2.0;
        MoveLegsFixedSpeedZ(
            {step_leg_id}, &legs_R,
            kSpeedFudge * config_.walk.lift_height_mm / lower_time_s,
            config_.stand_height_mm);
      } else {
        const double load_fraction =
            (sp - lower_end) / config_.walk.step.load_time;
        leg_R.landing = false;
        leg_R.stance = std::max(0.001, load_fraction);
        leg_R.velocity_mm_s.z() = 0.0;

        const auto kp =
            (1.0 - config_.walk.lower_kp) * load_fraction + config_.walk.lower_kp;
        leg_R.kp_scale = base::Point3D(kp, kp, kp);
        const auto kd =
            (1.0 - config_.walk.lower_kd) * load_fraction + config_.walk.lower_kd;
        leg_R.kd_scale = base::Point3D(kd, kd, kd);
      }
    }

    // Reset the kp_scale of anything on the ground.
    for (int step_leg_id : ground_legs) {
      auto& leg_R = GetLeg_R(&legs_R, step_leg_id);
      leg_R.kp_scale = {};
      leg_R.kd_scale = {};
    }

    // Advance the legs which are landing or on the ground.
    MoveLegsForLR(&legs_R);

    // Update all the legs that are in flight.
    if (!moving_legs_R.empty()) {
      MoveLegsTargetTime(
          &legs_R,
          ws.moving_target_remaining_s,
          moving_legs_R);
    }

    // Update the other legs.
    MoveLegsFixedSpeedZ(
        ground_legs, &legs_R,
        // we should already be basically at the right height,
        // so this velocity isn't that meaningful.
        config_.rest.velocity_mm_s,
        config_.stand_height_mm);

    UpdateLegsStanceForce(&legs_R, 0.0);

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

  /// Return a good target location for each leg during the swing
  /// phase.  We just aim to spend half the time reaching the idle
  /// position and half the time going past it.
  ///
  /// @p stance_time_s is how long we expect the leg to be in contact
  /// with the ground.
  std::vector<std::pair<int, base::Point3D>>
  GetSwingTarget_R(double stance_time_s) {

    const auto& desired_w_LR = status_.state.robot.desired_w_LR;
    const auto& desired_v_mm_s_R = status_.state.robot.desired_v_mm_s_R;
    const double dt = 0.5 * stance_time_s;
    const auto& v_mm_s = desired_v_mm_s_R;

    const Sophus::SE3d pose_T2_T1(
        Sophus::SO3d(
            Eigen::AngleAxisd(dt * desired_w_LR.z(), Eigen::Vector3d::UnitZ())
            .toRotationMatrix()),
        v_mm_s * dt);

    std::vector<std::pair<int, base::Point3D>> result;

    for (const auto& leg : legs_) {
      base::Point3D position_mm_R = pose_T2_T1 * leg.idle_R;
      result.push_back(std::make_pair(leg.leg, position_mm_R));
    }
    return result;
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
      if (leg_R.stance == 0.0 && !leg_R.landing) { continue; }

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

  template <typename T>
  static QC::Leg& GetLeg_R(T* legs_R, int id) {
    for (auto& leg_R : *legs_R) {
      if (leg_R.leg_id == id) { return leg_R; }
    }
    mjlib::base::AssertNotReached();
  }

  const Leg& GetLeg(int id) const {
    for (auto& leg : legs_) {
      if (leg.leg == id) { return leg; }
    }
    mjlib::base::AssertNotReached();
  }

  void MoveLegsTargetTime(
      std::vector<QC::Leg>* legs_R,
      double remaining_s,
      const std::vector<std::pair<int, base::Point3D>>& command_pose_mm_R) const {
    for (const auto& pair : command_pose_mm_R) {
      auto& leg_R = GetLeg_R(legs_R, pair.first);

      // This only makes sense for things that are not on the ground.
      MJ_ASSERT(leg_R.stance == 0.0);

      const base::Point3D error_mm = pair.second - leg_R.position_mm;
      const double error_norm_mm = error_mm.norm();
      const double velocity_mm_s = error_norm_mm /
          std::max(timestamps_.delta_s, remaining_s);

      // For now, we'll do this as just an infinite acceleration
      // profile.

      const double delta_mm =
          std::min(
              error_norm_mm,
              velocity_mm_s * timestamps_.delta_s);
      leg_R.position_mm += error_mm.normalized() * delta_mm;
      leg_R.velocity_mm_s =
          error_mm.normalized() * velocity_mm_s;
      // Since we are not in stance.
      leg_R.force_N = base::Point3D(0, 0, 0);
    }
  }

  void UpdateLegsStanceForce(
      std::vector<QC::Leg>* legs_R,
      double extra_z_N) {
    const double stance_legs = [&]() {
      double result = 0.0;
      for (const auto& leg_R : *legs_R) {
        result += leg_R.stance;
      }
      return result;
    }();

    for (auto& leg_R : *legs_R) {
      const double gravity_N =
          leg_R.stance ?
          config_.mass_kg * kGravity :
          0.0;
      const double force_z_N =
          stance_legs == 0.0 ?
          0.0 :
          leg_R.stance * gravity_N / stance_legs + extra_z_N;
      leg_R.force_N = base::Point3D(0, 0, force_z_N);
    }
  }

  bool MoveLegsFixedSpeed(
      std::vector<QC::Leg>* legs_R,
      double desired_velocity_mm_s,
      const std::vector<std::pair<int, base::Point3D>>& command_pose_mm_R,
      base::Point3D velocity_mask = base::Point3D(1., 1., 1),
      base::Point3D velocity_inverse_mask = base::Point3D(0., 0., 0.)) const {

    bool done = true;

    // We do each leg independently.
    for (const auto& pair : command_pose_mm_R) {
      auto& leg_R = GetLeg_R(legs_R, pair.first);

      TrajectoryState initial{leg_R.position_mm, leg_R.velocity_mm_s};
      const auto result = CalculateAccelerationLimitedTrajectory(
          initial, pair.second, desired_velocity_mm_s,
          config_.bounds.max_acceleration_mm_s2,
          timestamps_.delta_s);

      leg_R.position_mm = result.pose_l;
      leg_R.velocity_mm_s =
          velocity_inverse_mask.asDiagonal() * leg_R.velocity_mm_s  +
          velocity_mask.asDiagonal() * result.velocity_l_s;

      if ((leg_R.position_mm - pair.second).norm() > 1.0) {
        done = false;
      }
    }

    return done;
  }

  bool MoveLegsFixedSpeedZ(
      const std::vector<int>& leg_ids,
      std::vector<QC::Leg>* legs_R,
      double desired_velocity_mm_s,
      double desired_height_mm) const {
    std::vector<std::pair<int, base::Point3D>> desired_poses_mm_R;

    for (int id : leg_ids) {
      const auto& leg_R = GetLeg_R(legs_R, id);
      base::Point3D pose_mm_R = leg_R.position_mm;
      pose_mm_R.z() = desired_height_mm;
      desired_poses_mm_R.push_back(std::make_pair(leg_R.leg_id, pose_mm_R));
    }

    return MoveLegsFixedSpeed(
        legs_R,
        desired_velocity_mm_s,
        desired_poses_mm_R,
        base::Point3D(0, 0, 1),
        base::Point3D(1, 1, 0));
  }

  bool all_legs_stance() const {
    for (const auto& leg : old_control_log_->legs_R) {
      if (leg.stance != 1.0) { return false; }
    }
    return true;
  }

  boost::asio::executor executor_;
  Parameters parameters_;

  base::LogRef log_ = base::GetLogInstance("QuadrupedControl");

  Config config_;
  std::deque<Leg> legs_;

  QuadrupedControl::Status status_;
  QC current_command_;
  boost::posix_time::ptime current_command_timestamp_;
  ReportedServoConfig reported_servo_config_;

  std::array<ControlLog, 2> control_logs_;
  ControlLog* control_log_ = &control_logs_[0];
  ControlLog* old_control_log_ = &control_logs_[1];

  mjlib::io::RepeatingTimer timer_;
  using Client = MultiplexClient::Client;

  ClientGetter client_getter_;
  ImuGetter imu_getter_;

  Client* client_ = nullptr;
  ImuClient* imu_client_ = nullptr;

  using Request = std::vector<Client::IdRequest>;
  Request status_request_;
  Request config_status_request_;
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

  int outstanding_status_requests_ = 0;
  AttitudeData imu_data_;

  boost::signals2::signal<void (const Status*)> status_signal_;
  boost::signals2::signal<void (const CommandLog*)> command_signal_;
  boost::signals2::signal<void (const ControlLog*)> control_signal_;
  boost::signals2::signal<void (const AttitudeData*)> imu_signal_;
  boost::signals2::signal<
    void (const ReportedServoConfig*)> servo_config_signal_;

  std::vector<moteus::Value> values_cache_;

  std::vector<int> all_leg_ids_{0, 1, 2, 3};
};

QuadrupedControl::QuadrupedControl(base::Context& context,
                                   ClientGetter client_getter,
                                   ImuGetter imu_getter)
    : impl_(std::make_unique<Impl>(context, client_getter, imu_getter)) {}

QuadrupedControl::~QuadrupedControl() {}

void QuadrupedControl::AsyncStart(mjlib::io::ErrorCallback callback) {
  impl_->AsyncStart(std::move(callback));
}

void QuadrupedControl::Command(const QC& command) {
  impl_->Command(command);
}

const QuadrupedControl::Status& QuadrupedControl::status() const {
  return impl_->status_;
}

clipp::group QuadrupedControl::program_options() {
  return mjlib::base::ClippArchive().Accept(&impl_->parameters_).release();
}

}
}
