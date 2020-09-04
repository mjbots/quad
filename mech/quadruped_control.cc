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

#include <boost/algorithm/string.hpp>
#include <boost/asio/post.hpp>

#include <fmt/format.h>

#include "mjlib/base/clipp_archive.h"
#include "mjlib/base/json5_read_archive.h"

#include "mjlib/io/now.h"
#include "mjlib/io/repeating_timer.h"

#include "base/common.h"
#include "base/fit_plane.h"
#include "base/interpolate.h"
#include "base/logging.h"
#include "base/sophus.h"
#include "base/telemetry_registry.h"
#include "base/timestamped_log.h"

#include "mech/attitude_data.h"
#include "mech/mammal_ik.h"
#include "mech/moteus.h"
#include "mech/quadruped_config.h"
#include "mech/quadruped_context.h"
#include "mech/quadruped_trot.h"
#include "mech/quadruped_util.h"
#include "mech/swing_trajectory.h"
#include "mech/trajectory.h"

namespace pl = std::placeholders;

namespace mjmech {
namespace mech {

namespace {
constexpr int kNumServos = 12;

using QC = QuadrupedCommand;
using QM = QC::Mode;

template <typename Iter, typename Functor>
auto Average(Iter begin, Iter end, Functor f) -> decltype(f(*begin)) {
  using T = decltype(f(*begin));

  T sum = {};
  double count = {};
  for (Iter it = begin; it != end; ++it) {
    count++;
    sum += f(*it);
  }
  return (1.0 / count) * sum;
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
}

class QuadrupedControl::Impl {
 public:
  Impl(base::Context& context,
       Pi3hatGetter pi3hat_getter)
      : executor_(context.executor),
        telemetry_log_(context.telemetry_log.get()),
        timer_(executor_),
        pi3hat_getter_(pi3hat_getter) {
    context.telemetry_registry->Register("qc_status", &status_signal_);
    context.telemetry_registry->Register("qc_command", &command_signal_);
    context.telemetry_registry->Register("qc_control", &control_signal_);
    context.telemetry_registry->Register("imu", &imu_signal_);
    context.telemetry_registry->Register("servo_config", &servo_config_signal_);
  }

  void AsyncStart(mjlib::io::ErrorCallback callback) {
    pi3hat_ = pi3hat_getter_();

    BOOST_ASSERT(!!pi3hat_);

    // Load our configuration.
    std::vector<std::string> configs;
    boost::split(configs, parameters_.config, boost::is_any_of(" "));
    for (const auto& config : configs) {
      std::ifstream inf(config);
      mjlib::base::system_error::throw_if(
          !inf.is_open(),
          fmt::format("could not open config file '{}'", parameters_.config));

      mjlib::base::Json5ReadArchive(inf).Accept(&config_);
    }

    if (config_.legs.size() != 4 ||
        config_.joints.size() != kNumServos) {
      mjlib::base::Fail(
          fmt::format(
              "Incorrect number of legs/joints configured: {}/{} != 4/12",
              config_.legs.size(), config_.joints.size()));
    }

    context_.emplace(config_, &current_command_, &status_.state);

    PopulateStatusRequest();

    period_s_ = config_.period_s;
    timer_.start(mjlib::base::ConvertSecondsToDuration(period_s_),
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

    // Update our logging status.
    if (command.log != QuadrupedCommand::Log::kUnset) {
      if (command.log == QuadrupedCommand::Log::kEnable &&
          !telemetry_log_->IsOpen()) {
        base::OpenMaybeTimestampedLog(
            telemetry_log_,
            parameters_.log_filename_base,
            base::kTimestamped);
      } else if (command.log == QuadrupedCommand::Log::kDisable &&
                 telemetry_log_->IsOpen()) {
        telemetry_log_->Close();
      }
    }

    command_signal_(&command_log);
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

    if (!pi3hat_) { return; }
    if (outstanding_) { return; }

    timing_ = ControlTiming(executor_, timing_.cycle_start());

    if (timing_.status().delta_s > 1.5 * period_s_) {
      // We likely skipped a cycle.  Warn.
      WarnRateLimited(fmt::format("Skipped cycle: delta_s={}",
                                  timing_.status().delta_s));
    }

    outstanding_ = true;

    status_reply_ = {};

    // Ask for the IMU and the servo data simultaneously.
    outstanding_status_requests_ = 0;

    auto* request = [&]() {
      if (status_.mode == QM::kConfiguring) {
        return &config_status_request_;
      }
      return &status_request_;
    }();
    pi3hat_->Cycle(&imu_data_, request, &status_reply_,
                   std::bind(&Impl::HandleStatus, this, pl::_1));
  }

  void HandleStatus(const mjlib::base::error_code& ec) {
    mjlib::base::FailIf(ec);

    timing_.finish_query();

    imu_signal_(&imu_data_);

    // If we don't have all 12 servos, then skip this cycle.
    const uint16_t servo_bitmask = [&]() {
      uint16_t result = 0;
      for (const auto& item : status_reply_) {
        result |= (1 << item.id);
      }
      return result;
    }();
    const int found_servos = [&]() {
      int result = 0;
      for (int i = 1; i <= kNumServos; i++) {
        if (servo_bitmask & (1 << i)) {
          result++;
        }
      }
      return result;
    }();
    status_.missing_replies = kNumServos - found_servos;

    if (found_servos != kNumServos) {
      if (status_.state.joints.size() != kNumServos) {
        // We have to get at least one full set before we can start
        // updating.
        outstanding_ = false;
        return;
      }
    }

    // Fill in the status structure.
    if (!UpdateStatus()) {
      // Guess we didn't have enough to actually do anything.
      outstanding_ = false;
      return;
    }

    timing_.finish_status();

    // Now run our control loop and generate our command.
    std::swap(control_log_, old_control_log_);
    *control_log_ = {};
    RunControl();

    timing_.finish_control();

    if (!client_command_.empty()) {
      client_command_reply_.clear();
      pi3hat_->AsyncTransmit(
          &client_command_, &client_command_reply_,
          std::bind(&Impl::HandleCommand, this, pl::_1));
    } else {
      HandleCommand({});
    }
  }

  void HandleCommand(const mjlib::base::error_code& ec) {
    mjlib::base::FailIf(ec);
    outstanding_ = false;

    timing_.finish_command();
    status_.timestamp = Now();
    status_.timing = timing_.status();

    status_signal_(&status_);
  }

  std::optional<double> MaybeGetSign(int id) const {
    for (const auto& joint : config_.joints) {
      if (joint.id == id) { return joint.sign; }
    }
    return {};
  }

  bool UpdateStatus() {
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

    for (const auto& reply : status_reply_) {
      const auto maybe_sign = MaybeGetSign(reply.id);
      if (!maybe_sign) {
        log_.warn(fmt::format("Reply from unknown servo {}", reply.id));
        return false;
      }

      QuadrupedState::Joint& out_joint = find_or_make_joint(reply.id);

      out_joint.id = reply.id;

      const double sign = *maybe_sign;

      const auto* maybe_value = std::get_if<moteus::Value>(&reply.value);
      if (!maybe_value) { continue; }
      const auto& value = *maybe_value;
      switch (static_cast<moteus::Register>(reply.reg)) {
        case moteus::kMode: {
          out_joint.mode = moteus::ReadInt(value);
          break;
        }
        case moteus::kPosition: {
          out_joint.angle_deg = sign * moteus::ReadPosition(value);
          break;
        }
        case moteus::kVelocity: {
          out_joint.velocity_dps = sign * moteus::ReadVelocity(value);
          break;
        }
        case moteus::kTorque: {
          out_joint.torque_Nm = sign * moteus::ReadTorque(value);
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

    // We should only be here if we have something for all our joints.
    if (status_.state.joints.size() != kNumServos) {
      return false;
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

    auto find_or_make_leg = [&](int id) -> QuadrupedState::Leg& {
      for (auto& leg : status_.state.legs_B) {
        if (leg.leg == id) { return leg; }
      }

      status_.state.legs_B.push_back({});
      auto& result = status_.state.legs_B.back();
      result.leg = id;
      return result;
    };

    for (const auto& leg : context_->legs) {
      QuadrupedState::Leg& out_leg_B = find_or_make_leg(leg.leg);
      const auto effector_G = leg.ik.Forward_G(joint_angles);
      const auto effector_B = leg.pose_BG * effector_G;

      out_leg_B.leg = leg.leg;
      out_leg_B.position = effector_B.pose;
      out_leg_B.velocity = effector_B.velocity;
      out_leg_B.force_N = effector_B.force_N;

      const auto* control_B = [&]() -> const QC::Leg* {
        for (const auto& item_B : old_control_log_->legs_B) {
          if (item_B.leg_id == leg.leg) { return &item_B; }
        }
        return nullptr;
      }();

      if (control_B) {
        out_leg_B.stance = control_B->stance;
      }
    }

    std::sort(status_.state.legs_B.begin(), status_.state.legs_B.end(),
              [](const auto& lhs, const auto& rhs) {
                return lhs.leg < rhs.leg;
              });

    // Now update the robot values.

    // frame_RB isn't sensed, but is just a commanded value.

    // Do the A frame (Attitude)
    auto& frame_AB = status_.state.robot.frame_AB;
    Sophus::SE3d CB{
      Sophus::SO3d(),
          -config_.center_of_mass_B};
    Sophus::SE3d AC{
      Sophus::SO3d(imu_data_.attitude.eigen()),
          Eigen::Vector3d(),
    };

    frame_AB.pose = AC * CB;
    frame_AB.w = (M_PI / 180.0) * imu_data_.rate_dps;

    // Now the M frame (CoM)
    auto& frame_MB = status_.state.robot.frame_MB;
    auto axis_MC = imu_data_.attitude.euler_rad();
    axis_MC.yaw = 0.0;
    Sophus::SE3d MC{
      Sophus::SO3d(base::Quaternion::FromEuler(axis_MC).eigen()),
          Eigen::Vector3d(),
    };
    frame_MB.pose = MC * CB;

    // Do terrain.
    UpdateTerrain();

    {
      const double min_voltage =
          Min(status_.state.joints.begin(), status_.state.joints.end(),
              [](const auto& joint) { return joint.voltage; });
      auto& out_voltage = status_.state.robot.voltage;

      if (out_voltage == 0.0) {
        out_voltage = min_voltage;
      } else {
        const double alpha =
            std::pow(0.5, config_.period_s / config_.voltage_filter_s);
        out_voltage = alpha * out_voltage + (1.0 - alpha) * min_voltage;
      }

      if (out_voltage < config_.min_voltage) {
        Fault(fmt::format(
                  "Battery low: {} < {}", out_voltage, config_.min_voltage));
      }
    }

    return true;
  }

  void UpdateTerrain() {
    const auto& tf_AB = status_.state.robot.frame_AB.pose;
    auto& tf_TA = status_.state.robot.tf_TA;

    std::vector<base::Point3D> stance_A;
    for (const auto& leg_B : status_.state.legs_B) {
      Eigen::Vector3d p_A = tf_AB * leg_B.position;
      // If we are not in full stance, or if we are not pressing
      // against the ground, then use our estimate.
      if (leg_B.stance != 1.0 ||
          (leg_B.force_N.z() <
           ((1.0 / 8.0) * base::kGravity * config_.mass_kg))) {
        // Use the Z value from the current terrain.
        Eigen::Vector3d p_T = tf_TA * p_A;
        p_T.z() = 0;
        stance_A.push_back(tf_TA.inverse() * p_T);
      } else {
        stance_A.push_back(p_A);
      }
    }

    auto& robot = status_.state.robot;

    // Now update the attitude recursively.
    std::vector<base::Point3D> stance_T;
    for (const auto& leg_A : stance_A) {
      stance_T.push_back(tf_TA * leg_A);
    }

    // Fit a plane to these four points to see how to update our
    // terrain transform.
    const auto plane = base::FitPlane(stance_A);

    // We just always exactly set our translation.
    robot.tf_TA.translation().z() = -plane.c;

    // We filter our X and Y slopes.
    const double alpha = (
        std::pow(0.5, config_.period_s / config_.terrain_filter_s));
    robot.terrain_rad[0] = (
        alpha * robot.terrain_rad[0] + (1.0 - alpha) * std::atan(plane.a));
    robot.terrain_rad[1] = (
        alpha * robot.terrain_rad[1] + (1.0 - alpha) * std::atan(plane.b));

    robot.tf_TA.so3() = Sophus::SO3d(
        (base::Quaternion::FromAxisAngle(
            robot.terrain_rad[0], 0, 1, 0) *
         base::Quaternion::FromAxisAngle(
             -robot.terrain_rad[1], 1, 0, 0)).eigen());
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

    for (const auto& reply : status_reply_) {
      auto& out_servo = find_or_make_servo(reply.id);

      const auto* maybe_value = std::get_if<moteus::Value>(&reply.value);
      if (!maybe_value) { continue; }
      const auto& value = *maybe_value;

      switch (static_cast<moteus::Register>(reply.reg)) {
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
      case QM::kBackflip: {
        DoControl_Backflip();
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
      case QM::kWalk:
      case QM::kBackflip: {
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
          // Enter rest until we've stopped moving.
          if (IsRestAndSteadyState()) {
            status_.mode = current_command_.mode;
          } else {
            status_.mode = QM::kRest;
          }
        } else if (status_.mode == QM::kWalk) {
          // We can only leave the walk state when our desired
          // velocities are all 0 and all four legs are on the ground.
          if (status_.state.walk.idle_count >= 2 &&
              all_legs_stance()) {
            status_.state.jump.command = current_command_.jump.value_or(
                QuadrupedCommand::Jump());
            status_.mode = current_command_.mode;
          } else {
            current_command_.v_R = {};
            current_command_.w_R = {};
          }
        } else if (status_.mode == QM::kBackflip &&
                   status_.state.backflip.mode ==
                   QuadrupedState::Backflip::Mode::kDone) {
          status_.mode = current_command_.mode;
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
        case QM::kBackflip: {
          status_.state.backflip = {};
          break;
        }
        case QM::kRest: {
          status_.state.rest = {};
          break;
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
    if (reported_servo_config_.servos.size() != kNumServos) {
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

    // If we are the one who did the rezeroing, then they also must be
    // close enough to the desired position.
    if (status_.performed_rezero) {
      for (const auto& joint : status_.state.joints) {
        const auto* config = [&]() -> const QuadrupedConfig::Joint* {
          for (const auto& cj : config_.joints) {
            if (cj.id == joint.id) { return &cj; }
          }
          return nullptr;
        }();
        if (!config) {
          status_.fault = "Unknown joint";
          return false;
        }
        if ((std::abs(
                 base::WrapNeg180To180(
                     (config->sign * joint.angle_deg) -
                     config->rezero_pos_deg))) >
            config_.rezero_threshold_deg) {
          status_.fault = fmt::format("Legs not in turn-on position id {}={}",
                                      joint.id, joint.angle_deg);
          return false;
        }
      }
    }

    status_.fault = "";
    return true;
  }

  void DoControl_Configuring() {
    // If we are configuring, and have received a status that *all* of
    // our servos are not zeroed, then we skip a control cycle and
    // instead rezero them.
    const int need_rezero_count = [&]() {
      int total = 0;
      for (const auto& servo : reported_servo_config_.servos) {
        if (servo.rezero_state == 0) { total++; }
      }
      return total;
    }();

    if (need_rezero_count == kNumServos) {
      // Instead of sending a normal command, we will tell our servos
      // to rezero.
      EmitRezero();
    } else {
      EmitStop();
    }
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
    status_.state.robot.frame_RB = {};
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
          status_.state.stand_up.mode = M::kStanding;
        }
        break;
      }
      case M::kStanding: {
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

  bool CheckPrepositioning() const {
    // We're done when all our joints are close enough.
    const std::map<int, double> current_deg = [&]() {
      std::map<int, double> result;
      for (const auto& joint : status_.state.joints) {
        result[joint.id] = joint.angle_deg;
      }
      return result;
    }();

    for (const auto& leg : context_->legs) {
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
    for (const auto& leg : context_->legs) {
      QC::Joint joint;
      joint.power = true;
      joint.angle_deg = std::numeric_limits<double>::quiet_NaN();
      joint.velocity_dps = config_.stand_up.velocity_dps;
      joint.max_torque_Nm = config_.stand_up.max_preposition_torque_Nm;
      joint.kp_scale = config_.stand_up.preposition_kp_scale;

      auto add_joint = [&](int id, double angle_deg) {
        joint.id = id;
        joint.stop_angle_deg = angle_deg;
        joints.push_back(joint);
      };

      auto add_stopped_joint = [&](int id) {
        joint.id = id;
        joint.velocity_dps = 0.0;
        joint.stop_angle_deg = {};
        joints.push_back(joint);
      };

      switch (status_.state.stand_up.prepositioning_stage) {
        case 0: {
          // First just lift the shoulders while leaving the other
          // joints unchanged.
          add_joint(leg.config.ik.shoulder.id,
                    leg.shoulder_clearance_deg);
          add_stopped_joint(leg.config.ik.femur.id);
          add_stopped_joint(leg.config.ik.tibia.id);
          break;
        }
        case 1: {
          // Then move the other joints into position.
          add_joint(leg.config.ik.shoulder.id,
                    leg.shoulder_clearance_deg);
          add_joint(leg.config.ik.femur.id,
                    leg.resolved_stand_up_joints.femur_deg);
          add_joint(leg.config.ik.tibia.id,
                    leg.resolved_stand_up_joints.tibia_deg);
          break;
        }
        case 2: {
          // And finally put the shoulders back into place.
          add_joint(leg.config.ik.shoulder.id,
                    leg.resolved_stand_up_joints.shoulder_deg);
          add_joint(leg.config.ik.femur.id,
                    leg.resolved_stand_up_joints.femur_deg);
          add_joint(leg.config.ik.tibia.id,
                    leg.resolved_stand_up_joints.tibia_deg);
          break;
        }
      }
    }

    // See if we can advance to the next prepositioning stage.
    bool all_done = true;
    for (const auto& joint : joints) {
      const auto& joint_state = context_->GetJointState(joint.id);
      if (!!joint.stop_angle_deg &&
          std::abs(joint.stop_angle_deg.value_or(0.0) - joint_state.angle_deg) >
          config_.stand_up.tolerance_deg) {
        all_done = false;
      }
    }
    if (all_done) {
      status_.state.stand_up.prepositioning_stage =
          std::min(status_.state.stand_up.prepositioning_stage + 1, 2);
    }

    ControlJoints(joints);
  }

  void DoControl_StandUp_Standing() {
    std::vector<QC::Leg> legs_R = old_control_log_->legs_R;

    if (legs_R.empty()) {
      for (const auto& leg : context_->legs) {
        QC::Leg leg_cmd_R;
        leg_cmd_R.kp_N_m = config_.default_kp_N_m;
        leg_cmd_R.kd_N_m_s = config_.default_kd_N_m_s;
        leg_cmd_R.leg_id = leg.leg;
        leg_cmd_R.power = true;
        leg_cmd_R.position = leg.stand_up_R;
        leg_cmd_R.velocity = base::Point3D();
        leg_cmd_R.stance = 1.0;
        legs_R.push_back(leg_cmd_R);
      }
    }

    QuadrupedContext::MoveOptions move_options;
    move_options.override_acceleration =
        config_.stand_up.acceleration;

    const bool done = context_->MoveLegsFixedSpeed(
        &legs_R, config_.stand_up.velocity, [&]() {
          std::vector<std::pair<int, base::Point3D>> result;
          for (const auto& leg : context_->legs) {
            base::Point3D pose = leg.stand_up_R;
            pose.z() = config_.stand_height;
            result.push_back(std::make_pair(leg.leg, pose));
          }
          return result;
        }(),
        move_options);

    if (done) {
      status_.state.stand_up.mode = QuadrupedState::StandUp::Mode::kDone;
    }

    ControlLegs_R(std::move(legs_R), context_->LevelDesiredRB());
  }

  bool IsRestAndSteadyState() const {
    if (status_.mode != QM::kRest) { return false; }
    if (control_log_->legs_R.empty()) { return false; }

    for (const auto& leg_R : control_log_->legs_R) {
      if (leg_R.velocity != base::Point3D()) { return false; }
    }

    return true;
  }

  void DoControl_Rest() {
    ClearDesiredMotion();

    std::vector<QC::Leg> legs_R;

    MJ_ASSERT(!old_control_log_->legs_R.empty());

    legs_R = old_control_log_->legs_R;

    // Ensure all gains are back to their default and that
    // everything is marked as in stance.
    for (auto& leg_R : legs_R) {
      leg_R.kp_N_m = config_.default_kp_N_m;
      leg_R.kd_N_m_s = config_.default_kd_N_m_s;
      leg_R.kp_scale = {};
      leg_R.kd_scale = {};
      leg_R.stance = 1.0;
      leg_R.landing = false;
      // We don't want to be moving laterally when in rest, just up
      // and down.
      leg_R.velocity.head<2>() = Eigen::Vector2d(0., 0.);
    }
    QuadrupedContext::MoveOptions move_options;
    move_options.override_acceleration =
        config_.stand_up.acceleration;
    status_.state.rest.done = context_->MoveLegsFixedSpeedZ(
        all_leg_ids_,
        &legs_R,
        config_.rest.velocity,
        config_.stand_height,
        move_options);

    auto desired_RB = context_->LevelDesiredRB();
    desired_RB.pose.so3() =
        current_command_.rest.offset_RB.so3() * desired_RB.pose.so3();
    desired_RB.pose.translation() +=
        current_command_.rest.offset_RB.translation();

    ControlLegs_R(std::move(legs_R), desired_RB);
  }

  std::vector<std::pair<int, base::Point3D>> MakeIdleLegs() const {
    std::vector<std::pair<int, base::Point3D>> result;
    for (const auto& leg : context_->legs) {
      result.push_back(std::make_pair(leg.leg, leg.idle_R));
    }
    return result;
  }

  void DoControl_Jump() {
    using JM = QuadrupedState::Jump::Mode;

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
      UpdateCommandedR();
    }

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

      // Default all our legs to the standard cartesian kp.
      for (auto& leg_R : legs_R) {
        leg_R.kp_N_m = config_.default_kp_N_m;
        leg_R.kd_N_m_s = config_.default_kd_N_m_s;

        // TODO: This is kind of a hack.  Ideally, we wouldn't have to
        // fiddle with the servo gains during a jump, and could just
        // use cartesian gains.  However, our current IK model seems
        // to have a large mismatch with reality for these jumps, and
        // we rely on the agressive servo controls to compensate.
        const auto kp = config_.jump.kp_scale;
        leg_R.kp_scale = base::Point3D(kp, kp, kp);
        const auto kd = config_.jump.kd_scale;
        leg_R.kd_scale = base::Point3D(kd, kd, kd);
      }

      switch (status_.state.jump.mode) {
        case JM::kLowering: {
          // Move legs to take into account R rates.
          context_->MoveLegsForR(&legs_R);

          // Lower all legs until they reach the lower_height.
          const bool done = context_->MoveLegsFixedSpeedZ(
              all_leg_ids_,
              &legs_R,
              config_.jump.lower_velocity,
              config_.jump.lower_height);
          if (done) {
            status_.state.jump.mode = JM::kPushing;
            status_.state.jump.velocity =
                -config_.jump.lower_velocity;
            status_.state.jump.acceleration =
                js.command.acceleration;
            // Loop around and do the pushing behavior.
            continue;
          }
          break;
        }
        case JM::kPushing: {
          context_->MoveLegsForR(&legs_R);

          auto& velocity = status_.state.jump.velocity;
          velocity += js.command.acceleration * period_s_;
          for (auto& leg_R : legs_R) {
            leg_R.position.z() += velocity * period_s_;
            leg_R.velocity.z() = velocity;
            leg_R.acceleration.z() = js.command.acceleration;
          }
          const bool done =
              legs_R[0].position.z() >= config_.jump.upper_height;
          if (done) {
            js.mode = JM::kRetracting;
            js.acceleration = 0.0;

            for (auto& leg_R : legs_R) {
              leg_R.stance = 0.0;
              // Latch the current measured position so our retract
              // maneuver is well formed.
              const auto& cur_leg_B = GetLegState_B(leg_R.leg_id);
              leg_R.position =
                  status_.state.robot.frame_RB.pose * cur_leg_B.position;
              leg_R.velocity =
                  status_.state.robot.frame_RB.pose * cur_leg_B.velocity;
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
          QuadrupedContext::MoveOptions move_options;
          move_options.override_acceleration =
              config_.jump.retract_acceleration;
          const bool done = context_->MoveLegsFixedSpeed(
              &legs_R,
              config_.jump.retract_velocity,
              MakeIdleLegs(),
              move_options);

          if (done) {
            js.mode = JM::kFalling;
            for (auto& leg_R : legs_R) {
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
          context_->MoveLegsForR(&legs_R);

          // Update our leg values that remain for this state.
          for (auto& leg_R : legs_R) {
            // We have different gains.
            const auto kp = config_.jump.land_kp;
            leg_R.kp_scale = base::Point3D(kp, kp, kp);
            const auto kd = config_.jump.land_kd;
            leg_R.kd_scale = base::Point3D(kd, kd, kd);

            // And no velocity or acceleration.
            leg_R.velocity = base::Point3D();
            leg_R.acceleration = base::Point3D();
          }

          // Wait for our legs to be pushed up by a certain distance,
          // which will indicate we have made contact with the ground.
          const double average_height = Average(
              status_.state.legs_B.begin(),
              status_.state.legs_B.end(),
              [](const auto& leg_B) {
                return leg_B.position.z();
              });
          const double max_height = Max(
              status_.state.legs_B.begin(),
              status_.state.legs_B.end(),
              [](const auto& leg_B) {
                return leg_B.position.z();
              });
          // We require the average to be below a threshold, and for
          // all legs to have made contact of some kind.
          const double average_error =
              average_height - config_.stand_height;
          const double max_error =
              max_height - config_.stand_height;
          if (average_error < -config_.jump.land_threshold &&
              max_error < -0.5 * config_.jump.land_threshold) {
            js.mode = JM::kLanding;
            js.acceleration = 0.0;
            js.velocity = std::numeric_limits<double>::quiet_NaN();
            // Loop around and start landing.
            continue;
          }
          break;
        }
        case JM::kLanding: {
          context_->MoveLegsForR(&legs_R);
          for (auto& leg_R : legs_R) {
            leg_R.stance = 1.0;
          }

          const bool done = SetLandingParameters(&legs_R);

          if (done) {
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
              js.acceleration = js.command.acceleration;
              continue;
            } else {
              js.mode = JM::kDone;
              break;
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
      ControlLegs_R(std::move(legs_R), context_->LevelDesiredRB());
      return;
    }
  }

  bool SetLandingParameters(std::vector<QC::Leg>* legs_R) {
    auto get_vel = [](const auto& leg_B) { return leg_B.velocity.z(); };
    auto& js = status_.state.jump;

    const double average_velocity = Average(
        status_.state.legs_B.begin(),
        status_.state.legs_B.end(),
        get_vel);

    // Pick an acceleration that will ensure we reach stopped
    // before hitting our lower height.
    const double average_height = Average(
        status_.state.legs_B.begin(),
        status_.state.legs_B.end(),
        [](const auto& leg_B) {
          return leg_B.position.z();
        });
    const double error =
        std::max(
            10.0,
            average_height - config_.jump.lower_height);

    constexpr double kFudge = 0.7;
    js.acceleration =
        std::max(
            config_.jump.min_acceleration,
            std::min(
                config_.bounds.max_acceleration,
                kFudge * 0.5 * std::pow(average_velocity, 2) / error));
    if (!std::isfinite(js.velocity)) {
      js.velocity = average_velocity;
    } else {
      // Blend in the ramped rate and the observed rate with an
      // exponential filter.
      const double propagated =
          js.velocity + config_.period_s * js.acceleration;
      const double filter_s = 0.1;
      const double alpha = std::pow(0.5, config_.period_s / filter_s);
      js.velocity =
          alpha * propagated + (1.0 - alpha) * average_velocity;
    }

    const double command_height =
        std::max(average_height, config_.jump.lower_height);
    const double limited_velocity =
        std::min(js.velocity, -config_.jump.lower_velocity);

    const double command_velocity =
        (average_height > config_.jump.lower_height) ?
        limited_velocity : 0.0;

    for (auto& leg_R : *legs_R) {
      const auto& pose_RB = status_.state.robot.frame_RB.pose;
      auto desired_B = pose_RB.inverse() * leg_R.position;
      desired_B.z() = command_height;

      leg_R.position = pose_RB * desired_B;
      leg_R.velocity.z() = command_velocity;
      leg_R.acceleration = base::Point3D(0, 0, js.acceleration);
      leg_R.stance = 1.0;
      leg_R.landing = false;
    }

    const double min_velocity = Min(
        status_.state.legs_B.begin(),
        status_.state.legs_B.end(),
        [](const auto& leg_B) {
          return leg_B.velocity.z();
        });

    // We are done if we are low enough, or if no leg is still moving
    // down (that can happen when part of the leg begins to rest on
    // the ground).
    const bool done = (
        (average_height < config_.jump.lower_height) ||
        (min_velocity > -config_.jump.land_velocity));

    return done;
  }

  void DoControl_Walk() {
    auto result = QuadrupedTrot(&*context_, old_control_log_->legs_R);
    ControlLegs_R(std::move(result.legs_R), result.desired_RB);
  }

  void DoControl_Backflip() {
    using BM = QuadrupedState::Backflip::Mode;

    std::optional<BM> previous_mode;
    auto& bs = status_.state.backflip;
    const double dt_s = period_s_;

    while (true) {
      auto legs_R = old_control_log_->legs_R;

      // We loop around until we stop changing state.
      if (!!previous_mode) {
        MJ_ASSERT(bs.mode != *previous_mode);
      }
      previous_mode = bs.mode;

      switch (bs.mode) {
        case BM::kLowering: {
          status_.state.robot.frame_RB = {};

          const bool done = context_->MoveLegsFixedSpeedZ(
              all_leg_ids_,
              &legs_R,
              config_.jump.lower_velocity,
              config_.backflip.lower_height);
          if (done) {
            bs.mode = BM::kFrontPush;
            // Loop around and do the pushing behavior.
            continue;
          }

          break;
        }
        case BM::kFrontPush: {
          // During the front push phase, we anchor the robot through
          // the axis of the rear servos, and apply an additional
          // downward force on the front legs to impart an
          // acceleration about the pitch axis.

          // Just do a zeroth order numerical integration.  It doesn't
          // need to be that accurate.
          bs.pitch_accel_dps2 = config_.backflip.pitch_accel_dps2;
          bs.pitch_rate_dps += bs.pitch_accel_dps2 * dt_s;
          bs.pitch_deg += bs.pitch_rate_dps * dt_s;

          for (auto& leg_R : legs_R) {
            leg_R.stance = 1.0;
          }

          BackflipUpdateLegs(&legs_R, 0.0);

          if (bs.pitch_deg > config_.backflip.max_pitch_deg) {
            bs.mode = BM::kBackPush;
            // We'll get this next time.
            break;
          }

          break;
        }
        case BM::kBackPush: {
          // In this mode, we just track the current pitch rate.
          //
          // TODO(jpieper): We could use the IMU here, but otherwise
          // we currently have no IMU dependencies on control
          // whatsoever.

          const bool push = bs.pitch_deg > config_.backflip.push_pitch_deg;
          const double acceleration =
              push ? config_.backflip.acceleration : 0.0;

          bs.pitch_accel_dps2 =
              push ? 0.0 : config_.backflip.pitch_accel_dps2;
          bs.pitch_rate_dps += bs.pitch_accel_dps2 * dt_s;
          bs.pitch_deg += bs.pitch_rate_dps * dt_s;
          bs.velocity += acceleration * period_s_;

          for (auto& leg_R : legs_R) {
            const auto& config = [&]() -> const Config::Leg& {
              for (auto& v : context_->legs) {
                if (v.leg == leg_R.leg_id) { return v.config; }
              }
              mjlib::base::AssertNotReached();
            }();
            leg_R.stance = config.pose_BG.translation().x() < 0.0 ? 1.0 : 0.0;
          }

          BackflipUpdateLegs(&legs_R, acceleration);

          // We switch to flight when our legs have extended to our
          // max jump height.
          const double leg_z = [&]() {
            for (auto& leg_R : legs_R) {
              if (leg_R.stance == 1.0) { return leg_R.position.z(); }
            }
            mjlib::base::AssertNotReached();
          }();

          if (leg_z > config_.backflip.push_height) {
            // Reset our RB transform.
            auto& pose_RB = status_.state.robot.frame_RB.pose;

            for (auto& leg_R : legs_R) {
              const auto& leg_B = [&]() {
                for (const auto& v : old_control_log_->legs_B) {
                  if (v.leg_id == leg_R.leg_id) { return v; }
                }
                mjlib::base::AssertNotReached();
              }();
              leg_R.position = leg_B.position;
              leg_R.velocity = leg_B.velocity;
              leg_R.force_N = Eigen::Vector3d();
              const auto kp = config_.backflip.flight_kp;
              leg_R.kp_scale = base::Point3D(kp, kp, kp);
              leg_R.stance = 0.0;
            }

            pose_RB = Sophus::SE3d();

            bs.mode = BM::kFlight;
            // Execute at least one iteration here so that our new R
            // frame coordinates take effect.
            break;
          }

          break;
        }
        case BM::kFlight: {
          // In this mode, we get our legs gently back into the idle
          // position and set our gains to be ready for landing.
          context_->MoveLegsFixedSpeed(
              &legs_R, config_.backflip.flight_velocity,
              MakeIdleLegs());

          break;
        }
        case BM::kDone: {
          DoControl_Rest();
          return;
        }
      }

      // If we make it here, then we don't need to repeat.
      ControlLegs_R(std::move(legs_R), context_->LevelDesiredRB());
      return;
    }
  }

  void BackflipUpdateLegs(std::vector<QC::Leg>* legs_R,
                          double acceleration) {
    auto& bs = status_.state.backflip;

    const Eigen::Vector3d alpha_RB(
        0, base::Radians(-bs.pitch_accel_dps2), 0);
    const Eigen::Vector3d omega_RB(
        0, base::Radians(-bs.pitch_rate_dps), 0);

    // We'll denote the 'rleg' frame as centered between the
    // axis of rotation of the rear femurs.

    const double x = Min(
        context_->legs.begin(), context_->legs.end(), [](auto& leg) {
          return leg.pose_B_femur.x();
        });

    const Sophus::SE3d pose_rleg_B =
        Sophus::SE3d(Sophus::SO3d(), Eigen::Vector3d(-x, 0, 0));

    // And we'll denote the 'rlegp' frame as the one centered
    // between the axis of rotation and pitched up by our
    // current pitch amount.
    const Sophus::SE3d pose_rlegp_rleg =
        Sophus::SE3d(
            Eigen::AngleAxisd(base::Radians(bs.pitch_deg),
                              Eigen::Vector3d::UnitY()).toRotationMatrix(),
            Eigen::Vector3d());

    const Sophus::SE3d pose_rlegp_B =
        pose_rlegp_rleg * pose_rleg_B;

    auto& pose_RB = status_.state.robot.frame_RB.pose;
    const Sophus::SE3d old_pose_RB = pose_RB;
    pose_RB = pose_rleg_B.inverse() * pose_rlegp_B;
    const Sophus::SE3d delta_RB = old_pose_RB.inverse() * pose_RB;

    const double stance_legs = [&]() {
      double result = 0.0;
      for (const auto& leg_R : *legs_R) {
        result += leg_R.stance;
      }
      return result;
    }();

    for (auto& leg_R : *legs_R) {
      // Find the position in the 'rleg' frame.
      const Eigen::Vector3d leg_pose_B =
          pose_RB.inverse() * leg_R.position;
     const Eigen::Vector3d leg_pose_rleg =
          pose_rlegp_B * leg_pose_B;

      if (leg_R.stance == 0.0) {
        // We set the velocity and force to 0.  Ideally we would do so
        // in the body frame, but we don't currently have the ability
        // to set velocities in the body frame.
        leg_R.position = delta_RB * leg_R.position;
        leg_R.velocity = -omega_RB.cross(leg_pose_rleg);
        leg_R.force_N = Eigen::Vector3d();
      } else {
        // TODO(jpieper): These calculations would be unecessary
        // if we had the ability to pass the RB angular velocity
        // and acceleration into the R frame controller.  For now,
        // we just lie about the R frame velocity, since we know
        // it will be passed through.
        leg_R.position.z() += bs.velocity * period_s_;
        leg_R.velocity = omega_RB.cross(leg_pose_rleg) +
            Eigen::Vector3d(0, 0, bs.velocity);
        leg_R.force_N =
            (leg_R.stance * config_.mass_kg / stance_legs) *
            alpha_RB.cross(leg_pose_rleg) +
            (leg_R.stance / stance_legs) *
            base::Point3D(
                0, 0, config_.mass_kg * (
                    base::kGravity + acceleration));
      }
    }

    // TODO: have an RB rate of change so that we can properly
    // command velocities of the joints.
  }

  void ClearDesiredMotion() {
    status_.state.robot.desired_R = {};
  }

  void UpdateCommandedR() {
    context_->UpdateCommandedR();
  }

  void ControlLegs_R(std::vector<QC::Leg> legs_R,
                     const base::KinematicRelation& desired_RB) {
    control_log_->desired_RB = desired_RB;
    control_log_->legs_R = std::move(legs_R);
    std::sort(control_log_->legs_R.begin(),
              control_log_->legs_R.end(),
              [](const auto& lhs, const auto& rhs) {
                return lhs.leg_id < rhs.leg_id;
              });

    // Apply our desired RB frame with an exponential smoothing
    // filter.
    {
      auto& frame_RB = status_.state.robot.frame_RB;
      // Filter the RB desire.
      const base::Point3D delta =
          desired_RB.pose.translation() - frame_RB.pose.translation();
      frame_RB.pose.translation() +=
          config_.rb_filter_constant_Hz * config_.period_s * delta;
      frame_RB.pose.so3() =
          Sophus::SO3d(
              frame_RB.pose.so3().unit_quaternion().slerp(
                  config_.rb_filter_constant_Hz * config_.period_s,
                  desired_RB.pose.so3().unit_quaternion()));
      frame_RB.v = desired_RB.v;
      frame_RB.w = desired_RB.w;
    }

    const Sophus::SE3d pose_BR = status_.state.robot.frame_RB.pose.inverse();

    std::vector<QC::Leg> legs_B;
    for (const auto& leg_R : control_log_->legs_R) {
      legs_B.push_back(pose_BR * leg_R);
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
      leg_B.position.z() = std::max(
          config_.bounds.min_z_B,
          std::min(config_.bounds.max_z_B, leg_B.position.z()));
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

    if (control_log_->leg_pds.size() < control_log_->legs_B.size()) {
      control_log_->leg_pds.resize(control_log_->legs_B.size());
    }

    const double total_stance = [&]() {
      double result = 0.0;
      for (const auto& leg_B : control_log_->legs_B) {
        result += leg_B.stance;
      }
      return std::max(1.0, result);
    }();

    const base::Point3D g_M = base::Point3D(0., 0., 1.);
    const base::Point3D g_B = status_.state.robot.frame_MB.pose.inverse() * g_M;

    for (const auto& leg_B : control_log_->legs_B) {
      const auto& qleg = GetLeg(leg_B.leg_id);
      const auto& leg_state_B = GetLegState_B(leg_B.leg_id);
      auto& leg_pd = control_log_->leg_pds[leg_B.leg_id];

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
        const Sophus::SE3d pose_GB = qleg.pose_BG.inverse();

        IkSolver::Effector effector_B;

        effector_B.pose = leg_B.position;
        effector_B.velocity = leg_B.velocity;

        const double stance_fraction = leg_B.stance / total_stance;

        // Do the cartesian PD control.
        leg_pd.cmd_N = leg_B.force_N;
        leg_pd.gravity_N =
            stance_fraction * base::kGravity * config_.mass_kg * g_B;

        leg_pd.accel_N =
            (leg_B.acceleration) *
            base::Interpolate(
                config_.leg_mass_kg,
                stance_fraction * config_.mass_kg,
                leg_B.stance);
        leg_pd.err_m = leg_state_B.position - leg_B.position;
        leg_pd.p_N = -1 * (leg_pd.err_m.array() *
                           leg_B.kp_N_m.array()).matrix();
        leg_pd.err_m_s = leg_state_B.velocity - leg_B.velocity;
        leg_pd.d_N = -1 * (leg_pd.err_m_s.array() *
                           leg_B.kd_N_m_s.array()).matrix();
        leg_pd.total_N =
            leg_pd.cmd_N + leg_pd.gravity_N +
            leg_pd.accel_N + leg_pd.p_N + leg_pd.d_N;

        effector_B.force_N = leg_pd.total_N;

        const auto effector_G = pose_GB * effector_B;

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

            auto get_pd_scale = [&](const auto& value) {
              if (joint_angle.id == qleg.config.ik.shoulder.id) {
                return value.x();
              }
              if (joint_angle.id == qleg.config.ik.femur.id) {
                return value.y();
              }
              if (joint_angle.id == qleg.config.ik.tibia.id) {
                return value.z();
              }
              return value.x();
            };
            out_joint.kp_scale =
                leg_B.kp_scale ? get_pd_scale(*leg_B.kp_scale) :
                std::optional<double>();
            out_joint.kd_scale =
                leg_B.kd_scale ? get_pd_scale(*leg_B.kd_scale) :
                std::optional<double>();
            out_joints.push_back(out_joint);
          }
        }
      }
    }

    ControlJoints(std::move(out_joints));
  }

  void ControlJoints(std::vector<QC::Joint> joints) {
    control_log_->joints = std::move(joints);
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

    size_t pos = 0;
    for (const auto& joint : control_log_->joints) {
      if (client_command_.size() <= pos) {
        client_command_.resize(client_command_.size() + 1);
      }

      auto& request = client_command_[pos++];
      request.request.clear();
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
        const auto maybe_sign = MaybeGetSign(joint.id);
        if (!maybe_sign) {
          log_.warn(fmt::format("Unknown servo {}", joint.id));
          continue;
        }
        const double sign = *maybe_sign;

        if (joint.angle_deg != 0.0) { values.resize(1); }
        if (joint.velocity_dps != 0.0) { values.resize(2); }
        if (joint.torque_Nm != 0.0) { values.resize(3); }
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
            case 3:
            case 4: {
              values[i] = moteus::WritePwm(1.0, moteus::kInt16);
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

        // We do kp and kd separately so we can use the float type.
        values.clear();
        if (joint.kp_scale) { values.resize(1); }
        if (joint.kd_scale) { values.resize(2); }
        for (size_t i = 0; i < values.size(); i++) {
          switch (i) {
            case 0: {
              double kp = joint.kp_scale.value_or(1.0);
              if (kp < 0.0) {
                kp = 0.0;
                log_.warn("negative joint kp!");
              }
              values[i] = moteus::WritePwm(kp, moteus::kFloat);
              break;
            }
            case 1: {
              double kd = joint.kd_scale.value_or(1.0);
              if (kd < 0.0) {
                kd = 0.0;
                log_.warn("negative joint kd!");
              }
              values[i] = moteus::WritePwm(kd, moteus::kFloat);
              break;
            }
          }
        }

        if (!values.empty()) {
          request.request.WriteMultiple(moteus::kCommandKpScale, values);
        }
      }
    }
    if (client_command_.size() > pos) {
      client_command_.resize(pos);
    }
  }

  void EmitRezero() {
    log_.warn("Emitting rezero to all servos");

    status_.performed_rezero = true;

    client_command_.resize(config_.joints.size());
    size_t pos = 0;
    for (const auto& joint : config_.joints) {
      auto& request = client_command_[pos++];
      request.request.clear();
      request.id = joint.id;

      request.request.WriteSingle(
          moteus::kRezero, static_cast<float>(joint.rezero_pos_deg / 360.0));
    }
  }

  boost::posix_time::ptime Now() {
    return mjlib::io::Now(executor_.context());
  }

  const QuadrupedContext::Leg& GetLeg(int id) const {
    return context_->GetLeg(id);
  }

  const QuadrupedState::Leg& GetLegState_B(int id) const {
    return context_->GetLegState_B(id);
  }

  bool all_legs_stance() const {
    for (const auto& leg : old_control_log_->legs_R) {
      if (leg.stance != 1.0) { return false; }
    }
    return true;
  }

  void WarnRateLimited(const std::string& message) {
    const boost::posix_time::time_duration kRateLimitTime =
        boost::posix_time::seconds(1);

    const auto now = Now();
    if (last_warn_timestamp_.is_not_a_date_time() ||
        (now - last_warn_timestamp_) > kRateLimitTime) {
      last_warn_timestamp_ = now;
      log_.warn(message);
    }
  }

  boost::asio::any_io_executor executor_;
  mjlib::telemetry::FileWriter* const telemetry_log_;
  Parameters parameters_;

  base::LogRef log_ = base::GetLogInstance("QuadrupedControl");

  Config config_;
  std::optional<QuadrupedContext> context_;

  QuadrupedControl::Status status_;
  QC current_command_;
  boost::posix_time::ptime current_command_timestamp_;
  ReportedServoConfig reported_servo_config_;

  std::array<ControlLog, 2> control_logs_;
  ControlLog* control_log_ = &control_logs_[0];
  ControlLog* old_control_log_ = &control_logs_[1];

  double period_s_ = 0.0;
  mjlib::io::RepeatingTimer timer_;
  using Client = mjlib::multiplex::AsioClient;

  Pi3hatGetter pi3hat_getter_;

  Pi3hatInterface* pi3hat_ = nullptr;

  using Request = Client::Request;
  Request status_request_;
  Request config_status_request_;
  Client::Reply status_reply_;

  Request client_command_;
  Client::Reply client_command_reply_;

  bool outstanding_ = false;
  ControlTiming timing_{executor_, {}};

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

  boost::posix_time::ptime last_warn_timestamp_;
};

QuadrupedControl::QuadrupedControl(base::Context& context,
                                   Pi3hatGetter pi3hat_getter)
    : impl_(std::make_unique<Impl>(context, pi3hat_getter)) {}

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
