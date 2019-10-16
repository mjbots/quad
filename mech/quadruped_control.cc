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

#include "base/logging.h"
#include "base/sophus.h"

#include "mech/mammal_ik.h"
#include "mech/moteus.h"

namespace pl = std::placeholders;

namespace mjmech {
namespace mech {

namespace {
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

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(joints));
    a->Visit(MJ_NVP(legs));
  }
};

struct Leg {
  int leg = 0;
  Config::Leg config;
  Sophus::SE3d pose_mm_BG;
  MammalIk ik;

  Leg(const Config::Leg& config_in)
      : leg(config_in.leg),
        config(config_in),
        pose_mm_BG(config_in.pose_mm_BG),
        ik(config_in.ik) {}
};

struct CommandLog {
  boost::posix_time::ptime timestamp;

  const QuadrupedCommand* command = nullptr;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(timestamp));
    const_cast<QuadrupedCommand*>(command)->Serialize(a);
  }
};
}

class QuadrupedControl::Impl {
 public:
  Impl(base::Context& context)
      : executor_(context.executor),
        timer_(executor_) {
    context.telemetry_registry->Register(
        "quadruped_control_status", &status_signal_);
    context.telemetry_registry->Register(
        "quadruped_control_command", &command_signal_);

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
        std::bind(callback, mjlib::base::error_code()));
  }

  void Command(const QuadrupedCommand& command) {
    CommandLog command_log;
    command_log.timestamp = Now();
    command_log.command = &command;

    current_command_ = command;

    command_signal_(&command_log);
  }

  void Configure() {
    for (const auto& leg : config_.legs) {
      legs_.emplace_back(leg);
    }
  }

  void PopulateStatusRequest() {
    status_request_ = {};
    for (const auto& joint : config_.joints) {
      status_request_.requests.push_back({});
      auto& current = status_request_.requests.back();
      current.id = joint.id;

      // Read mode, position, velocity, and torque.
      current.request.ReadMultiple(moteus::Register::kMode, 4, 1);
      current.request.ReadMultiple(moteus::Register::kVoltage, 3, 0);
    }
  }

  void HandleTimer(const mjlib::base::error_code& ec) {
    mjlib::base::FailIf(ec);

    if (!client_) { return; }
    if (outstanding_) { return; }

    timestamps_.cycle_start = Now();

    outstanding_ = true;

    status_reply_ = {};
    client_->AsyncRegister(&status_request_, &status_reply_,
                           std::bind(&Impl::HandleStatus, this, pl::_1));
  }

  void HandleStatus(const mjlib::base::error_code& ec) {
    mjlib::base::FailIf(ec);

    timestamps_.status_done = Now();

    // If we don't have all 12 servos, then skip this cycle.
    if (status_reply_.replies.size() != 12) {
      log_.warn(fmt::format("missing replies, sz={}",
                            status_reply_.replies.size()));
      outstanding_ = false;
      return;
    }

    // Fill in the status structure.
    UpdateStatus();

    // Now run our control loop and generate our command.
    client_command_ = {};
    RunControl();

    timestamps_.control_done = Now();

    if (!client_command_.requests.empty()) {
      client_command_reply_ = {};
      client_->AsyncRegister(&client_command_, &client_command_reply_,
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

    status_signal_(&status_);
  }

  void UpdateStatus() {
    IkSolver::JointAngles joint_angles;
    std::vector<QuadrupedState::Link> links;

    status_.state.joints.clear();

    for (const auto& reply : status_reply_.replies) {
      // This has raw values
      QuadrupedState::Joint out_joint;

      // And this has values which have had a sign correction applied.
      QuadrupedState::Link out_link;
      IkSolver::Joint ik_joint;

      out_joint.id = out_link.id = ik_joint.id = reply.id;

      const double sign = [&]() {
        for (const auto& joint : config_.joints) {
          if (joint.id == reply.id) { return joint.sign; }
        }
        mjlib::base::AssertNotReached();
      }();

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
            out_joint.angle_deg = moteus::ReadPosition(value);
            out_link.angle_deg = ik_joint.angle_deg =
                sign * out_joint.angle_deg;
            break;
          }
          case moteus::kVelocity: {
            out_joint.velocity_dps = moteus::ReadPosition(value);
            out_link.velocity_dps = ik_joint.velocity_dps =
                sign * out_joint.velocity_dps;
            break;
          }
          case moteus::kTorque: {
            out_joint.torque_Nm = moteus::ReadTorque(value);
            out_link.torque_Nm = ik_joint.torque_Nm =
                sign * out_joint.torque_Nm;
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

      status_.state.joints.push_back(out_joint);
      joint_angles.push_back(ik_joint);
      links.push_back(out_link);
    }

    status_.state.legs_B.clear();

    auto get_link = [&](int id) {
      for (const auto& link : links) {
        if (link.id == id) { return link; }
      }
      mjlib::base::AssertNotReached();
    };

    for (const auto& leg : legs_) {
      QuadrupedState::Leg out_leg_B;
      const auto effector = leg.ik.Forward(joint_angles);

      out_leg_B.leg = leg.leg;
      out_leg_B.position_mm = leg.pose_mm_BG * effector.pose_mm_G;
      out_leg_B.velocity_mm_s =
          leg.pose_mm_BG.so3() * effector.velocity_mm_s_G;
      out_leg_B.force_N =
          leg.pose_mm_BG.so3() * effector.force_N_G;

      out_leg_B.links.push_back(get_link(leg.config.ik.shoulder.id));
      out_leg_B.links.push_back(get_link(leg.config.ik.femur.id));
      out_leg_B.links.push_back(get_link(leg.config.ik.tibia.id));

      status_.state.legs_B.push_back(std::move(out_leg_B));
    }
  }

  void RunControl() {
  }

  boost::posix_time::ptime Now() {
    return mjlib::io::Now(executor_.context());
  }

  boost::asio::executor executor_;
  Parameters parameters_;
  boost::program_options::options_description options_;

  base::LogRef log_ = base::GetLogInstance("QuadrupedControl");

  Config config_;
  std::deque<Leg> legs_;

  QuadrupedControl::Status status_;
  QuadrupedCommand current_command_;

  mjlib::io::RepeatingTimer timer_;
  using Client = MultiplexClient::Client;

  Client* client_ = nullptr;

  Client::Request status_request_;
  Client::Reply status_reply_;

  Client::Request client_command_;
  Client::Reply client_command_reply_;

  bool outstanding_ = false;

  struct Timestamps {
    boost::posix_time::ptime cycle_start;
    boost::posix_time::ptime status_done;
    boost::posix_time::ptime control_done;
    boost::posix_time::ptime command_done;
  } timestamps_;

  boost::signals2::signal<void (const Status*)> status_signal_;
  boost::signals2::signal<void (const CommandLog*)> command_signal_;
};

QuadrupedControl::QuadrupedControl(base::Context& context)
    : impl_(std::make_unique<Impl>(context)) {}

QuadrupedControl::~QuadrupedControl() {}

void QuadrupedControl::AsyncStart(mjlib::io::ErrorCallback callback) {
  impl_->AsyncStart(callback);
}

void QuadrupedControl::SetClient(MultiplexClient::Client* client) {
  impl_->client_ = client;
}

void QuadrupedControl::Command(const QuadrupedCommand& command) {
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
