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

#include "mech/jump_test.h"

#include <iostream>
#include <sstream>

#include <boost/algorithm/string.hpp>
#include <boost/asio/executor.hpp>
#include <boost/asio/read_until.hpp>
#include <boost/asio/streambuf.hpp>
#include <boost/date_time/posix_time/posix_time_io.hpp>

#include <fmt/format.h>

#include "mjlib/base/fail.h"
#include "mjlib/base/program_options_archive.h"
#include "mjlib/base/time_conversions.h"
#include "mjlib/io/async_sequence.h"
#include "mjlib/io/deadline_timer.h"
#include "mjlib/io/now.h"

namespace mjmech {
namespace mech {

namespace {
namespace pl = std::placeholders;

enum State {
  // These states are used during the initial startup.

  kIdle,  // nothing is powered
  kPowered,  // powered, but not yet pre-positioned
  kPrepositioning,  // pre-positioning


  // The following are the states used in steady state jump(s).

  kStanding,  // standing up
  kReady,  // ready
  kSquatting,  // squatting to jump
  kJumping,  // pushing down
  kLandingPrepare,  // squatting legs up to land
  kLanding,  // in-flight and waiting for weight to be on legs
};

std::map<State, const char*> StateMapper() {
  return {
    { State::kIdle, "kIdle" },
    { State::kPowered, "kPowered" },
    { State::kPrepositioning, "kPrepositioning" },
    { State::kStanding, "kStanding" },
    { State::kReady, "kReady" },
    { State::kSquatting, "kSquatting" },
    { State::kJumping, "kJumping" },
    { State::kLandingPrepare, "kLandingPrepare" },
    { State::kLanding, "kLanding" },
    { State::kStanding, "kStanding" },
  };
}
}

class JumpTest::Impl {
 public:
  Impl(JumpTest* parent,
       base::Context& context)
      : parent_(parent),
        param_(&parent_->parameters_),
        executor_(context.executor),
        factory_(context.factory.get()),
        timer_(executor_) {
    stdin_options_.type = mjlib::io::StreamFactory::Type::kStdio;
  }

  void AsyncStart(mjlib::io::ErrorCallback callback) {
    if (param_->skip_powered) {
      state_ = State::kPrepositioning;
    }
    mjlib::io::AsyncSequence(executor_)
        .Add([this](auto callback) {
            std::cout << "starting children\n";
            param_->children.Start(callback);
          })
        .Add([this](auto callback) {
            std::cout << "starting mplex\n";
            parent_->m_.multiplex_client->RequestClient(
                [this, callback](const auto& ec, auto* client) {
                  std::cout << "got mplex\n";
                  if (ec) {
                    callback(ec);
                    return;
                  }

                  parent_->m_.moteus_servo->SetClient(client);
                  callback(ec);
                });
          })
        .Add([this](auto callback) {
            std::cout << "starting timer\n";
            StartTimer();
            callback(mjlib::base::error_code());
          })
        .Add([this](auto callback) {
            std::cout << "starting stdin\n";
            factory_->AsyncCreate(
                stdin_options_,
                [this, callback](const auto& ec, auto stream) {
                  if (ec) {
                    callback(ec);
                    return;
                  }

                  this->stdin_stream_ = stream;
                  StartRead();
                  callback(ec);
                });
          })
        .Start(callback);
  }

  void StartTimer() {
    timer_.expires_from_now(mjlib::base::ConvertSecondsToDuration(
                                param_->period_s));
    timer_.async_wait(std::bind(&Impl::HandleTimer, this,
                                std::placeholders::_1));
  }

  void StartRead() {
    boost::asio::async_read_until(*stdin_stream_, stdin_streambuf_, "\n",
                                  std::bind(&Impl::HandleRead, this, pl::_1));
  }

  void HandleRead(mjlib::base::error_code ec) {
    mjlib::base::FailIf(ec);

    std::ostringstream ostr;
    ostr << &stdin_streambuf_;

    ProcessCommand(ostr.str());

    StartRead();
  }

  void ProcessCommand(const std::string& cmd_in) {
    const auto cmd = boost::trim_copy(cmd_in);
    std::cout << "Got: " + cmd << "\n";

    if (cmd == "s") {
      parent_->m_.moteus_servo->EnablePower(ServoInterface::kPowerFree, {},
                                            mjlib::base::FailIf);
      state_ = kIdle;
      return;
    }

    // Otherwise, just advance.

    switch (state_) {
      case State::kIdle: {
        state_ = State::kPowered;
        break;
      }
      case State::kPowered: {
        state_ = State::kPrepositioning;
        break;
      }
      case State::kPrepositioning:
      case State::kStanding: {
        break;
      }
      case State::kReady: {
        state_ = State::kSquatting;
        break;
      }
      case State::kSquatting:
      case State::kJumping:
      case State::kLandingPrepare:
      case State::kLanding: {
        break;
      }
    }
  }

  void HandleTimer(mjlib::base::error_code ec) {
    if (ec == boost::asio::error::operation_aborted) { return; }
    mjlib::base::FailIf(ec);

    StartTimer();
    ProcessState();

    std::cout << fmt::format(
        "{} {:16} \n  {}\n\n",
        boost::lexical_cast<std::string>(mjlib::io::Now(executor_.context())),
        StateMapper().at(state_),
        FormatCurrent());

    RequestStatus();
  }

  std::string FormatCurrent() {
    static constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
    std::ostringstream ostr;
    bool first = true;
    for (const auto& item : current_stats_) {
      if (!first) { ostr << ",  "; }
      first = false;
      ostr << fmt::format(
          "{:d}={:5.1f}/t:{:4.1f}/v:{:5.1f}",
          item.first,
          item.second.angle_deg.value_or(kNaN),
          item.second.torque_Nm.value_or(kNaN),
          item.second.velocity_dps.value_or(kNaN));
    }
    return ostr.str();
  }

  void RequestStatus() {
    ServoInterface::StatusOptions options;
    options.pose = true;
    options.velocity = true;
    options.torque = true;
    options.error = true;
    parent_->m_.moteus_servo->GetStatus(
        {1, 2, 3}, options, std::bind(&Impl::HandleStatus, this, pl::_1, pl::_2));
  }

  void HandleStatus(const mjlib::base::error_code& ec,
                    const std::vector<ServoInterface::JointStatus>& status) {
    if (ec == boost::asio::error::operation_aborted) {
      // Guess it must be a transient error.  just bail for now.
      return;
    }
    mjlib::base::FailIf(ec);

    for (auto item : status) {
      current_stats_[item.address] = item;
    }
  }

  double GetJointSpeed(double input_dps, int id) {
    if (tibia_ids_.count(id)) {
      return 2.0 * input_dps;
    }
    return input_dps;
  }

  double GetJointServo(const JointSetup& joint, int id) {
    const int leg_num =
        (id >= 7 && id <= 9) ? 0 :
        (id >= 1 && id <= 3) ? 1 :
        (id >= 4 && id <= 6) ? 2 :
        (id >= 10 && id <= 12) ? 3 :
        -1;

    const double femur_sign =
        (leg_num == 0 || leg_num == 2) ? -1.0 : 1.0;
    const double tibia_sign =
        (leg_num == 1 || leg_num == 3) ? -1.0 : 1.0;

    if (shoulder_ids_.count(id)) {
      return joint.shoulder;
    } else if (femur_ids_.count(id)) {
      return joint.femur * femur_sign;
    } else if (tibia_ids_.count(id)) {
      return joint.tibia * tibia_sign;
    }
    mjlib::base::Fail("unknown id");
  }

  template <typename JointSetter>
  void EmitCommand(JointSetter joint_setter) {
    std::vector<ServoInterface::Joint> joints;
    for (int id : all_ids_) {
      ServoInterface::Joint joint;
      joint.address = id;
      joint.angle_deg = std::numeric_limits<double>::quiet_NaN();
      joint.max_torque_Nm = 50.0;

      joint_setter(&joint);

      joints.push_back(joint);
    }
    parent_->m_.moteus_servo->SetPose(joints, mjlib::base::FailIf);
  }

  bool InPosition(const JointSetup& joint_deg) {
    // Wait for one of the legs to get close to the final position.
    std::optional<double> max_error;
    for (int id : { 1, 2, 3} ) {
      const double error = std::abs(
          GetJointServo(joint_deg, id) -
          current_stats_[id].angle_deg.value_or(-100000));
      if (!max_error || error > *max_error) { max_error = error; }
    }
    BOOST_ASSERT(!!max_error);
    return (*max_error < 3);
  }

  void ProcessState() {
    const auto now = mjlib::io::Now(executor_.context());
    if (state_ != start_time_state_ ||
        start_time_time_.is_special()) {
      start_time_state_ = state_;
      start_time_time_ = now;
    }

    const auto state_time_s = mjlib::base::ConvertDurationToSeconds(
        now - start_time_time_);

    switch (state_) {
      case State::kIdle: {
        break;
      }
      case State::kPowered: {
        parent_->m_.moteus_servo->EnablePower(
            ServoInterface::kPowerBrake, {}, mjlib::base::FailIf);
        break;
      }
      case State::kPrepositioning: {
        // Repeatedly command all servos, in an idempotent manner, to
        // slowly reach their initial state.

        EmitCommand([&](auto* joint) {
            joint->goal_deg =
                GetJointServo(param_->preposition_deg, joint->address);
            joint->velocity_dps =
                GetJointSpeed(param_->preposition_speed_dps, joint->address);
            joint->max_torque_Nm = param_->preposition_max_torque_Nm;
          });

        if (state_time_s > param_->preposition_time_s) {
          state_ = State::kStanding;
        }
        break;
      }
      case State::kStanding: {
        EmitCommand([&](auto* joint) {
            joint->goal_deg =
                GetJointServo(param_->standing_deg, joint->address);
            joint->velocity_dps =
                GetJointSpeed(param_->standing_speed_dps, joint->address);
          });

        if (state_time_s > param_->standing_time_s) {
          state_ = State::kReady;
        }
        break;
      }
      case State::kReady: {
        // Nothing to do here but wait.
        break;
      }
      case State::kSquatting: {
        EmitCommand([&](auto* joint) {
            joint->goal_deg = GetJointServo(param_->squat_deg, joint->address);
            joint->velocity_dps =
                GetJointSpeed(param_->squat_dps, joint->address);
          });

        if (state_time_s > param_->squat_time_s) {
          state_ = State::kJumping;
        }

        break;
      }
      case State::kJumping: {
        EmitCommand([&](auto* joint) {
            joint->goal_deg = GetJointServo(param_->jump_deg, joint->address);
            joint->velocity_dps =
                GetJointSpeed(param_->jump_dps, joint->address);
            joint->max_torque_Nm = param_->jump_max_torque_Nm;
          });

        if (InPosition(param_->jump_deg)) {
          state_ = State::kLandingPrepare;
        }
        break;
      }
      case State::kLandingPrepare: {
        EmitCommand([&](auto* joint) {
            joint->goal_deg =
                GetJointServo(param_->landing_prepare_deg, joint->address);
            joint->velocity_dps = GetJointSpeed(param_->landing_prepare_dps, joint->address);
          });

        if (InPosition(param_->landing_prepare_deg) ||
            state_time_s > param_->landing_prepare_time_s) {
          state_ = State::kLanding;
        }
        break;
      }
      case State::kLanding: {
        EmitCommand([&](auto* joint) {
            joint->angle_deg =
                GetJointServo(param_->landing_prepare_deg, joint->address);
            joint->kp = param_->landing_kp;
          });

        // When the velocity is low, and torque is being applied, then
        // we can switch to standing.
        std::optional<double> max_velocity_dps;
        for (int id : { 1, 2, 3 }) {
          const auto this_dps = std::abs(
              current_stats_[id].velocity_dps.value_or(1000.0));
          if (!max_velocity_dps || this_dps > *max_velocity_dps) {
            max_velocity_dps = this_dps;
          }
        }
        BOOST_ASSERT(!!max_velocity_dps);

        double sum_torque_Nm = 0.0;
        for (int id : { 1, 2 }) {
          sum_torque_Nm += std::abs(
              current_stats_[id].torque_Nm.value_or(0.0));
        }

        if (*max_velocity_dps < param_->landing_threshold_dps &&
            sum_torque_Nm > param_->landing_torque_Nm) {
          state_ = State::kStanding;
        }

        break;
      }
    }
  }

  JumpTest* const parent_;
  JumpTest::Parameters* const param_;
  boost::asio::executor executor_;
  mjlib::io::StreamFactory* const factory_;
  boost::program_options::options_description options_;
  mjlib::io::DeadlineTimer timer_;

  const std::vector<int> all_ids_ = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};
  const std::set<int> shoulder_ids_ = {3, 6, 9, 12};
  const std::set<int> femur_ids_ = {1, 4, 7, 10};
  const std::set<int> tibia_ids_ = {2, 5, 8, 11};

  mjlib::io::StreamFactory::Options stdin_options_;
  mjlib::io::SharedStream stdin_stream_;
  boost::asio::streambuf stdin_streambuf_;

  State state_ = State::kIdle;

  State start_time_state_ = state_;
  boost::posix_time::ptime start_time_time_;
  std::map<int, ServoInterface::JointStatus> current_stats_;
};

JumpTest::JumpTest(base::Context& context)
    : impl_(std::make_unique<Impl>(this, context)) {
  m_.multiplex_client = std::make_unique<MultiplexClient>(
      impl_->executor_);
  m_.moteus_servo = std::make_unique<MoteusServo>(
      impl_->executor_, context.telemetry_registry.get());

  mjlib::base::ProgramOptionsArchive(&impl_->options_).Accept(&impl_->stdin_options_);
  mjlib::base::ProgramOptionsArchive(&impl_->options_).Accept(&parameters_);
}

JumpTest::~JumpTest() {}

void JumpTest::AsyncStart(mjlib::io::ErrorCallback handler) {
  impl_->AsyncStart(handler);
}

boost::program_options::options_description* JumpTest::options() {
  return &impl_->options_;
}

}
}
