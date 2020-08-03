// Copyright 2020 Josh Pieper, jjp@pobox.com.
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

#include "mech/telepresence_control.h"

#include <boost/asio/post.hpp>

#include "mjlib/base/clipp_archive.h"
#include "mjlib/base/limit.h"
#include "mjlib/io/repeating_timer.h"

#include "base/logging.h"
#include "base/telemetry_registry.h"

#include "mech/moteus.h"

namespace pl = std::placeholders;

namespace mjmech {
namespace mech {

namespace {
struct CommandLog {
  boost::posix_time::ptime timestamp;

  const TelepresenceControl::CommandData* command = &ignored_command;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(timestamp));
    const_cast<TelepresenceControl::CommandData*>(command)->Serialize(a);
  }

  static inline TelepresenceControl::CommandData ignored_command;
};

struct ControlLog {
  boost::posix_time::ptime timestamp;

  TelepresenceControl::ControlData control;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(timestamp));
    a->Visit(MJ_NVP(control));
  }
};
}

class TelepresenceControl::Impl {
 public:
  Impl(base::Context& context,
       ClientGetter client_getter)
      : executor_(context.executor),
        client_getter_(client_getter) {
    context.telemetry_registry->Register("telepresence", &telepresence_signal_);
    context.telemetry_registry->Register("command", &command_signal_);
    context.telemetry_registry->Register("control", &control_signal_);
  }

  void AsyncStart(mjlib::io::ErrorCallback callback) {
    client_ = client_getter_();

    timer_.start(mjlib::base::ConvertSecondsToDuration(parameters_.period_s),
                 std::bind(&Impl::HandleTimer, this, pl::_1));

    boost::asio::post(
        executor_,
        std::bind(std::move(callback), mjlib::base::error_code()));
  }

  void Command(const CommandData& data) {
    const auto now = Now();

    const bool higher_priority = data.priority >= current_command_.priority;
    const bool stale =
        !current_command_timestamp_.is_not_a_date_time() &&
        (mjlib::base::ConvertDurationToSeconds(
            now - current_command_timestamp_) > parameters_.command_timeout_s);

    if (!higher_priority && !stale) {
      return;
    }

    current_command_ = data;
    current_command_timestamp_ = now;

    CommandLog command_log;
    command_log.timestamp = Now();
    command_log.command = &current_command_;
    command_signal_(&command_log);
  }

  void HandleTimer(const mjlib::base::error_code& ec) {
    if (ec == boost::asio::error::operation_aborted) { return; }
    mjlib::base::FailIf(ec);

    if (!client_) { return; }
    if (outstanding_) { return; }

    timing_ = ControlTiming(executor_, timing_.cycle_start());

    outstanding_ = true;

    status_reply_.clear();
    // TODO: Fill in status_request_.
    client_->AsyncTransmit(
        &status_request_, &status_reply_,
        std::bind(&Impl::HandleStatus, this, pl::_1));
  }

  void HandleStatus(const mjlib::base::error_code& ec) {
    mjlib::base::FailIf(ec);

    UpdateStatus();

    timing_.finish_status();

    RunControl();
    timing_.finish_control();
  }

  void UpdateServo(const mjlib::multiplex::AsioClient::IdRegisterValue& reply,
                   Status::Servo* servo) {
    servo->id = reply.id;
    const double sign = 1.0;

    const auto* maybe_value = std::get_if<moteus::Value>(&reply.value);
    if (!maybe_value) { return; }
    const auto& value = *maybe_value;
    switch (static_cast<moteus::Register>(reply.reg)) {
      case moteus::kMode: {
        servo->mode = moteus::ReadInt(value);
        break;
      }
      case moteus::kPosition: {
        servo->angle_deg = sign * moteus::ReadPosition(value);
        break;
      }
      case moteus::kVelocity: {
        servo->velocity_dps = sign * moteus::ReadPosition(value);
        break;
      }
      case moteus::kTorque: {
        servo->torque_Nm = sign * moteus::ReadTorque(value);
        break;
      }
      case moteus::kVoltage: {
        servo->voltage = moteus::ReadVoltage(value);
        break;
      }
      case moteus::kTemperature: {
        servo->temperature_C = moteus::ReadTemperature(value);
        break;
      }
      case moteus::kFault: {
        servo->fault = moteus::ReadInt(value);
        break;
      }
      default: {
        break;
      }
    }
  }

  void UpdateStatus() {
    for (const auto& reply : status_reply_) {
      auto* const servo = [&]() -> Status::Servo* {
        if (reply.id == parameters_.id1) { return &status_.servo1; }
        if (reply.id == parameters_.id2) { return &status_.servo2; }
        return nullptr;
      }();
      if (servo) {
        UpdateServo(reply, servo);
      }
    }

    if (status_.mode != Mode::kFault) {
      std::string fault;

      for (auto* servo : { &status_.servo1, &status_.servo2 }) {
        if (servo->fault) {
          if (!fault.empty()) { fault += ", "; }
          fault += fmt::format("servo {} fault: {}", servo->id, servo->fault);
        }
      }

      if (!fault.empty()) {
        Fault(fault);
      }
    }
  }

  void RunControl() {
    if (current_command_.mode != status_.mode) {
      MaybeChangeMode();
    }

    switch (status_.mode) {
      case Mode::kStop: { DoControl_Stop(); break; }
      case Mode::kActive: { DoControl_Active(); break; }
      case Mode::kFault: { DoControl_Fault(); break; }
    }
  }

  void MaybeChangeMode() {
    const auto old_mode = status_.mode;
    switch (current_command_.mode) {
      case Mode::kStop: {
        status_.mode = Mode::kStop;
        break;
      }
      case Mode::kActive: {
        if (status_.mode == Mode::kFault) { break; }

        status_.mode = Mode::kActive;
        break;
      }
      case Mode::kFault: {
        mjlib::base::AssertNotReached();
      }
    }

    if (old_mode != status_.mode &&
        old_mode == Mode::kFault) {
      status_.fault = "";
    }
  }

  void HandleCommand(const mjlib::base::error_code& ec) {
    timing_.finish_command();

    status_.timestamp = Now();
    status_.timing = timing_.status();
    telepresence_signal_(&status_);

    outstanding_ = false;
  }

  void DoControl_Stop() {
    ControlData control;
    Control(control);
  }

  void DoControl_Active() {
    ControlData control;
    control.power = false;

    Control(control);
  }

  void DoControl_Fault() {
    DoControl_Stop();
  }

  void Fault(std::string_view message) {
    status_.mode = Mode::kFault;
    status_.fault = message;

    log_.warn("Fault: " + std::string(message));

    DoControl_Fault();
  }

  void Control(const ControlData& control) {
    ControlLog control_log;
    control_log.timestamp = Now();
    control_log.control = control;
    control_signal_(&control_log);

    client_command_.clear();

    client_command_reply_.clear();
    client_->AsyncTransmit(
        &client_command_, &client_command_reply_,
        std::bind(&Impl::HandleCommand, this, pl::_1));
  }


  boost::posix_time::ptime Now() {
    return mjlib::io::Now(executor_.context());
  }

  boost::asio::any_io_executor executor_;
  ClientGetter client_getter_;

  base::LogRef log_ = base::GetLogInstance("QuadrupedControl");

  Status status_;

  CommandData current_command_;
  boost::posix_time::ptime current_command_timestamp_;
  Parameters parameters_;

  using Client = mjlib::multiplex::AsioClient;
  Client* client_ = nullptr;

  mjlib::io::RepeatingTimer timer_{executor_};
  bool outstanding_ = false;
  int status_outstanding_ = 0;

  using Request = Client::Request;
  Request status_request_;
  Client::Reply status_reply_;

  Request client_command_;
  Client::Reply client_command_reply_;

  boost::signals2::signal<void (const Status*)> telepresence_signal_;
  boost::signals2::signal<void (const CommandLog*)> command_signal_;
  boost::signals2::signal<void (const ControlLog*)> control_signal_;

  ControlTiming timing_{executor_, {}};

  mjlib::base::PID pid_{&parameters_.pid, &status_.pid};
};

TelepresenceControl::TelepresenceControl(base::Context& context,
                                         ClientGetter client_getter)
    : impl_(std::make_unique<Impl>(context, client_getter)) {}

TelepresenceControl::~TelepresenceControl() {}

void TelepresenceControl::AsyncStart(mjlib::io::ErrorCallback callback) {
  impl_->AsyncStart(std::move(callback));
}

const TelepresenceControl::Status& TelepresenceControl::status() const {
  return impl_->status_;
}

void TelepresenceControl::Command(const CommandData& command) {
  impl_->Command(command);
}

clipp::group TelepresenceControl::program_options() {
  return mjlib::base::ClippArchive().Accept(&impl_->parameters_).release();
}


}
}
