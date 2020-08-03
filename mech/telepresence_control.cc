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

    outstanding_ = true;

    timing_ = ControlTiming(executor_, timing_.cycle_start());

    RunControl();
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
    for (const auto& reply : client_command_reply_) {
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

  void DoControl_Stop() {
    ControlData control;
    Control(control);
  }

  void DoControl_Active() {
    ControlData control;

    control.power = true;
    const double torque_Nm = pid_.Apply(
        status_.servo1.angle_deg, status_.servo2.angle_deg,
        status_.servo1.velocity_dps, status_.servo2.velocity_dps,
        1.0 / parameters_.period_s);

    // We process our scale so that we only *decrease* gains, not
    // increase them.  That means you can tweak this parameter and not
    // worry too much about loop stability, but one parameter still
    // changes the relationship between both sides.
    double id1_scale = parameters_.id1_scale;
    double id2_scale = 1.0;

    if (id1_scale > 1.0) {
      id2_scale /= id1_scale;
      id1_scale = 1.0;
    }

    control.servo1.torque_Nm = id1_scale * torque_Nm;
    control.servo2.torque_Nm = id2_scale * -torque_Nm;

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

  void AddServoCommand(int id, bool power, double torque_Nm, double velocity_dps) {
    client_command_.push_back({});
    auto& request = client_command_.back();
    request.id = id;

    const auto mode =
        power ? moteus::Mode::kPosition : moteus::Mode::kStopped;
    request.request.WriteSingle(moteus::kMode, static_cast<int8_t>(mode));

    if (power) {
      request.request.WriteSingle(
          moteus::kCommandFeedforwardTorque,
          moteus::WriteTorque(torque_Nm, moteus::kInt16));

      {
        // Static only to save allocations.
        static std::vector<moteus::Value> values;
        if (values.empty()) {
          values.push_back(moteus::WritePwm(0, moteus::kInt8));
          values.push_back(moteus::WritePwm(0, moteus::kInt8));
        }
        request.request.WriteMultiple(moteus::kCommandKpScale, values);
      }
    }

    request.request.ReadMultiple(moteus::Register::kPosition, 3, 2);
    // To reach 2kHz
    // request.request.ReadMultiple(moteus::Register::kVoltage, 3, 1);
  }

  void Control(const ControlData& control) {
    timing_.finish_query();
    timing_.finish_status();
    timing_.finish_control();

    ControlLog control_log;
    control_log.timestamp = Now();
    control_log.control = control;
    control_signal_(&control_log);

    client_command_.clear();

    AddServoCommand(
        parameters_.id1, control.power,
        control.servo1.torque_Nm, control.servo1.velocity_dps);
    AddServoCommand(
        parameters_.id2, control.power,
        control.servo2.torque_Nm, control.servo2.velocity_dps);

    client_command_reply_.clear();
    client_->AsyncTransmit(
        &client_command_, &client_command_reply_,
        std::bind(&Impl::HandleCommand, this, pl::_1));
  }

  void HandleCommand(const mjlib::base::error_code& ec) {
    timing_.finish_command();

    UpdateStatus();

    status_.timestamp = Now();
    status_.timing = timing_.status();
    telepresence_signal_(&status_);

    outstanding_ = false;
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

  using Request = Client::Request;
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
