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

#include "mech/turret_control.h"

#include <boost/asio/post.hpp>

#include "mjlib/base/clipp_archive.h"
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

  const TurretControl::CommandData* command = &ignored_command;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(timestamp));
    const_cast<TurretControl::CommandData*>(command)->Serialize(a);
  }

  static inline TurretControl::CommandData ignored_command;
};

struct ControlLog {
  boost::posix_time::ptime timestamp;

  TurretControl::ControlData control;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(timestamp));
    a->Visit(MJ_NVP(control));
  }
};
}

class TurretControl::Impl {
 public:
  Impl(base::Context& context,
       ClientGetter client_getter,
       ImuGetter imu_getter)
      : executor_(context.executor),
        client_getter_(client_getter),
        imu_getter_(imu_getter) {
    context.telemetry_registry->Register("imu", &imu_signal_);
    context.telemetry_registry->Register("turret", &turret_signal_);
    context.telemetry_registry->Register("command", &command_signal_);
    context.telemetry_registry->Register("control", &control_signal_);
  }

  void AsyncStart(mjlib::io::ErrorCallback callback) {
    client_ = client_getter_();
    imu_client_ = imu_getter_();

    PopulateStatusRequest();

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

  // private

  void PopulateStatusRequest() {
    status_request_ = {};
    for (int id : { 1, 2}) {
      status_request_.push_back({});
      auto& current = status_request_.back();
      current.id = id;
      current.request.ReadMultiple(moteus::Register::kMode, 4, 3);
      current.request.ReadMultiple(moteus::Register::kVoltage, 3, 1);
    }
  }

  void HandleTimer(const mjlib::base::error_code& ec) {
    if (ec == boost::asio::error::operation_aborted) { return; }
    mjlib::base::FailIf(ec);

    if (!client_) { return; }
    if (outstanding_) { return; }

    timing_ = ControlTiming(executor_, timing_.cycle_start());

    outstanding_ = true;

    status_outstanding_ = 2;
    status_reply_ = {};

    client_->AsyncRegisterMultiple(
        status_request_, &status_reply_,
        std::bind(&Impl::HandleStatus, this, pl::_1));

    imu_client_->ReadImu(
        &imu_data_, std::bind(&Impl::HandleStatus, this, pl::_1));
  }

  void HandleStatus(const mjlib::base::error_code& ec) {
    mjlib::base::FailIf(ec);

    status_outstanding_--;
    if (status_outstanding_ > 0) { return; }

    imu_signal_(&imu_data_);

    UpdateStatus();
    timing_.finish_status();

    RunControl();
    timing_.finish_control();
  }

  void UpdateStatus() {
    for (const auto& reply : status_reply_.replies) {
      auto* const servo = [&]() -> Status::GimbalServo* {
        if (reply.id == 1) { return &status_.pitch_servo; }
        if (reply.id == 2) { return &status_.yaw_servo; }
        return nullptr;
      }();
      if (!servo) { continue; }

      servo->id = reply.id;
      const double sign = servo_sign_.at(reply.id);

      for (const auto& pair : reply.reply) {
        const auto* maybe_value = std::get_if<moteus::Value>(&pair.second);
        if (!maybe_value) { continue; }
        const auto& value = *maybe_value;
        switch (static_cast<moteus::Register>(pair.first)) {
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
    }

    status_.imu.pitch_deg = imu_data_.euler_deg.pitch;
    status_.imu.pitch_rate_dps = imu_data_.rate_dps.y();
    status_.imu.yaw_deg = imu_data_.euler_deg.yaw;
    status_.imu.yaw_rate_dps = imu_data_.rate_dps.z();

    if (status_.mode != Mode::kFault) {
      std::string fault;

      for (auto* servo : { &status_.pitch_servo, &status_.yaw_servo }) {
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

        status_.control = {};

        // Start controlling right where we are.
        status_.control.pitch.angle_deg = imu_data_.euler_deg.pitch;
        status_.control.yaw.angle_deg = imu_data_.euler_deg.yaw;

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
    turret_signal_(&status_);

    outstanding_ = false;
  }

  void DoControl_Stop() {
    ControlData control;
    Control(control);
  }

  void DoControl_Active() {
    status_.control.pitch.angle_deg +=
        current_command_.pitch_rate_dps * parameters_.period_s;
    status_.control.yaw.angle_deg +=
        current_command_.yaw_rate_dps * parameters_.period_s;

    // Wrap around our yaw angle to always be a limited distance from
    // the current actual.
    const double yaw_delta_deg =
        base::Degrees(
            base::WrapNegPiToPi(
                base::Radians(
                    status_.control.yaw.angle_deg - imu_data_.euler_deg.yaw)));
    status_.control.yaw.angle_deg = imu_data_.euler_deg.yaw + yaw_delta_deg;

    ControlData control;
    control.pitch.power = true;
    control.pitch.torque_Nm =
        pitch_pid_.Apply(
            imu_data_.euler_deg.pitch, status_.control.pitch.angle_deg,
            imu_data_.rate_dps.y(), current_command_.pitch_rate_dps,
            1.0 / parameters_.period_s);

    control.yaw.power = true;
    control.yaw.torque_Nm =
        yaw_pid_.Apply(
            imu_data_.euler_deg.yaw, status_.control.yaw.angle_deg,
            imu_data_.rate_dps.z(), current_command_.yaw_rate_dps,
            1.0 / parameters_.period_s);

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

  void AddServoCommand(int id, const ControlData::Servo& servo) {
    const double sign = servo_sign_.at(id);

    client_command_.push_back({});
    auto& request = client_command_.back();
    request.id = id;

    const auto mode =
        servo.power ? moteus::Mode::kPosition : moteus::Mode::kStopped;
    request.request.WriteSingle(moteus::kMode, static_cast<int8_t>(mode));

    if (servo.power) {
      request.request.WriteSingle(
          moteus::kCommandFeedforwardTorque,
          static_cast<float>(sign * servo.torque_Nm));

      // Static only to save allocations.
      static std::vector<moteus::Value> values;
      if (values.empty()) {
        values.push_back(moteus::WritePwm(0, moteus::kInt8));
        values.push_back(moteus::WritePwm(0, moteus::kInt8));
      }
      request.request.WriteMultiple(moteus::kCommandKpScale, values);
    }
  }

  void Control(const ControlData& control) {
    ControlLog control_log;
    control_log.timestamp = Now();
    control_log.control = control;
    control_signal_(&control_log);

    client_command_ = {};
    AddServoCommand(1, control.pitch);
    AddServoCommand(2, control.yaw);

    client_command_reply_ = {};
    client_->AsyncRegisterMultiple(
        client_command_, &client_command_reply_,
        std::bind(&Impl::HandleCommand, this, pl::_1));
  }


  boost::posix_time::ptime Now() {
    return mjlib::io::Now(executor_.context());
  }

  boost::asio::executor executor_;
  ClientGetter client_getter_;
  ImuGetter imu_getter_;

  base::LogRef log_ = base::GetLogInstance("QuadrupedControl");

  Status status_;
  CommandData current_command_;
  boost::posix_time::ptime current_command_timestamp_;
  Parameters parameters_;

  using Client = MultiplexClient::Client;
  Client* client_ = nullptr;
  ImuClient* imu_client_ = nullptr;

  mjlib::io::RepeatingTimer timer_{executor_};
  bool outstanding_ = false;
  int status_outstanding_ = 0;

  using Request = std::vector<Client::IdRequest>;
  Request status_request_;
  Client::Reply status_reply_;

  Request client_command_;
  Client::Reply client_command_reply_;

  AttitudeData imu_data_;
  boost::signals2::signal<void (const AttitudeData*)> imu_signal_;
  boost::signals2::signal<void (const Status*)> turret_signal_;
  boost::signals2::signal<void (const CommandLog*)> command_signal_;
  boost::signals2::signal<void (const ControlLog*)> control_signal_;

  ControlTiming timing_{executor_, {}};

  std::map<int, double> servo_sign_ = {
    { 1, 1.0 },
    { 2, -1.0 },
  };

  mjlib::base::PID pitch_pid_{&parameters_.pitch, &status_.control.pitch.pid};
  mjlib::base::PID yaw_pid_{&parameters_.yaw, &status_.control.yaw.pid};
};

TurretControl::TurretControl(base::Context& context,
                             ClientGetter client_getter,
                             ImuGetter imu_getter)
    : impl_(std::make_unique<Impl>(context, client_getter, imu_getter)) {}

TurretControl::~TurretControl() {}

void TurretControl::AsyncStart(mjlib::io::ErrorCallback callback) {
  impl_->AsyncStart(std::move(callback));
}

const TurretControl::Status& TurretControl::status() const {
  return impl_->status_;
}

void TurretControl::Command(const CommandData& command) {
  impl_->Command(command);
}

clipp::group TurretControl::program_options() {
  return mjlib::base::ClippArchive().Accept(&impl_->parameters_).release();
}


}
}
