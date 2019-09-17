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

#include "mech/moteus_servo.h"

#include <functional>

#include <boost/signals2/signal.hpp>

#include <fmt/format.h>

#include "mjlib/base/fail.h"
#include "mjlib/base/limit.h"
#include "mjlib/base/program_options_archive.h"
#include "mjlib/io/now.h"
#include "mjlib/multiplex/threaded_client.h"

#include "base/logging.h"
#include "mech/moteus.h"

namespace pl = std::placeholders;
namespace mp = mjlib::multiplex;

namespace mjmech {
namespace mech {

namespace {

using mjlib::base::Limit;
using Value = mp::Format::Value;

template <typename T>
uint32_t u32(T value) {
  return static_cast<uint32_t>(value);
}

template <typename T>
Value ScaleSaturate(double value, double scale) {
  if (!std::isfinite(value)) {
    return std::numeric_limits<T>::min();
  }

  const double scaled = value / scale;
  const auto max = std::numeric_limits<T>::max();
  // We purposefully limit to +- max, rather than to min.  The minimum
  // value for our two's complement types is reserved for NaN.
  return Limit<T>(static_cast<T>(scaled), -max, max);
}

enum RegisterTypes {
  kInt8 = 0,
  kInt16 = 1,
  kInt32 = 2,
  kFloat = 3,
};

Value ScaleMapping(double value,
                   double int8_scale, double int16_scale, double int32_scale,
                   RegisterTypes type) {
  switch (type) {
    case kInt8: return ScaleSaturate<int8_t>(value, int8_scale);
    case kInt16: return ScaleSaturate<int16_t>(value, int16_scale);
    case kInt32: return ScaleSaturate<int32_t>(value, int32_scale);
    case kFloat: return static_cast<float>(value);
  }
  MJ_ASSERT(false);
  return Value(static_cast<int8_t>(0));
}

Value write_position(double value, RegisterTypes reg) {
  return ScaleMapping(value / 360.0, 0.01, 0.001, 0.00001, reg);
}

Value write_velocity(double value, RegisterTypes reg) {
  return ScaleMapping(value / 360.0, 0.1, 0.001, 0.00001, reg);
}

Value write_torque(double value, RegisterTypes reg) {
  return ScaleMapping(value, 0.5, 0.01, 0.001, reg);
}

Value write_pwm(double value, RegisterTypes reg) {
  return ScaleMapping(value, 1.0 / 127.0,
                      1.0 / 32767.0,
                      1.0 / 2147483647.0,
                      reg);
}

struct ValueScaler {
  double int8_scale;
  double int16_scale;
  double int32_scale;

  double operator()(int8_t value) const {
    if (value == std::numeric_limits<int8_t>::min()) {
      return std::numeric_limits<double>::quiet_NaN();
    }
    return value * int8_scale;
  }

  double operator()(int16_t value) const {
    if (value == std::numeric_limits<int16_t>::min()) {
      return std::numeric_limits<double>::quiet_NaN();
    }
    return value * int16_scale;
  }

  double operator()(int32_t value) const {
    if (value == std::numeric_limits<int32_t>::min()) {
      return std::numeric_limits<double>::quiet_NaN();
    }
    return value * int32_scale;
  }

  double operator()(float value) const {
    return value;
  }
};

double ReadScale(Value value,
                 double int8_scale,
                 double int16_scale,
                 double int32_scale) {
  return std::visit(ValueScaler{int8_scale, int16_scale, int32_scale}, value);
}

double read_position(Value value) {
  return ReadScale(value, 0.01, 0.001, 0.00001) * 360.0;
}

double read_torque(Value value) {
  return ReadScale(value, 0.5, 0.01, 0.001);
}

double read_voltage(Value value) {
  return ReadScale(value, 1, 0.1, 0.001);
}

double read_temperature(Value value) {
  return ReadScale(value, 1.0, 0.1, 0.001);
}

int read_int(Value value) {
  return std::visit([](auto v) { return static_cast<int>(v); }, value);
}

struct Command {
  boost::posix_time::ptime timestamp;
  std::vector<ServoInterface::Joint> joints;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(timestamp));
    a->Visit(MJ_NVP(joints));
  }
};
}

class MoteusServo::Impl {
 public:
  Impl(const boost::asio::executor& executor,
       base::TelemetryRegistry* telemetry_registry)
      : executor_(executor) {
    mjlib::base::ProgramOptionsArchive(&options_).Accept(&parameters_);
    telemetry_registry->Register("moteus_command", &command_signal_);
  }

  void AsyncStart(mjlib::io::ErrorCallback handler) {
    boost::asio::post(
        executor_,
        std::bind(handler, mjlib::base::error_code()));
  }

  void SetPose(const std::vector<Joint>& joints,
               mjlib::io::ErrorCallback handler) {

    {
      Command command;
      command.timestamp = mjlib::io::Now(executor_.context());
      command.joints = joints;
      std::sort(command.joints.begin(), command.joints.end(),
                [](const auto& lhs, const auto& rhs) {
                  return lhs.address < rhs.address;
                });
      command_signal_(&command);
    }

    ReallySetPose(joints, handler);
  }

  void ReallySetPose(const std::vector<Joint>& joints,
                     mjlib::io::ErrorCallback handler) {
    if (!mp_client_) { return; }
    if (outstanding_) {
      log_.debug("skipping SetPose because we are backed up");
      boost::asio::post(
          executor_,
          std::bind(handler, boost::asio::error::operation_aborted));
      return;
    }

    request_.requests.clear();

    for (const auto& joint : joints) {
      request_.requests.push_back({});
      auto& request_pair = request_.requests.back();
      request_pair.id = joint.address;
      auto& request = request_pair.request;

      request = {};
      request.WriteSingle(moteus::kMode,
                          static_cast<int8_t>(moteus::Mode::kPosition));

      request.WriteMultiple(
          moteus::kCommandPosition,
          {
            static_cast<float>(joint.angle_deg / 360.0),
            static_cast<float>(joint.velocity_dps / 360.0f),
            static_cast<float>(joint.torque_Nm),
            static_cast<float>(joint.kp),
            1.0f,
            static_cast<float>(joint.max_torque_Nm * joint.max_torque_scale),
            (std::isfinite(joint.goal_deg) ?
              static_cast<float>(joint.goal_deg / 360.0f) :
              std::numeric_limits<float>::quiet_NaN()),
          });
    }

    outstanding_ = true;
    mp_client_->AsyncRegister(
        &request_,
        nullptr,
        std::bind(&Impl::HandleSetPose, this, pl::_1, handler));
  }

  void HandleSetPose(const mjlib::base::error_code& ec,
                     mjlib::io::ErrorCallback handler) {
    mjlib::base::FailIf(ec);
    outstanding_ = false;

    handler(ec);
  }

  void EnablePower(PowerState power_state,
                   const std::vector<int>& ids,
                   mjlib::io::ErrorCallback handler) {
    if (!mp_client_) { return; }

    request_.requests.resize(ids.size());
    for (size_t i = 0; i < ids.size(); i++) {
      auto& request_pair = request_.requests[i];
      request_pair.id = ids[i];

      const auto value = [&]() {
        switch (power_state) {
          case PowerState::kPowerFree: return moteus::Mode::kStopped;
          case PowerState::kPowerBrake: return moteus::Mode::kStopped;
          case PowerState::kPowerEnable: return moteus::Mode::kPosition;
        }
      }();
      request_pair.request = {};
      request_pair.request.WriteSingle(
          moteus::kMode, static_cast<int8_t>(value));
    };

    mp_client_->AsyncRegister(
        &request_,
        nullptr,
        handler);
  }

  void GetStatus(const std::vector<int>& ids,
                 const StatusOptions& status_options,
                 StatusHandler handler) {

    read_reply_.replies.clear();
    read_request_.requests.resize(ids.size());

    for (size_t i =0; i < ids.size(); i++) {
      auto& request_pair = read_request_.requests[i];
      request_pair.id = ids[i];
      auto& request = request_pair.request;
      request = {};

      request.ReadSingle(moteus::kMode, kInt8);
      if (status_options.pose) {
        request.ReadSingle(moteus::kPosition, kFloat);
      }
      if (status_options.temperature) {
        request.ReadSingle(moteus::kTemperature, kFloat);
      }
      if (status_options.voltage) {
        request.ReadSingle(moteus::kVoltage, kFloat);
      }
      if (status_options.velocity) {
        request.ReadSingle(moteus::kVelocity, kFloat);
      }
      if (status_options.torque) {
        request.ReadSingle(moteus::kTorque, kFloat);
      }
      request.ReadSingle(moteus::kFault, kInt32);
    }

    mp_client_->AsyncRegister(
        &read_request_,
        &read_reply_,
        std::bind(&Impl::HandleStatus, this, pl::_1, handler));
  }

  void HandleStatus(const mjlib::base::error_code& ec,
                    StatusHandler handler) {
    if (ec) {
      log_.debug(fmt::format("reply with error: {}", ec.message()));
      boost::asio::post(
          executor_,
          std::bind(handler, ec, std::vector<JointStatus>()));
      return;
    }

    std::vector<JointStatus> result;
    for (const auto& reply_pair : read_reply_.replies) {
      JointStatus this_status;
      this_status.address = reply_pair.id;
      const auto& reply = reply_pair.reply;

      log_.debug(fmt::format("got {} regs of data", reply.size()));

      ReadJoint(reply, &this_status);
      result.push_back(this_status);
    }

    boost::asio::post(
        executor_,
        std::bind(handler, ec, result));
  }

  void Update(PowerState power_state,
              const StatusOptions& status_options,
              const std::vector<Joint>* command,
              std::vector<JointStatus>* output,
              mjlib::io::ErrorCallback callback) {
    MJ_ASSERT(!outstanding_);

    auto map_power_state = [](auto power_state) {
      switch (power_state) {
        case kPowerFree:
        case kPowerBrake: {
          return static_cast<int8_t>(moteus::Mode::kStopped);
        }
        case kPowerEnable: {
          return static_cast<int8_t>(moteus::Mode::kPosition);
        }
      }
    };

    request_.requests.resize(command->size());
    for (size_t i = 0; i < command->size(); i++) {
      const auto& joint = (*command)[i];
      auto& request_pair = request_.requests[i];
      request_pair.id = joint.address;
      auto& request = request_pair.request;
      request = {};

      // First, set our power state.
      request.WriteSingle(moteus::kMode,
                          map_power_state(power_state));

      PopulateCommand(joint, &request);
      PopulateQuery(status_options, &request);
    }

    reply_.replies.clear();

    mp_client_->AsyncRegister(
        &request_,
        &reply_,
        std::bind(&Impl::HandleUpdate, this, pl::_1, output, callback));
  }

  void ReadJoint(const mp::RegisterReply& reply,
                 JointStatus* joint) {
    for (auto& pair : reply) {
      auto* maybe_value = std::get_if<Value>(&pair.second);
      if (!maybe_value) { continue; }
      auto value = *maybe_value;
      switch (static_cast<moteus::Register>(pair.first)) {
        case moteus::kMode: {
          const auto mode = read_int(value);
          joint->torque_on = mode >= 5;
          break;
        }
        case moteus::kPosition: {
          joint->angle_deg = read_position(value);
          break;
        }
        case moteus::kVelocity: {
          joint->velocity_dps = read_position(value);
          break;
        }
        case moteus::kTorque: {
          joint->torque_Nm = read_torque(value);
          break;
        }
        case moteus::kVoltage: {
          joint->voltage = read_voltage(value);
          break;
        }
        case moteus::kTemperature: {
          joint->temperature_C = read_temperature(value);
          break;
        }
        default: {
          break;
        }
      }
    }
  }

  void HandleUpdate(const mjlib::base::error_code& ec,
                    std::vector<JointStatus>* output,
                    mjlib::io::ErrorCallback callback) {
    if (ec) {
      boost::asio::post(
          executor_,
          std::bind(callback, ec));
      return;
    }

    output->clear();
    for (auto& reply_pair : reply_.replies) {
      output->push_back({});
      auto& joint = output->back();
      joint.address = reply_pair.id;
      ReadJoint(reply_pair.reply, &joint);
    }

    boost::asio::post(
        executor_,
        std::bind(callback, ec));
  }

  void PopulateQuery(const StatusOptions& status_options,
                     mp::RegisterRequest* request) {
    // We always ask for at least the mode as an int16.  We optionally
    // ask for other things as well.
    {
      moteus::Register max_reg = moteus::kMode;
      if (status_options.pose) { max_reg = moteus::kPosition; }
      if (status_options.velocity) { max_reg = moteus::kVelocity; }
      if (status_options.torque) { max_reg = moteus::kTorque; }

      const auto len = u32(max_reg) - u32(moteus::kMode) + 1;

      request->ReadMultiple(moteus::kMode, len, 1);
    }

    // Now do the second grouping, these we get as int8s.  Here, we
    // might not do anything at all if none of the respective things
    // have been asked for.
    {
      std::optional<moteus::Register> max_reg;
      if (status_options.voltage) { max_reg = moteus::kVoltage; }
      if (status_options.temperature) { max_reg = moteus::kTemperature; }
      if (status_options.error) { max_reg = moteus::kFault; }

      std::optional<moteus::Register> min_reg;
      if (status_options.error) { min_reg = moteus::kFault; }
      if (status_options.temperature) { min_reg = moteus::kTemperature; }
      if (status_options.voltage) { min_reg = moteus::kVoltage; }

      if (!!max_reg) {
        const auto len = u32(*max_reg) - u32(*min_reg) + 1;
        request->ReadMultiple(*min_reg, len, 0);
      }
    }
  }

  void PopulateCommand(const Joint& joint, mp::RegisterRequest* request) {
    auto& values = values_cache_;

    // Now, figure out how to encode our command.  We write out all
    // our command as a single WriteMultiple of int16s.  We use a
    // shorter list of values if that is possible.
    if (joint.angle_deg != 0.0) { values.resize(1); }
    if (joint.velocity_dps != 0.0) { values.resize(2);}
    if (joint.torque_Nm != 0.0) { values.resize(3); }
    if (joint.kp != 1.0) { values.resize(4); }
    if (joint.max_torque_scale != 1.0 ||
        joint.max_torque_Nm != joint.kDefaultMaxTorque) {
      values.resize(6);
    }
    if (std::isfinite(joint.goal_deg )) { values.resize(7); }

    // Guess we're not sending anything.
    if (values.empty()) { return; }

    for (size_t i = 0; i < values.size(); i++) {
      switch (i) {
        case 0: {
          values[i] = write_position(joint.angle_deg, kInt16);
          break;
        }
        case 1: {
          values[i] = write_velocity(joint.velocity_dps, kInt16);
          break;
        }
        case 2: {
          values[i] = write_torque(joint.torque_Nm, kInt16);
          break;
        }
        case 3: {
          values[i] = write_pwm(joint.kp, kInt16);
          break;
        }
        case 4: {
          values[i] = write_pwm(1.0, kInt16);  // kd
          break;
        }
        case 5: {
          values[i] = write_torque(
              joint.max_torque_Nm * joint.max_torque_scale, kInt16);
          break;
        }
        case 6: {
          values[i] = write_position(joint.goal_deg, kInt16);
          break;
        }
      }
    }

    request->WriteMultiple(moteus::kCommandPosition, values);
  }

  base::LogRef log_ = base::GetLogInstance("MoteusServo");

  boost::signals2::signal<void (const Command*)> command_signal_;

  boost::asio::executor executor_;

  Parameters parameters_;
  boost::program_options::options_description options_;

  bool outstanding_ = false;
  mp::ThreadedClient::Request request_;
  mp::ThreadedClient::Reply reply_;

  mp::ThreadedClient::Request read_request_;
  mp::ThreadedClient::Reply read_reply_;

  mjlib::multiplex::ThreadedClient* mp_client_ = nullptr;
  std::vector<Value> values_cache_;
};

MoteusServo::MoteusServo(const boost::asio::executor& executor,
                         base::TelemetryRegistry* telemetry_registry)
    : impl_(std::make_unique<Impl>(executor, telemetry_registry)) {}
MoteusServo::~MoteusServo() {}

MoteusServo::Parameters* MoteusServo::parameters() {
  return &impl_->parameters_;
}

boost::program_options::options_description* MoteusServo::options() {
  return &impl_->options_;
}

void MoteusServo::AsyncStart(mjlib::io::ErrorCallback handler) {
  impl_->AsyncStart(handler);
}

void MoteusServo::SetClient(mjlib::multiplex::ThreadedClient* client) {
  impl_->mp_client_ = client;
}

void MoteusServo::SetPose(const std::vector<Joint>& joints,
                          mjlib::io::ErrorCallback handler) {
  impl_->SetPose(joints, handler);
}

void MoteusServo::EnablePower(PowerState power_state,
                              const std::vector<int>& ids,
                              mjlib::io::ErrorCallback handler) {
  auto ids_copy = ids;
  if (ids_copy.empty()) { ids_copy.push_back(moteus::Ids::kBroadcastId); }
  impl_->EnablePower(power_state, ids_copy, handler);
}

void MoteusServo::GetStatus(const std::vector<int>& ids,
                            const StatusOptions& status_options,
                            StatusHandler handler) {
  impl_->GetStatus(ids, status_options, handler);
}

void MoteusServo::ClearErrors(const std::vector<int>&,
                              mjlib::io::ErrorCallback callback) {
  boost::asio::post(
      impl_->executor_,
      std::bind(callback, mjlib::base::error_code()));
}

void MoteusServo::Update(PowerState power_state,
                         const StatusOptions& status_options,
                         const std::vector<Joint>* command,
                         std::vector<JointStatus>* output,
                         mjlib::io::ErrorCallback callback) {
  impl_->Update(power_state, status_options, command, output, callback);
}

}
}
