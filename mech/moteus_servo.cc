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
#include "mjlib/base/program_options_archive.h"
#include "mjlib/multiplex/threaded_client.h"

#include "base/logging.h"
#include "base/now.h"
#include "mech/moteus.h"

namespace pl = std::placeholders;
namespace mp = mjlib::multiplex;

namespace mjmech {
namespace mech {

using Value = mp::Format::Value;

enum RegisterTypes {
  kInt32 = 2,
  kFloat = 3,
};

template <typename T>
T ReadCast(const mp::Format::ReadResult& value) {
  return std::visit([](auto a) {
      return static_cast<T>(a);
    }, std::get<Value>(value));
}

namespace {
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
  Impl(boost::asio::io_service& service,
       base::TelemetryRegistry* telemetry_registry)
      : service_(service) {
    mjlib::base::ProgramOptionsArchive(&options_).Accept(&parameters_);
    telemetry_registry->Register("moteus_command", &command_signal_);
  }

  void AsyncStart(mjlib::io::ErrorCallback handler) {
    service_.post(std::bind(handler, mjlib::base::error_code()));
  }

  void SetPose(const std::vector<Joint>& joints,
               mjlib::io::ErrorCallback handler) {

    {
      Command command;
      command.timestamp = base::Now(service_);
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

      // TODO(jpieper): MAJOR HACK.  This is not the right abstraction
      // layer for this.
      const bool is_shoulder = ((joint.address - 1) % 3) == 2;

      request.WriteMultiple(
          moteus::kCommandPosition,
          {
            static_cast<float>(joint.angle_deg / 360.0),
            static_cast<float>(joint.velocity_dps / 360.0f),
            static_cast<float>(joint.torque_Nm),
            static_cast<float>(joint.kp),
            1.0f,
            static_cast<float>(
                joint.power * (is_shoulder ?
                               parameters_.max_torque_shoulder_Nm :
                               parameters_.max_torque_legs_Nm)),
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

      request.ReadSingle(moteus::kMode, 0);
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
      service_.post(std::bind(handler, ec, std::vector<JointStatus>()));
      return;
    }

    std::vector<JointStatus> result;
    for (const auto& reply_pair : read_reply_.replies) {
      JointStatus this_status;
      this_status.address = reply_pair.id;
      const auto& reply = reply_pair.reply;

      log_.debug(fmt::format("got {} regs of data", reply.size()));

      for (const auto& pair : reply) {
        auto* maybe_value = std::get_if<Value>(&pair.second);
        if (!maybe_value) {
          log_.debug(fmt::format("error in reg {} : {}",
                                 pair.first, std::get<uint32_t>((pair.second))));
          // There must have been an error of some sort.
          //
          // TODO(jpieper): Figure out how we want to report this.
          continue;
        }
        log_.debug(fmt::format("reg {:03x} has value", pair.first));
        switch (static_cast<moteus::Register>(pair.first)) {
          case moteus::kMode: {
            const auto mode = ReadCast<int32_t>(pair.second);
            this_status.torque_on = mode >= 5;
            break;
          }
          case moteus::kPosition: {
            this_status.angle_deg = ReadCast<float>(pair.second) * 360.0;
            break;
          }
          case moteus::kTemperature: {
            this_status.temperature_C = ReadCast<float>(pair.second);
            break;
          }
          case moteus::kVoltage: {
            this_status.voltage = ReadCast<float>(pair.second);
            break;
          }
          case moteus::kVelocity: {
            this_status.velocity_dps = ReadCast<float>(pair.second) * 360.0;
            break;
          }
          case moteus::kTorque: {
            this_status.torque_Nm = ReadCast<float>(pair.second);
            break;
          }
          case moteus::kFault: {
            this_status.error = ReadCast<int32_t>(pair.second);
            break;
          }
          default: {
            break;
          }
        }
      }
      result.push_back(this_status);
    }

    service_.post(std::bind(handler, ec, result));
  }

  base::LogRef log_ = base::GetLogInstance("MoteusServo");

  boost::signals2::signal<void (const Command*)> command_signal_;

  boost::asio::io_service& service_;

  Parameters parameters_;
  boost::program_options::options_description options_;

  bool outstanding_ = false;
  mp::ThreadedClient::Request request_;
  mp::ThreadedClient::Request read_request_;
  mp::ThreadedClient::Reply read_reply_;

  mjlib::multiplex::ThreadedClient* mp_client_ = nullptr;
};

MoteusServo::MoteusServo(boost::asio::io_service& service,
                         base::TelemetryRegistry* telemetry_registry)
    : impl_(std::make_unique<Impl>(service, telemetry_registry)) {}
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

void MoteusServo::GetPose(const std::vector<int>&, PoseHandler) {
  mjlib::base::AssertNotReached();
}

void MoteusServo::GetTemperature(const std::vector<int>&, TemperatureHandler) {
  mjlib::base::AssertNotReached();
}

void MoteusServo::GetVoltage(const std::vector<int>&, VoltageHandler) {
  mjlib::base::AssertNotReached();
}

void MoteusServo::GetStatus(const std::vector<int>& ids,
                            const StatusOptions& status_options,
                            StatusHandler handler) {
  impl_->GetStatus(ids, status_options, handler);
}

void MoteusServo::ClearErrors(const std::vector<int>&,
                              mjlib::io::ErrorCallback callback) {
  impl_->service_.post(std::bind(callback, mjlib::base::error_code()));
}

}
}
