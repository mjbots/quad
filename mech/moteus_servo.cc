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

#include <fmt/format.h>

#include "mjlib/base/fail.h"
#include "mjlib/base/program_options_archive.h"
#include "mjlib/multiplex/asio_client.h"

#include "base/logging.h"
#include "mech/moteus.h"

namespace pl = std::placeholders;

namespace mjmech {
namespace mech {

using Value = mjlib::multiplex::Format::Value;

enum RegisterTypes {
  kInt32 = 2,
  kFloat = 3,
};

template <typename T>
T ReadCast(const mjlib::multiplex::Format::ReadResult& value) {
  return std::visit([](auto a) {
      return static_cast<T>(a);
    }, std::get<Value>(value));
}

class MoteusServo::Impl {
 public:
  Impl(boost::asio::io_service& service)
      : service_(service) {
    mjlib::base::ProgramOptionsArchive(&options_).Accept(&parameters_);
  }

  void AsyncStart(mjlib::io::ErrorCallback handler) {
    service_.post(std::bind(handler, mjlib::base::error_code()));
  }

  void SetPose(const std::vector<Joint>& joints,
               mjlib::io::ErrorCallback handler) {
    if (!mp_client_) { return; }
    if (outstanding_) {
      log_.debug("skipping SetPose because we are backed up");
      return;
    }

    if (joints.empty()) {
      service_.post(std::bind(handler, mjlib::base::error_code()));
      return;
    }

    request_ = {};
    request_.WriteSingle(moteus::kMode,
                         static_cast<int8_t>(moteus::Mode::kPosition));
    const auto& joint = joints.back();

    // TODO(jpieper): MAJOR HACK.  This is not the right abstraction
    // layer for this.
    const bool is_shoulder = ((joint.address - 1) % 3) == 2;

    request_.WriteMultiple(
        moteus::kCommandPosition,
        {
          static_cast<float>(joint.angle_deg / 360.0),
          static_cast<float>(joint.velocity_dps / 360.0f),
          static_cast<float>(
              joint.power * (is_shoulder ?
                             parameters_.max_torque_shoulder_Nm :
                             parameters_.max_torque_legs_Nm)),
              });

    if (std::isfinite(joint.goal_deg)) {
      request_.WriteSingle(
          moteus::kCommandStopPosition,
          static_cast<float>(joint.goal_deg / 360.0f));
    }

    if (joint.torque_Nm != 0.0) {
      request_.WriteSingle(
          moteus::kCommandFeedforwardTorque,
          static_cast<float>(joint.torque_Nm));
    }

    auto remainder = joints;
    remainder.pop_back();

    outstanding_ = true;
    mp_client_->AsyncRegister(
        joints.back().address, request_,
        std::bind(&Impl::HandleSetPose, this, pl::_1, pl::_2, remainder, handler));
  }

  void HandleSetPose(const mjlib::base::error_code& ec,
                     const mjlib::multiplex::RegisterReply&,
                     const std::vector<Joint>& remainder,
                     mjlib::io::ErrorCallback handler) {
    mjlib::base::FailIf(ec);
    outstanding_ = false;

    SetPose(remainder, handler);
  }

  void EnablePower(PowerState power_state,
                   const std::vector<int>& ids,
                   mjlib::io::ErrorCallback handler) {
    if (!mp_client_) { return; }

    if (ids.empty()) {
      service_.post(std::bind(handler, mjlib::base::error_code()));
      return;
    }

    request_ = {};
    const auto value = [&]() {
      switch (power_state) {
        case PowerState::kPowerFree: return moteus::Mode::kStopped;
        case PowerState::kPowerBrake: return moteus::Mode::kStopped;
        case PowerState::kPowerEnable: return moteus::Mode::kPosition;
      }
    }();
    request_.WriteSingle(moteus::kMode, static_cast<int8_t>(value));

    auto remainder = ids;
    remainder.pop_back();

    mp_client_->AsyncRegister(
        ids.back(), request_,
        std::bind(&Impl::HandleEnablePower, this, pl::_1, pl::_2,
                  power_state, remainder, handler));
  }

  void HandleEnablePower(const mjlib::base::error_code& ec,
                         const mjlib::multiplex::RegisterReply&,
                         PowerState power_state,
                         const std::vector<int>& ids,
                         mjlib::io::ErrorCallback handler) {
    mjlib::base::FailIf(ec);

    EnablePower(power_state, ids, handler);
  }

  void GetStatus(const std::vector<int>& ids,
                 const StatusOptions& status_options,
                 StatusHandler handler) {
    StartNextStatusRequest(ids, status_options, handler, {});
  }

  void StartNextStatusRequest(const std::vector<int>& ids,
                              const StatusOptions& status_options,
                              StatusHandler handler,
                              const std::vector<JointStatus>& current_result) {
    if (ids.empty()) {
      service_.post(std::bind(handler, mjlib::base::error_code(), current_result));
      return;
    }

    read_request_ = {};
    read_request_.ReadSingle(moteus::kMode, 0);
    if (status_options.pose) {
      read_request_.ReadSingle(moteus::kPosition, kFloat);
    }
    if (status_options.temperature) {
      read_request_.ReadSingle(moteus::kTemperature, kFloat);
    }
    if (status_options.voltage) {
      read_request_.ReadSingle(moteus::kVoltage, kFloat);
    }
    read_request_.ReadSingle(moteus::kFault, kInt32);

    auto remainder = ids;
    remainder.pop_back();

    auto this_id = ids.back();

    mp_client_->AsyncRegister(
        this_id, read_request_,
        std::bind(&Impl::HandleStatus, this, pl::_1, pl::_2,
                  this_id, remainder, status_options, handler, current_result));
  }

  void HandleStatus(const mjlib::base::error_code& ec,
                    const mjlib::multiplex::RegisterReply& reply,
                    int this_id,
                    const std::vector<int>& remainder,
                    const StatusOptions& status_options,
                    StatusHandler handler,
                    std::vector<JointStatus> current_result) {
    if (ec) {
      log_.debug(fmt::format("reply with error: {}", ec.message()));
      service_.post(std::bind(handler, ec, current_result));
      return;
    }

    JointStatus this_status;
    this_status.address = this_id;

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
        case moteus::kFault: {
          this_status.error = ReadCast<int32_t>(pair.second);
          break;
        }
        default: {
          break;
        }
      }
    }

    current_result.push_back(this_status);

    StartNextStatusRequest(remainder, status_options, handler, current_result);
  }

  base::LogRef log_ = base::GetLogInstance("MoteusServo");

  boost::asio::io_service& service_;

  Parameters parameters_;
  boost::program_options::options_description options_;

  bool outstanding_ = false;
  mjlib::multiplex::RegisterRequest request_;
  mjlib::multiplex::RegisterRequest read_request_;

  mjlib::multiplex::AsioClient* mp_client_ = nullptr;
};

MoteusServo::MoteusServo(boost::asio::io_service& service)
    : impl_(std::make_unique<Impl>(service)) {}
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

void MoteusServo::SetClient(mjlib::multiplex::AsioClient* client) {
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
