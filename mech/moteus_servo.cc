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

#include "mjlib/base/fail.h"
#include "mjlib/base/program_options_archive.h"
#include "mjlib/multiplex/asio_client.h"

#include "base/logging.h"
#include "mech/moteus.h"

namespace pl = std::placeholders;

namespace mjmech {
namespace mech {

class MoteusServo::Impl {
 public:
  Impl(boost::asio::io_service& service,
       mjlib::io::StreamFactory& factory)
      : service_(service),
        factory_(factory) {
    mjlib::base::ProgramOptionsArchive(&options_).Accept(&stream_parameters_);
    mjlib::base::ProgramOptionsArchive(&options_).Accept(&parameters_);
  }

  void AsyncStart(mjlib::io::ErrorCallback handler) {
    factory_.AsyncCreate(
        stream_parameters_,
        std::bind(&Impl::HandleCreate, this, pl::_1, pl::_2, handler));
  }

  void HandleCreate(const mjlib::base::error_code& ec,
                    mjlib::io::SharedStream stream,
                    mjlib::io::ErrorCallback handler) {
    if (!ec) {
      stream_ = stream;
      mp_client_ = std::make_unique<mjlib::multiplex::AsioClient>(stream_.get());
    }
    service_.post(std::bind(handler, ec));
  }

  void SetPose(const std::vector<Joint>& joints,
               mjlib::io::ErrorCallback handler) {
    BOOST_ASSERT(!!mp_client_);
    BOOST_ASSERT(!outstanding_);

    if (joints.empty()) {
      service_.post(std::bind(handler, mjlib::base::error_code()));
      return;
    }

    request_ = {};
    request_.WriteSingle(moteus::kMode,
                         static_cast<int8_t>(moteus::Mode::kPosition));
    request_.WriteMultiple(
        moteus::kCommandPosition,
        {
          static_cast<float>(joints.back().angle_deg / 360.0),
          static_cast<float>(joints.back().velocity_dps / 360.0f),
          static_cast<float>(parameters_.max_current),
          static_cast<float>(joints.back().goal_deg / 360.0f),
        });

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

  base::LogRef log_ = base::GetLogInstance("MoteusServo");

  boost::asio::io_service& service_;
  mjlib::io::StreamFactory& factory_;
  mjlib::io::StreamFactory::Options stream_parameters_;

  Parameters parameters_;
  boost::program_options::options_description options_;

  mjlib::io::SharedStream stream_;
  std::unique_ptr<mjlib::multiplex::AsioClient> mp_client_;

  bool outstanding_ = false;
  mjlib::multiplex::RegisterRequest request_;
};

MoteusServo::MoteusServo(boost::asio::io_service& service,
                         mjlib::io::StreamFactory& factory)
    : impl_(std::make_unique<Impl>(service, factory)) {}
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

}
}
