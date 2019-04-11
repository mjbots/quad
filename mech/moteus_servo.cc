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

#include "mjlib/base/program_options_archive.h"
#include "mjlib/multiplex/asio_client.h"

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

  boost::asio::io_service& service_;
  mjlib::io::StreamFactory& factory_;
  mjlib::io::StreamFactory::Options stream_parameters_;

  Parameters parameters_;
  boost::program_options::options_description options_;

  mjlib::io::SharedStream stream_;
  std::unique_ptr<mjlib::multiplex::AsioClient> mp_client_;
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

void MoteusServo::SetPose(const std::vector<Joint>&, mjlib::io::ErrorCallback) {
}

void MoteusServo::EnablePower(PowerState, const std::vector<int>&,
                              mjlib::io::ErrorCallback) {
}

void MoteusServo::GetPose(const std::vector<int>&, PoseHandler) {
}

void MoteusServo::GetTemperature(const std::vector<int>&, TemperatureHandler) {
}

void MoteusServo::GetVoltage(const std::vector<int>&, VoltageHandler) {
}

}
}
