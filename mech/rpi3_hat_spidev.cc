// Copyright 2019-2020 Josh Pieper, jjp@pobox.com.
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

#include "mech/rpi3_hat_spidev.h"

#include <boost/asio/post.hpp>

#include "mjlib/base/fail.h"

namespace mjmech {
namespace mech {

class Rpi3HatSpidev::Impl {
 public:
  Impl(const boost::asio::executor& executor,
       const Options& options)
      : executor_(executor),
        options_(options) {}

  void AsyncStart(mjlib::io::ErrorCallback callback) {
    boost::asio::post(
        executor_,
        std::bind(std::move(callback), mjlib::base::error_code()));
  }

  void AsyncRegister(const IdRequest& request,
                     SingleReply* reply,
                     mjlib::io::ErrorCallback callback) {
    mjlib::base::AssertNotReached();
  }

  void AsyncRegisterMultiple(
      const std::vector<IdRequest>& request,
      Reply* reply,
      mjlib::io::ErrorCallback) {
    mjlib::base::AssertNotReached();
  }

  mjlib::io::SharedStream MakeTunnel(
      uint8_t id,
      uint32_t channel,
      const TunnelOptions& options) {
    mjlib::base::AssertNotReached();
  }

  boost::asio::executor executor_;
  const Options options_;
};

Rpi3HatSpidev::Rpi3HatSpidev(const boost::asio::executor& executor,
                             const Options& options)
    : impl_(std::make_unique<Impl>(executor, options)) {}

Rpi3HatSpidev::~Rpi3HatSpidev() {}

void Rpi3HatSpidev::AsyncStart(mjlib::io::ErrorCallback callback) {
  impl_->AsyncStart(std::move(callback));
}

void Rpi3HatSpidev::AsyncRegister(const IdRequest& request,
                                  SingleReply* reply,
                                  mjlib::io::ErrorCallback callback) {
  impl_->AsyncRegister(request, reply, std::move(callback));
}

void Rpi3HatSpidev::AsyncRegisterMultiple(
    const std::vector<IdRequest>& request,
    Reply* reply,
    mjlib::io::ErrorCallback callback) {
  impl_->AsyncRegisterMultiple(request, reply, std::move(callback));
}

mjlib::io::SharedStream Rpi3HatSpidev::MakeTunnel(
    uint8_t id,
    uint32_t channel,
    const TunnelOptions& options) {
  return impl_->MakeTunnel(id, channel, options);
}

}
}
