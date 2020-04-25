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

namespace mjmech {
namespace mech {

class TurretControl::Impl {
 public:
  Impl(base::Context& context,
       ClientGetter client_getter,
       ImuGetter imu_getter)
      : executor_(context.executor),
        client_getter_(client_getter),
        imu_getter_(imu_getter) {
  }

  void AsyncStart(mjlib::io::ErrorCallback callback) {
    boost::asio::post(
        executor_,
        std::bind(std::move(callback), mjlib::base::error_code()));
  }

  boost::asio::executor executor_;
  ClientGetter client_getter_;
  ImuGetter imu_getter_;

  Status status_;
  Parameters parameters_;
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

clipp::group TurretControl::program_options() {
  return mjlib::base::ClippArchive().Accept(&impl_->parameters_).release();
}


}
}
