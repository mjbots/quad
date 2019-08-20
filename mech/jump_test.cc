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

#include "mech/jump_test.h"

#include <boost/asio/io_service.hpp>

namespace mjmech {
namespace mech {

class JumpTest::Impl {
 public:
  Impl(base::Context& context) : service_(context.service) {}

  boost::asio::io_service& service_;
  boost::program_options::options_description options_;
};

JumpTest::JumpTest(base::Context& context)
    : impl_(std::make_unique<Impl>(context)) {}
JumpTest::~JumpTest() {}

void JumpTest::AsyncStart(mjlib::io::ErrorCallback callback) {
  impl_->service_.post(std::bind(callback, mjlib::base::error_code()));
}

boost::program_options::options_description* JumpTest::options() {
  return &impl_->options_;
}

}
}
