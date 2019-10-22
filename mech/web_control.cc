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

#include "mech/web_control.h"

#include "mjlib/base/program_options_archive.h"

namespace mjmech {
namespace mech {

class WebControl::Impl {
 public:
  Impl(boost::asio::io_context& context,
       QuadrupedControl* quadruped_control)
      : context_(context),
        quadruped_control_(quadruped_control) {
    mjlib::base::ProgramOptionsArchive(&options_).Accept(&parameters_);
  }

  void AsyncStart(mjlib::io::ErrorCallback callback) {
    boost::asio::post(
        context_,
        std::bind(callback, mjlib::base::error_code()));
  }

  boost::asio::io_context& context_;
  QuadrupedControl* const quadruped_control_;
  Parameters parameters_;
  boost::program_options::options_description options_;
};

WebControl::WebControl(boost::asio::io_context& context,
                       QuadrupedControl* quadruped_control)
    : impl_(std::make_unique<Impl>(context, quadruped_control)) {}

WebControl::~WebControl() {}

void WebControl::AsyncStart(mjlib::io::ErrorCallback callback) {
  impl_->AsyncStart(callback);
}

boost::program_options::options_description* WebControl::options() {
  return &impl_->options_;
}

}
}
