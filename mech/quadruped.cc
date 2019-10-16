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

#include "mech/quadruped.h"

#include "mjlib/base/program_options_archive.h"

namespace mjmech {
namespace mech {

class Quadruped::Impl {
 public:
  Impl(base::Context& context)
      : executor_(context.executor) {
    m_.multiplex_client = std::make_unique<MultiplexClient>(executor_);
    m_.quadruped_control = std::make_unique<QuadrupedControl>(context);

    m_.multiplex_client->RequestClient([this](const auto& ec, auto* client) {
        mjlib::base::FailIf(ec);
        m_.quadruped_control->SetClient(client);
      });

    mjlib::base::ProgramOptionsArchive(&options_).Accept(&p_);
  }

  void AsyncStart(mjlib::io::ErrorCallback callback) {
    p_.children.Start(callback);
  }

  boost::asio::executor executor_;
  Members m_;
  Parameters p_{&m_};
  boost::program_options::options_description options_;
};

Quadruped::Quadruped(base::Context& context)
    : impl_(std::make_unique<Impl>(context)) {}

Quadruped::~Quadruped() {}

void Quadruped::AsyncStart(mjlib::io::ErrorCallback callback) {
  impl_->AsyncStart(callback);
}

Quadruped::Parameters* Quadruped::parameters() {
  return &impl_->p_;
}

boost::program_options::options_description* Quadruped::options() {
  return &impl_->options_;
}

}
}
