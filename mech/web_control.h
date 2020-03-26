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

#pragma once

#include <memory>

#include <clipp/clipp.h>

#include <boost/asio/executor.hpp>

#include "mech/quadruped_control.h"

namespace mjmech {
namespace mech {

/// Exposes an embedded web server with a command and control UI.
class WebControl {
 public:
  WebControl(const boost::asio::executor&, QuadrupedControl* control);
  ~WebControl();

  struct Parameters {
    int port = 4778;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(port));
    }
  };

  clipp::group program_options();
  void AsyncStart(mjlib::io::ErrorCallback);

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}
}
