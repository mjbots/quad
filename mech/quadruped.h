// Copyright 2019-2020 Josh Pieper, jjp@pobox.com.  All rights reserved.
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

#include <boost/noncopyable.hpp>

#include "base/component_archives.h"
#include "base/context.h"

#include "mech/multiplex_client.h"
#include "mech/quadruped_control.h"
#include "mech/web_control.h"

namespace mjmech {
namespace mech {

class Quadruped : boost::noncopyable {
 public:
  Quadruped(base::Context& context);
  ~Quadruped();

  void AsyncStart(mjlib::io::ErrorCallback);

  struct Members {
    std::unique_ptr<MultiplexClient> multiplex_client;
    std::unique_ptr<QuadrupedControl> quadruped_control;
    std::unique_ptr<WebControl> web_control;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(multiplex_client));
      a->Visit(MJ_NVP(quadruped_control));
      a->Visit(MJ_NVP(web_control));
    }
  };

  struct Parameters {
    template <typename Archive>
    void Serialize(Archive* a) {
    }
  };

  clipp::group program_options();

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}
}
