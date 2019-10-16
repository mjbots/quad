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

#pragma once

#include <memory>

#include <boost/noncopyable.hpp>
#include <boost/program_options.hpp>

#include "base/component_archives.h"
#include "base/context.h"
#include "mech/multiplex_client.h"
#include "mech/quadruped_control.h"

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

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(multiplex_client));
      a->Visit(MJ_NVP(quadruped_control));
    }
  };

  struct Parameters {
    base::ComponentParameters<Members> children;

    template <typename Archive>
    void Serialize(Archive* a) {
      children.Serialize(a);
    }

    Parameters(Members* m) : children(m) {}
  };

  Parameters* parameters();
  boost::program_options::options_description* options();

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}
}
