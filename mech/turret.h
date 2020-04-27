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

#pragma once

#include <memory>

#include <clipp/clipp.h>

#include <boost/noncopyable.hpp>

#include "mjlib/io/selector.h"
#include "mjlib/multiplex/asio_client.h"

#include "base/component_archives.h"
#include "base/context.h"

#include "mech/aux_stm32.h"
#include "mech/turret_control.h"
#include "mech/turret_rf_control.h"
#include "mech/web_control.h"

namespace mjmech {
namespace mech {

class Turret : boost::noncopyable {
 public:
  Turret(base::Context&);
  ~Turret();

  void AsyncStart(mjlib::io::ErrorCallback);

  using TurretWebControl =
      WebControl<TurretControl::CommandData, TurretControl::Status>;

  struct Members {
    std::unique_ptr<
      mjlib::io::Selector<mjlib::multiplex::AsioClient>> multiplex_client;
    std::unique_ptr<mjlib::io::Selector<AuxStm32>> imu_client;
    std::unique_ptr<TurretControl> turret_control;
    std::unique_ptr<TurretWebControl> web_control;
    std::unique_ptr<TurretRfControl> rf_control;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(multiplex_client));
      a->Visit(MJ_NVP(imu_client));
      a->Visit(MJ_NVP(turret_control));
      a->Visit(MJ_NVP(web_control));
      a->Visit(MJ_NVP(rf_control));
    }
  };

  Members* m();

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
