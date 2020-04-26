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

#include <functional>
#include <memory>
#include <string>

#include <clipp/clipp.h>

#include <boost/noncopyable.hpp>

#include "mjlib/base/visitor.h"

#include "base/context.h"

#include "mech/control_timing.h"
#include "mech/imu_client.h"
#include "mech/multiplex_client.h"

namespace mjmech {
namespace mech {

class TurretControl : boost::noncopyable {
 public:
  /// @param client_getter will be called at AsyncStart time
  using ClientGetter = std::function<mjlib::multiplex::AsioClient*()>;
  using ImuGetter = std::function<ImuClient*()>;
  TurretControl(base::Context&,
                ClientGetter client_getter, ImuGetter imu_getter);
  ~TurretControl();

  void AsyncStart(mjlib::io::ErrorCallback handler);

  struct Parameters {
    double period_s = 0.01;
    double max_torque_Nm = -1.0;
    std::string config;

    double command_timeout_s = 1.0;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(period_s));
      a->Visit(MJ_NVP(max_torque_Nm));
      a->Visit(MJ_NVP(config));
      a->Visit(MJ_NVP(command_timeout_s));
    }
  };

  struct Status {
    boost::posix_time::ptime timestamp;

    boost::posix_time::ptime mode_start;
    std::string fault;

    int missing_replies = 0;
    ControlTiming::Status timing;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(timestamp));
      a->Visit(MJ_NVP(mode_start));
      a->Visit(MJ_NVP(fault));
      a->Visit(MJ_NVP(missing_replies));
      a->Visit(MJ_NVP(timing));
    }
  };

  const Status& status() const;

  clipp::group program_options();

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};


}
}
