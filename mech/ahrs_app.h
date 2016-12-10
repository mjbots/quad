// Copyright 2015-2016 Josh Pieper, jjp@pobox.com.  All rights reserved.
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

#include "base/component_archives.h"
#include "base/program_options_archive.h"

#include "mjmech_imu_driver.h"
#include "ahrs.h"

namespace mjmech {
namespace mech {
class AhrsApp : boost::noncopyable {
 public:
  template <typename Context>
  AhrsApp(Context& context) {
    m_.imu.reset(new MjmechImuDriver(context));
    m_.ahrs.reset(new Ahrs(context, m_.imu->imu_data_signal()));

    base::MergeProgramOptions(m_.imu->options(), "imu.", &options_);
    base::MergeProgramOptions(m_.ahrs->options(), "ahrs.", &options_);
  }

  void AsyncStart(base::ErrorHandler handler) {
    parameters_.children.Start(handler);
  }

  struct Members {
    std::unique_ptr<MjmechImuDriver> imu;
    std::unique_ptr<Ahrs> ahrs;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(imu));
      a->Visit(MJ_NVP(ahrs));
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

  boost::program_options::options_description* options() { return &options_; }

 private:
  Members m_;
  Parameters parameters_{&m_};
  boost::program_options::options_description options_;
};
}
}
