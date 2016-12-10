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

#include "mech_defines.h"
#include "servo_monitor.h"

namespace mjmech {
namespace mech {
class ServoMonitorApp : boost::noncopyable {
 public:
  template <typename Context>
  ServoMonitorApp(Context& context) {
    m_.servo_base.reset(new Mech::ServoBase(
                            context.service, *context.factory));
    m_.servo_iface.reset(
        new ServoMonitor::HerkuleXServoConcrete<Mech::ServoBase>(
            m_.servo_base.get()));
    m_.servo_monitor.reset(new ServoMonitor(context, m_.servo_iface.get()));

    base::ProgramOptionsArchive(&options_).Accept(&parameters_);
  }

  void AsyncStart(base::ErrorHandler handler) {
    parameters_.children.Start(handler);
  }

  struct Members {
    std::unique_ptr<Mech::ServoBase> servo_base;
    std::unique_ptr<ServoMonitor::HerkuleXServo> servo_iface;
    std::unique_ptr<ServoMonitor> servo_monitor;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(servo_base));
      a->Visit(MJ_NVP(servo_monitor));
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

  Parameters* parameters() { return &parameters_; }
  boost::program_options::options_description* options() { return &options_; }

 private:
  Members m_;
  Parameters parameters_{&m_};
  boost::program_options::options_description options_;
};
}
}
