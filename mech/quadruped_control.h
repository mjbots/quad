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

#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <boost/noncopyable.hpp>

#include "mjlib/base/visitor.h"

#include "base/context.h"

#include "mech/control_timing.h"
#include "mech/pi3hat_interface.h"
#include "mech/quadruped_command.h"
#include "mech/quadruped_state.h"

namespace mjmech {
namespace mech {

/// This sequences the primary control modes of the quadruped.
class QuadrupedControl : boost::noncopyable {
 public:
  /// @param client_getter will be called at AsyncStart time
  using Pi3hatGetter = std::function<Pi3hatInterface*()>;
  QuadrupedControl(base::Context&, Pi3hatGetter client_getter);
  ~QuadrupedControl();

  void AsyncStart(mjlib::io::ErrorCallback handler);

  struct Parameters {
    double max_torque_Nm = -1.0;
    std::string config;

    bool enable_imu = true;

    double command_timeout_s = 1.0;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(max_torque_Nm));
      a->Visit(MJ_NVP(config));
      a->Visit(MJ_NVP(enable_imu));
      a->Visit(MJ_NVP(command_timeout_s));
    }
  };

  struct Status {
    boost::posix_time::ptime timestamp;

    QuadrupedCommand::Mode mode = QuadrupedCommand::Mode::kConfiguring;
    boost::posix_time::ptime mode_start;
    std::string fault;

    QuadrupedState state;

    int missing_replies = 0;
    ControlTiming::Status timing;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(timestamp));
      a->Visit(MJ_NVP(mode));
      a->Visit(MJ_NVP(mode_start));
      a->Visit(MJ_NVP(fault));
      a->Visit(MJ_NVP(state));
      a->Visit(MJ_NVP(missing_replies));
      a->Visit(MJ_NVP(timing));
    }
  };

  struct ControlLog {
    using QC = QuadrupedCommand;

    boost::posix_time::ptime timestamp;
    std::vector<QC::Joint> joints;

    struct LegPD {
      base::Point3D cmd_N;
      base::Point3D gravity_N;
      base::Point3D accel_N;
      base::Point3D err_m;
      base::Point3D err_m_s;
      base::Point3D p_N;
      base::Point3D d_N;
      base::Point3D total_N;

      template <typename Archive>
      void Serialize(Archive* a) {
        a->Visit(MJ_NVP(cmd_N));
        a->Visit(MJ_NVP(gravity_N));
        a->Visit(MJ_NVP(accel_N));
        a->Visit(MJ_NVP(err_m));
        a->Visit(MJ_NVP(err_m_s));
        a->Visit(MJ_NVP(p_N));
        a->Visit(MJ_NVP(d_N));
        a->Visit(MJ_NVP(total_N));
      }
    };
    std::vector<LegPD> leg_pds;

    std::vector<QC::Leg> legs_B;
    std::vector<QC::Leg> legs_R;
    base::KinematicRelation desired_RB;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(timestamp));
      a->Visit(MJ_NVP(joints));
      a->Visit(MJ_NVP(leg_pds));
      a->Visit(MJ_NVP(legs_B));
      a->Visit(MJ_NVP(legs_R));
      a->Visit(MJ_NVP(desired_RB));
    }
  };

  void Command(const QuadrupedCommand&);
  const Status& status() const;

  clipp::group program_options();

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}
}
