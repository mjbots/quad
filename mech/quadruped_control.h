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

#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <boost/noncopyable.hpp>
#include <boost/program_options.hpp>

#include "mjlib/base/visitor.h"

#include "base/context.h"

#include "mech/moteus_servo.h"
#include "mech/multiplex_client.h"
#include "mech/quadruped_command.h"
#include "mech/quadruped_state.h"

namespace mjmech {
namespace mech {

/// This sequences the primary control modes of the quadruped.
class QuadrupedControl : boost::noncopyable {
 public:
  QuadrupedControl(base::Context&);
  ~QuadrupedControl();

  void AsyncStart(mjlib::io::ErrorCallback handler);
  void SetClient(MultiplexClient::Client*);

  struct Parameters {
    double period_s = 0.01;
    std::string config;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(period_s));
      a->Visit(MJ_NVP(config));
    }
  };

  struct Status {
    boost::posix_time::ptime timestamp;

    QuadrupedCommand::Mode mode = QuadrupedCommand::Mode::kStopped;
    boost::posix_time::ptime mode_start;
    std::string fault;

    QuadrupedState state;

    double time_status_s;
    double time_control_s;
    double time_command_s;
    double time_cycle_s;
    double time_delta_s;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(timestamp));
      a->Visit(MJ_ENUM(mode, QuadrupedCommand::ModeMapper));
      a->Visit(MJ_NVP(mode_start));
      a->Visit(MJ_NVP(fault));
      a->Visit(MJ_NVP(state));
      a->Visit(MJ_NVP(time_status_s));
      a->Visit(MJ_NVP(time_control_s));
      a->Visit(MJ_NVP(time_command_s));
      a->Visit(MJ_NVP(time_cycle_s));
      a->Visit(MJ_NVP(time_delta_s));
    }
  };

  void Command(const QuadrupedCommand&);
  const Status& status() const;

  Parameters* parameters();
  boost::program_options::options_description* options();

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}
}
