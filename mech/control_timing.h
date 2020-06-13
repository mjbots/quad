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

#include <boost/asio/executor.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>

#include "mjlib/base/time_conversions.h"
#include "mjlib/base/visitor.h"
#include "mjlib/io/now.h"

namespace mjmech {
namespace mech {

class ControlTiming {
 public:
  ControlTiming(const boost::asio::executor& executor,
                boost::posix_time::ptime last_cycle_start)
      : executor_(executor) {
    timestamps_.last_cycle_start = last_cycle_start;
    timestamps_.cycle_start = Now();
    timestamps_.delta_s = mjlib::base::ConvertDurationToSeconds(
        timestamps_.cycle_start - timestamps_.last_cycle_start);
  }

  struct Status {
    double query_s = 0.0;
    double status_s = 0.0;
    double control_s = 0.0;
    double command_s = 0.0;
    double cycle_s = 0.0;
    double delta_s = 0.0;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(query_s));
      a->Visit(MJ_NVP(status_s));
      a->Visit(MJ_NVP(control_s));
      a->Visit(MJ_NVP(command_s));
      a->Visit(MJ_NVP(cycle_s));
      a->Visit(MJ_NVP(delta_s));
    }
  };

  Status status() const {
    Status result;

    result.query_s = mjlib::base::ConvertDurationToSeconds(
        timestamps_.query_done - timestamps_.cycle_start);
    result.status_s = mjlib::base::ConvertDurationToSeconds(
        timestamps_.status_done - timestamps_.query_done);
    result.control_s = mjlib::base::ConvertDurationToSeconds(
        timestamps_.control_done - timestamps_.status_done);
    result.command_s = mjlib::base::ConvertDurationToSeconds(
        timestamps_.command_done - timestamps_.control_done);
    result.cycle_s = mjlib::base::ConvertDurationToSeconds(
        timestamps_.command_done - timestamps_.cycle_start);
    result.delta_s = timestamps_.delta_s;

    return result;
  }

  boost::posix_time::ptime cycle_start() const { return timestamps_.cycle_start; }

  void finish_query() { timestamps_.query_done = Now(); }
  void finish_status() { timestamps_.status_done = Now(); }
  void finish_control() { timestamps_.control_done = Now(); }
  void finish_command() { timestamps_.command_done = Now(); }

 private:
  struct Timestamps {
    boost::posix_time::ptime last_cycle_start;
    double delta_s = 0.0;

    boost::posix_time::ptime cycle_start;
    boost::posix_time::ptime query_done;
    boost::posix_time::ptime status_done;
    boost::posix_time::ptime control_done;
    boost::posix_time::ptime command_done;
  };

  boost::posix_time::ptime Now() const {
    return mjlib::io::Now(executor_.context());
  }

  boost::asio::executor executor_;
  Timestamps timestamps_;
};

}
}
