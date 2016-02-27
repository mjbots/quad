// Copyright 2015 Mikhail Afanasyev.  All rights reserved.
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
#include "base/fail.h"
#include "base/logging.h"

#include "mcast_video_link.h"
#include "mech_warfare_command.h"

namespace mjmech {
namespace mech {

// this is a class for the main mw_command binary
class MWCommand : boost::noncopyable {
 public:
  template <typename Context>
  MWCommand(Context& context)
    : service_(context.service) {
    m_.video_link.reset(new McastVideoLinkReceiver(context));
  }

  void AsyncStart(base::ErrorHandler handler) {
    std::shared_ptr<base::ErrorHandlerJoiner> joiner =
        std::make_shared<base::ErrorHandlerJoiner>(
            [=](base::ErrorCode ec) {
              if (!ec) {
                service_.post([=]() {MaybeSendOnce(); });
              }
              handler(ec);
            });

    parameters_.children.Start(joiner->Wrap("starting children"));
    commander_.AsyncStart(joiner->Wrap("starting commander"));
  }

  struct Members {
    std::unique_ptr<McastVideoLinkReceiver> video_link;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(video_link));
    }
  };

  struct Parameters {
    base::ComponentParameters<Members> children;
    mw_command::Commander::Parameters* mw_params;

    // If true, sends a command once and immediately exits.
    bool send_once = false;
    double turret_pitch_rate_dps = 0.0;
    double turret_yaw_rate_dps = 0.0;


    Parameters(base::ComponentParameters<Members> _children,
               mw_command::Commander::Parameters* _mw_params)
        : children(_children), mw_params(_mw_params) {};

    template <typename Archive>
    void Serialize(Archive* a) {
      children.Serialize(a);
      mw_params->Serialize(a);

      a->Visit(MJ_NVP(send_once));
      a->Visit(MJ_NVP(turret_pitch_rate_dps));
      a->Visit(MJ_NVP(turret_yaw_rate_dps));

    }
  };

  Parameters* parameters() { return &parameters_; }

 private:

  void MaybeSendOnce() {
    bool turret_set = (
        parameters_.turret_pitch_rate_dps != 0.0 ||
        parameters_.turret_yaw_rate_dps != 0.0);

    if (!parameters_.send_once) {
      if (turret_set) {
        base::Fail("turret_* options have no effect when send_once=False");
      }
      return;
    }

    mw_command::MechMessage message;
    message.gait = commander_.parameters()->cmd;
    TurretCommand& turret = message.turret;
    if (turret_set) {
      turret.rate = TurretCommand::Rate();
      turret.rate->x_deg_s = parameters_.turret_yaw_rate_dps;
      turret.rate->y_deg_s = parameters_.turret_pitch_rate_dps;
    }
    commander_.SendMechMessage(message);
    service_.stop();
    log_.info("message sent, exiting");
  };

  boost::asio::io_service& service_;
  Members m_;
  // This is not a member because we do not want an extra level of indirection
  // in command line options.
  mw_command::Commander commander_{service_};
  Parameters parameters_ = Parameters(&m_, commander_.parameters());
  base::LogRef log_ = base::GetLogInstance("mw_command");
};

}
}
