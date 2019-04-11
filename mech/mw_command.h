// Copyright 2015-2016 Mikhail Afanasyev.  All rights reserved.
// Copyright 2019 Josh Pieper, jjp@pobox.com.
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

#include "mjlib/base/fail.h"
#include "mjlib/base/program_options_archive.h"

#include "base/component_archives.h"
#include "base/logging.h"

#include "mech_warfare_command.h"
#include "video_controller_app.h"

namespace mjmech {
namespace mech {

// this is a class for the main mw_command binary
class MWCommand : boost::noncopyable {
 public:
  template <typename Context>
  MWCommand(Context& context)
    : service_(context.service) {
    m_.video_controller.reset(new VideoControllerApp(context));
    m_.commander.reset(new mw_command::Commander(service_));

    mjlib::base::ProgramOptionsArchive(&options_).Accept(&parameters_);
  }

  void AsyncStart(mjlib::io::ErrorCallback handler) {
    std::shared_ptr<base::ErrorHandlerJoiner> joiner =
        std::make_shared<base::ErrorHandlerJoiner>(
            [=](mjlib::base::error_code ec) {
              if (!ec) {
                service_.post([=]() {MaybeSendOnce(); });
              }
              handler(ec);
            });

    m_.video_controller->AsyncStart(joiner->Wrap("starting video_controller"));
    m_.commander->AsyncStart(joiner->Wrap("starting commander"));

    m_.commander->target_offset_signal()->connect([this](int x, int y) {
        m_.video_controller->SetTargetOffset(x, y);
      });
  }

  struct Members {
    std::unique_ptr<VideoControllerApp> video_controller;
    std::unique_ptr<mw_command::Commander> commander;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(video_controller));
      a->Visit(MJ_NVP(commander));
    }
  };

  struct Parameters {
    // We are not using ComponentParameters because we do not want an extra
    // level of indirection in command line options.
    Members* members = nullptr;

    // If true, sends a command once and immediately exits.
    bool send_once = false;
    double turret_pitch_rate_dps = 0.0;
    double turret_yaw_rate_dps = 0.0;


    Parameters(Members& m) : members(&m) {};

    template <typename Archive>
    void Serialize(Archive* a) {
      members->video_controller->parameters()->Serialize(a);
      members->commander->parameters()->Serialize(a);

      a->Visit(MJ_NVP(send_once));
      a->Visit(MJ_NVP(turret_pitch_rate_dps));
      a->Visit(MJ_NVP(turret_yaw_rate_dps));

    }
  };

  Parameters* parameters() { return &parameters_; }
  boost::program_options::options_description* options() { return &options_; }

 private:

  void MaybeSendOnce() {
    bool turret_set = (
        parameters_.turret_pitch_rate_dps != 0.0 ||
        parameters_.turret_yaw_rate_dps != 0.0);

    if (!parameters_.send_once) {
      if (turret_set) {
        mjlib::base::Fail("turret_* options have no effect when send_once=False");
      }
      return;
    }

    mw_command::MechMessage message;
    message.gait = m_.commander->parameters()->cmd;
    TurretCommand& turret = message.turret;
    if (turret_set) {
      turret.rate = TurretCommand::Rate();
      turret.rate->x_deg_s = parameters_.turret_yaw_rate_dps;
      turret.rate->y_deg_s = parameters_.turret_pitch_rate_dps;
    }
    m_.commander->SendMechMessage(message);
    service_.stop();
    log_.info("message sent, exiting");
  };

  boost::asio::io_service& service_;
  Members m_;
  Parameters parameters_ = Parameters(m_);
  boost::program_options::options_description options_;
  base::LogRef log_ = base::GetLogInstance("mw_command");
};

}
}
