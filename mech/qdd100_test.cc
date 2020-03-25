// Copyright 2020 Josh Pieper, jjp@pobox.com.  All rights reserved.
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

#include <linux/input.h>

#include <functional>

#include <fmt/format.h>

#include <boost/asio/executor.hpp>
#include <boost/asio/io_context.hpp>

#include "mjlib/base/clipp_archive.h"
#include "mjlib/base/clipp.h"
#include "mjlib/base/fail.h"
#include "mjlib/base/time_conversions.h"
#include "mjlib/io/repeating_timer.h"
#include "mjlib/multiplex/stream_asio_client_builder.h"

#include "base/linux_input.h"
#include "mech/moteus.h"

namespace pl = std::placeholders;

namespace mjmech {
namespace mech {

namespace {
using Value = mjlib::multiplex::Format::Value;

struct Options {
  std::string joystick;
  mjlib::multiplex::StreamAsioClientBuilder::Options stream;
  double period_s = 0.05;
  double max_torque_Nm = 1.0;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(joystick));
    a->Visit(MJ_NVP(stream));
    a->Visit(MJ_NVP(period_s));
    a->Visit(MJ_NVP(max_torque_Nm));
  }
};

class Application {
 public:
  Application(const boost::asio::executor& executor,
              const Options& options)
      : executor_(executor),
        options_(options),
        timer_(executor),
        linux_input_(executor, options.joystick),
        asio_client_(executor, options.stream) {
  }

  void Start() {
    asio_client_.AsyncStart(std::bind(&Application::HandleStart, this, pl::_1));
  }

  void HandleStart(const mjlib::base::error_code& ec) {
    mjlib::base::FailIf(ec);

    // Start by sending a stop command to reset any state the servo
    // might have.
    Stop();

    timer_.start(mjlib::base::ConvertSecondsToDuration(options_.period_s),
                 std::bind(&Application::HandleTimer, this, pl::_1));

    StartRead();
    UpdateDisplay();
  }

  void Stop() {
    MJ_ASSERT(outstanding_ == false);

    id_request_ = {};
    id_request_.id = 1;
    id_request_.request.WriteSingle(
        moteus::kMode, Value(static_cast<int8_t>(moteus::Mode::kStopped)));

    outstanding_ = true;
    single_reply_ = {};
    asio_client_.AsyncRegister(
        id_request_, &single_reply_,
        std::bind(&Application::HandleStop, this, pl::_1));
  }

  void HandleStop(const mjlib::base::error_code& ec) {
    outstanding_ = false;
    mjlib::base::FailIf(ec);
  }

  void HandleTimer(const mjlib::base::error_code& ec) {
    if (ec == boost::asio::error::operation_aborted) { return; }
    mjlib::base::FailIf(ec);

    if (outstanding_) { return; }

    outstanding_ = true;

    const float command_Nm = GetCommandTorqueNm();

    id_request_ = {};
    id_request_.id = 1;
    id_request_.request.WriteSingle(
        moteus::kMode, Value(static_cast<int8_t>(moteus::Mode::kPosition)));
    id_request_.request.WriteMultiple(
        moteus::kCommandPosition,
        {Value(0.0f),  // position
              Value(0.0f),  // velocity
              Value(command_Nm),
              Value(0.0f),  // kp_scale
              Value(0.0f),  // kd_scale
              });

    id_request_.request.ReadMultiple(moteus::kMode, 5, moteus::kFloat);
    id_request_.request.ReadMultiple(moteus::kVoltage, 3, moteus::kFloat);

    current_reply_ = {};
    asio_client_.AsyncRegister(
        id_request_, &current_reply_,
        std::bind(&Application::HandleClient, this, pl::_1));
  }

  void StartRead() {
    linux_input_.AsyncRead(
        &event_, std::bind(
            &Application::HandleJoystick, this, pl::_1));
  }

  void HandleJoystick(const mjlib::base::error_code& ec) {
    mjlib::base::FailIf(ec);

    UpdateDisplay();

    StartRead();
  }

  void HandleClient(const mjlib::base::error_code& ec) {
    outstanding_ = false;

    if (ec == boost::asio::error::operation_aborted) { return; }
    mjlib::base::FailIf(ec);

    single_reply_ = current_reply_;

    UpdateDisplay();
  }

  double GetCommandTorqueNm() {
    return linux_input_.abs_info(ABS_Y).scaled() *
        options_.max_torque_Nm;
  }

  void UpdateDisplay() {
    double command = GetCommandTorqueNm();

    auto get = [&](auto moteus_reg) -> double {
      if (single_reply_.reply.count(moteus_reg) == 0) {
        return std::numeric_limits<double>::signaling_NaN();
      }

      return (std::get<float>(
                  std::get<Value>(single_reply_.reply.at(moteus_reg))));
    };

    std::cout << fmt::format(
        " cmd={:5.1f}Nm  qdd100[ mode={:4} "
        "pos={:7.1f}deg vel={:6.1f}dps torque={:5.1f}Nm i={:4.1f}A "
        "t={:3.0f}C ]  \r",
        command,
        MapMode(get(moteus::kMode)),
        get(moteus::kPosition) * 360.0,
        get(moteus::kVelocity) * 360.0,
        get(moteus::kTorque),
        get(moteus::kQCurrent),
        get(moteus::kTemperature));
    std::cout.flush();
  }

  std::string MapMode(float value) {
    if (!std::isfinite(value)) { return "unk"; }
    return mode_text_.at(static_cast<int>(value));
  }

  boost::asio::executor executor_;
  const Options options_;
  mjlib::io::RepeatingTimer timer_;
  base::LinuxInput linux_input_;
  mjlib::multiplex::StreamAsioClientBuilder asio_client_;

  base::LinuxInput::Event event_;

  using AsioClient = mjlib::multiplex::AsioClient;

  AsioClient::IdRequest id_request_;
  AsioClient::SingleReply current_reply_;
  AsioClient::SingleReply single_reply_;
  bool outstanding_ = false;

  const std::map<int, std::string> mode_text_ = {
    { 0, "stop" },
    { 1, "flt" },
    { 2, "" },
    { 3, "" },
    { 4, "" },
    { 5, "pwm" },
    { 6, "volt" },
    { 7, "foc" },
    { 8, "vdq" },
    { 9, "cur" },
    { 10, "pos" },
    { 11, "tmt" },
    { 12, "zero" },
  };
};

}

int do_main(int argc, char**argv) {
  Options options;

  auto group = mjlib::base::ClippArchive().Accept(&options).group();
  mjlib::base::ClippParse(argc, argv, group);

  boost::asio::io_context context;
  Application application(context.get_executor(), options);

  application.Start();

  context.run();
  return 0;
}

}
}

int main(int argc, char** argv) {
  return mjmech::mech::do_main(argc, argv);
}
