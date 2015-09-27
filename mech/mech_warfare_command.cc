// Copyright 2015 Josh Pieper, jjp@pobox.com.  All rights reserved.
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

#include <boost/asio.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/program_options.hpp>
#include <boost/property_tree/json_parser.hpp>

#include "base/fail.h"
#include "base/json_archive.h"
#include "base/linux_input.h"
#include "base/program_options_archive.h"
#include "base/property_tree_archive.h"

#include "gait.h"
#include "turret_command.h"

namespace {
using namespace mjmech::base;
using namespace mjmech::mech;
namespace pt = boost::property_tree;
typedef boost::asio::ip::udp udp;

struct AxisMapping {
  int translate_x = -1;
  int sign_translate_x = 1;
  int translate_y = -1;
  int sign_translate_y = 1;
  int rotate = -1;
  int sign_rotate = 1;
  int body_z = -1;
  int sign_body_z = 1;

  int body_pitch = -1;
  int sign_body_pitch = 1;
  int body_roll = -1;
  int sign_body_roll = 1;

  int body_x = -1;
  int sign_body_x = 1;
  int body_y = -1;
  int sign_body_y = 1;

  int turret_x = -1;
  int sign_turret_x = 1;

  int turret_y = -1;
  int sign_turret_y = 1;

  int fire = -1;
  int laser = -1;
  int agitator = -1;

  int crouch = -1;
  int body = -1;
  int turret = -1;
};

AxisMapping GetAxisMapping(const LinuxInput* input) {
  auto features = input->features(EV_ABS);

  // For now, we'll just hard-code the mapping used by jpieper's xbox
  // controller.  In the long term, or even medium term, this should
  // probably be configurable somehow.
  AxisMapping result;
  if (features.capabilities.test(ABS_X)) {
    result.translate_x = ABS_X;
    result.sign_translate_x = 1;

    result.body_x = ABS_X;
    result.sign_body_x = 1;
  }
  if (features.capabilities.test(ABS_Y)) {
    result.translate_y = ABS_Y;
    result.sign_translate_y = -1;

    result.body_y = ABS_Y;
    result.sign_body_y = -1;
  }

  if (features.capabilities.test(ABS_RX)) {
    result.rotate = ABS_RX;
    result.sign_rotate = 1;

    result.body_roll = ABS_RX;
    result.sign_body_roll = 1;

    result.turret_x = ABS_RX;
    result.sign_turret_x = -1;
  }
  if (features.capabilities.test(ABS_RY)) {
    result.body_z = ABS_RY;
    result.sign_body_z = -1;

    result.body_pitch = ABS_RY;
    result.sign_body_pitch = 1;

    result.turret_y = ABS_RY;
    result.sign_turret_y = 1;
  }

  result.crouch = BTN_A;
  result.body = BTN_TR;
  result.turret = BTN_TL;

  result.laser = BTN_WEST;
  result.fire = ABS_RZ;
  result.agitator = BTN_NORTH;

  return result;
}

struct Options {
  double period_s = 0.1;
  double deadband = 0.15;
  double max_translate_x_mm_s = 300.0;
  double max_translate_y_mm_s = 300.0;
  double max_rotate_deg_s = 100.0;
  double max_body_z_mm = 30.0;
  double min_body_z_mm = -60.0;
  double idle_body_y_mm = 5.0;
  double forward_body_y_mm = 15.0;
  double reverse_body_y_mm = -5.0;
  double max_body_x_mm = 30;
  double max_body_y_mm = 30;
  double max_body_pitch_deg = 20;
  double max_body_roll_deg = 20;
  double max_turret_rate_deg_s = 100;
  bool verbose = false;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(period_s));
    a->Visit(MJ_NVP(deadband));
    a->Visit(MJ_NVP(max_translate_x_mm_s));
    a->Visit(MJ_NVP(max_translate_y_mm_s));
    a->Visit(MJ_NVP(max_rotate_deg_s));
    a->Visit(MJ_NVP(max_body_z_mm));
    a->Visit(MJ_NVP(min_body_z_mm));
    a->Visit(MJ_NVP(idle_body_y_mm));
    a->Visit(MJ_NVP(forward_body_y_mm));
    a->Visit(MJ_NVP(reverse_body_y_mm));
    a->Visit(MJ_NVP(max_body_x_mm));
    a->Visit(MJ_NVP(max_body_y_mm));
    a->Visit(MJ_NVP(max_body_pitch_deg));
    a->Visit(MJ_NVP(max_body_roll_deg));
    a->Visit(MJ_NVP(max_turret_rate_deg_s));
    a->Visit(MJ_NVP(verbose));
  }
};

struct MechMessage {
  Command gait;
  TurretCommand turret;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(gait));
    a->Visit(MJ_NVP(turret));
  }
};

std::string SerializeCommand(const MechMessage& message_in) {
  MechMessage message = message_in;
  return mjmech::base::JsonWriteArchive::Write(&message);
}

class Commander {
 public:
  Commander(const Options& options,
            boost::asio::io_service& service,
            const udp::endpoint& target,
            LinuxInput* linux_input,
            udp::socket* socket)
      : options_(options),
        service_(service),
        target_(target),
        linux_input_(linux_input),
        socket_(socket),
        timer_(service),
        mapping_(GetAxisMapping(linux_input)) {
  }

  void Start() {
    StartRead();
    StartTimer();
  }

  void StartRead() {
    linux_input_->AsyncRead(
        &event_, std::bind(&Commander::HandleRead, this, std::placeholders::_1));
  }

  void StartTimer() {
    timer_.expires_from_now(ConvertSecondsToDuration(options_.period_s));
    timer_.async_wait(std::bind(&Commander::HandleTimeout, this,
                                std::placeholders::_1));
  }

 private:
  void HandleRead(ErrorCode ec) {
    FailIf(ec);
    StartRead();

    if (event_.ev_type == EV_KEY) {
      key_map_[event_.code] = event_.value;
      if (event_.code == mapping_.laser && event_.value) {
        laser_on_ = !laser_on_;
      }
    }
  }

  void HandleTimeout(ErrorCode ec) {
    FailIf(ec);
    StartTimer();

    MechMessage message;
    Command& command = message.gait;

    auto maybe_map = [this](double* destination, int mapping, int sign,
                            double minval, double maxval) {
      if (mapping < 0) { return; }

      double scaled = linux_input_->abs_info(mapping).scaled();
      scaled *= sign;
      if (std::abs(scaled) < options_.deadband) {
        scaled = 0.0;
      } else if (scaled > 0.0) {
        scaled = (scaled - options_.deadband) / (1.0 - options_.deadband);
      } else {
        scaled = (scaled + options_.deadband) / (1.0 - options_.deadband);
      }

      const double value = ((scaled < 0) ? minval : maxval) * std::abs(scaled);

      *destination = value;
    };

    if (key_map_[mapping_.body]) {
      maybe_map(&command.body_x_mm, mapping_.body_x,
                mapping_.sign_body_x,
                -options_.max_body_x_mm, options_.max_body_x_mm);
      maybe_map(&command.body_y_mm, mapping_.body_y,
                mapping_.sign_body_y,
                -options_.max_body_y_mm, options_.max_body_y_mm);
      maybe_map(&command.body_pitch_deg, mapping_.body_pitch,
                mapping_.sign_body_pitch,
                -options_.max_body_pitch_deg, options_.max_body_pitch_deg);
      maybe_map(&command.body_roll_deg, mapping_.body_roll,
                mapping_.sign_body_roll,
                -options_.max_body_roll_deg, options_.max_body_roll_deg);
    } else if (key_map_[mapping_.turret]) {
      message.turret.rate = TurretCommand::Rate();
      maybe_map(&(message.turret.rate->x_deg_s),
                mapping_.turret_x, mapping_.sign_turret_x,
                -options_.max_turret_rate_deg_s,
                options_.max_turret_rate_deg_s);
      maybe_map(&(message.turret.rate->y_deg_s),
                mapping_.turret_y, mapping_.sign_turret_y,
                -options_.max_turret_rate_deg_s,
                options_.max_turret_rate_deg_s);

      const bool do_fire =
          linux_input_->abs_info(mapping_.fire).scaled() > 0.0;
      message.turret.fire.sequence = sequence_++;
      using FM = TurretCommand::Fire::Mode;
      message.turret.fire.command = do_fire ? FM::kCont : FM::kOff;
    } else {
      maybe_map(&command.translate_x_mm_s, mapping_.translate_x,
                mapping_.sign_translate_x,
                -options_.max_translate_x_mm_s, options_.max_translate_x_mm_s);
      maybe_map(&command.translate_y_mm_s, mapping_.translate_y,
                mapping_.sign_translate_y,
                -options_.max_translate_y_mm_s, options_.max_translate_y_mm_s);
      maybe_map(&command.rotate_deg_s, mapping_.rotate,
                mapping_.sign_rotate,
                -options_.max_rotate_deg_s, options_.max_rotate_deg_s);
      maybe_map(&command.body_z_mm, mapping_.body_z,
                mapping_.sign_body_z,
                options_.min_body_z_mm, options_.max_body_z_mm);
    }

    if (key_map_[mapping_.crouch]) {
      command.body_z_mm = options_.min_body_z_mm;
    }

    const bool active =
        command.translate_x_mm_s != 0.0 ||
        command.translate_y_mm_s != 0.0 ||
        command.rotate_deg_s != 0;
    command.lift_height_percent = active ? 100.0 : 0.0;

    if (!key_map_[mapping_.body]) {
      if (active) {
        if (command.translate_y_mm_s > 0.0) {
          command.body_y_mm = options_.forward_body_y_mm;
        } else if (command.translate_y_mm_s < 0.0) {
          command.body_y_mm = options_.reverse_body_y_mm;
        } else {
          command.body_y_mm = options_.idle_body_y_mm;
        }
      } else {
        command.body_y_mm = options_.idle_body_y_mm;
      }
    }

    message.turret.laser_on = laser_on_;
    message.turret.agitator =
        key_map_[mapping_.agitator] ?
        TurretCommand::AgitatorMode::kOn : TurretCommand::AgitatorMode::kOff;

    if (options_.verbose) {
      std::cout << boost::format(
          "x=%4.0f y=%4.0f r=%4.0f z=%4.0f bx=%4.0f by=%4.0f") %
          command.translate_x_mm_s %
          command.translate_y_mm_s %
          command.rotate_deg_s %
          command.body_z_mm %
          command.body_x_mm %
          command.body_y_mm;
      if (message.turret.rate) {
        std::cout << boost::format(
            " r=%4.0f,%4.0f") %
            message.turret.rate->x_deg_s %
            message.turret.rate->y_deg_s;
      } else {
        std::cout << boost::format(" r=%4s,%4s") % "" % "";
      }
      std::cout << "\r";
      std::cout.flush();
    }


    std::string message_str = SerializeCommand(message);
    socket_->send_to(boost::asio::buffer(message_str), target_);
  }

  const Options options_;
  boost::asio::io_service& service_;
  const udp::endpoint target_;
  LinuxInput* const linux_input_;
  udp::socket* const socket_;
  boost::asio::deadline_timer timer_;
  const AxisMapping mapping_;

  bool laser_on_ = false;
  LinuxInput::Event event_;
  std::map<int, int> key_map_;
  int sequence_ = 0;
};

int work(int argc, char** argv) {
  boost::asio::io_service service;

  namespace po = boost::program_options;

  std::string target = "192.168.0.123";
  std::string joystick;
  MechMessage message;
  Command& command = message.gait;
  Options options;

  po::options_description desc("Allowable options");
  desc.add_options()
      ("help,h", "display usage message")
      ("target,t", po::value(&target), "destination of commands")
      ("joystick,j", po::value(&joystick),
       "send live commands from joystick at this device")
      ;

  ProgramOptionsArchive(&desc, "cmd.").Accept(&command);
  ProgramOptionsArchive(&desc, "opt.").Accept(&options);

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);

  if (vm.count("help")) {
    std::cerr << desc;
    return 0;
  }

  std::string host;
  std::string port_str;
  const size_t colon = target.find_first_of(':');
  if (colon != std::string::npos) {
    host = target.substr(0, colon);
    port_str = target.substr(colon + 1);
  } else {
    host = target;
    port_str = "13356";
  }

  udp::resolver resolver(service);
  auto it = resolver.resolve(udp::resolver::query(host, port_str));

  udp::socket socket(service, udp::v4());

  if (joystick.empty()) {
    auto message_str = SerializeCommand(message);
    socket.send_to(boost::asio::buffer(message_str), *it);
  } else {
    LinuxInput input(service, joystick);

    Commander commander(options, service, *it, &input, &socket);

    commander.Start();

    service.run();
  }

  return 0;
}
}

int main(int argc, char** argv) {
  try {
    return work(argc, argv);
  } catch (std::exception& e) {
    std::cerr << "error: " << e.what() << "\n";
    return 1;
  }
  return 0;
}
