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

#include <linux/input.h>

#include <boost/asio.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/program_options.hpp>
#include <boost/property_tree/json_parser.hpp>

#include "base/deadline_timer.h"
#include "base/fail.h"
#include "base/json_archive.h"
#include "base/linux_input.h"
#include "base/program_options_archive.h"
#include "base/property_tree_archive.h"

#include "drive_command.h"
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
  int sign_turret_x = -1;

  int turret_y = -1;
  int sign_turret_y = -1;

  int fire = -1;
  int laser = -1;
  int agitator = -1;

  int crouch = -1;
  int manual = -1;
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
    result.sign_turret_x = 1;
  }
  if (features.capabilities.test(ABS_RY)) {
    result.body_z = ABS_RY;
    result.sign_body_z = -1;

    result.body_pitch = ABS_RY;
    result.sign_body_pitch = 1;

    result.turret_y = ABS_RY;
    result.sign_turret_y = -1;
  }

  result.crouch = BTN_A;
  result.manual = BTN_TR;
  result.body = ABS_Z;
  result.turret = BTN_TL;

  result.laser = BTN_WEST;
  result.fire = ABS_RZ;
  result.agitator = BTN_NORTH;

  return result;
}

struct OptOptions {
  double period_s = 0.1;
  double deadband = 0.20;
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
  double turret_linear_transition_point = 0.5;
  double turret_linear_fine_percent = 0.2;
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
    a->Visit(MJ_NVP(turret_linear_transition_point));
    a->Visit(MJ_NVP(turret_linear_fine_percent));
    a->Visit(MJ_NVP(verbose));
  }
};

struct CommanderOptions {
  std::string target = "192.168.0.123";
  Command cmd;
  OptOptions opt;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(target));
    a->Visit(MJ_NVP(cmd));
    a->Visit(MJ_NVP(opt));
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

struct MechDriveMessage {
  DriveCommand drive;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(drive));
  }
};

template <typename T>
std::string SerializeCommand(const T& message_in) {
  T message = message_in;
  return mjmech::base::JsonWriteArchive::Write(&message);
}

class Commander {
 public:
  Commander(const CommanderOptions& options,
            boost::asio::io_service& service)
      : options_(options.opt),
        service_(service),
        command_(options.cmd),
        timer_(service) {

    std::string host;
    std::string port_str;
    std::string target = options.target;
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
    target_ = *it;

    socket_.reset(new udp::socket(service, udp::v4()));
  }

  void Start() {
    BOOST_ASSERT(linux_input_);
    StartRead();
    StartTimer();
  }

  // Set the linux input object. Pointer must be valid for the lifetime of the
  // object.
  void SetLinuxInput(LinuxInput* target) {
    linux_input_ = target;
    mapping_ = GetAxisMapping(linux_input_);
  }


  void SendMechMessage(const MechMessage& message) {
    std::string message_str = SerializeCommand(message);
    socket_->send_to(boost::asio::buffer(message_str), target_);
  }

 private:
  void StartRead() {
    linux_input_->AsyncRead(
        &event_, std::bind(&Commander::HandleRead, this,
                           std::placeholders::_1));
  }

  void StartTimer() {
    timer_.expires_from_now(ConvertSecondsToDuration(options_.period_s));
    timer_.async_wait(std::bind(&Commander::HandleTimeout, this,
                                std::placeholders::_1));
  }

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

  void MaybeMapNonLinear(
      double* destination, int mapping, int sign,
      double center, double minval, double maxval,
      double transition_point, double fine_percent) const {

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

    double abs_scaled = std::abs(scaled);

    if (abs_scaled <= transition_point) {
      abs_scaled = (abs_scaled / transition_point) * fine_percent;
    } else {
      abs_scaled =
          ((abs_scaled - transition_point) /
           (1.0 - transition_point) *
           (1.0 - fine_percent)) +
          fine_percent;
    }

    const double value = (
        (scaled < 0) ?
        (abs_scaled * (minval - center) + center) :
        (abs_scaled * (maxval - center) + center));

    *destination = value;
  };

  void MaybeMap(double* destination, int mapping, int sign,
                double center, double minval, double maxval) const {
    MaybeMapNonLinear(destination, mapping, sign,
                      center, minval, maxval, 1.0, 1.0);
  }

  bool body_enabled() const {
    return linux_input_->abs_info(mapping_.body).scaled() > 0.0;
  }

  bool turret_enabled() const {
    auto it = key_map_.find(mapping_.turret);
    if (it == key_map_.end()) { return false; }
    return it->second;
  }

  bool manual_enabled() const {
    auto it = key_map_.find(mapping_.manual);
    if (it == key_map_.end()) { return false; }
    return it->second;
  }

  bool drive_enabled() const {
    return !body_enabled() && !turret_enabled() && !manual_enabled();
  }

  void HandleTimeout(ErrorCode ec) {
    FailIf(ec);
    StartTimer();

    // The body mode acts independently of anything else and updates
    // the persistent body pose.
    if (body_enabled()) {
      MaybeMap(&command_.body_x_mm, mapping_.body_x,
               mapping_.sign_body_x, 0.0,
               -options_.max_body_x_mm, options_.max_body_x_mm);
      MaybeMap(&command_.body_y_mm, mapping_.body_y,
               mapping_.sign_body_y, 0.0,
               -options_.max_body_y_mm, options_.max_body_y_mm);
      MaybeMap(&command_.body_pitch_deg, mapping_.body_pitch,
               mapping_.sign_body_pitch, 0.0,
               -options_.max_body_pitch_deg, options_.max_body_pitch_deg);
      MaybeMap(&command_.body_roll_deg, mapping_.body_roll,
               mapping_.sign_body_roll, 0.0,
               -options_.max_body_roll_deg, options_.max_body_roll_deg);
    }

    if (!drive_enabled()) {
      DoManual();
    } else {
      DoDrive();
    }
  }

  void DoManual() {
    MechMessage message;
    Command& command = message.gait;

    command = command_;

    if (turret_enabled()) {
      message.turret.rate = TurretCommand::Rate();
      MaybeMapNonLinear(
          &(message.turret.rate->x_deg_s),
          mapping_.turret_x, mapping_.sign_turret_x,
          0,
          -options_.max_turret_rate_deg_s,
          options_.max_turret_rate_deg_s,
          options_.turret_linear_transition_point,
          options_.turret_linear_fine_percent);
      MaybeMapNonLinear(
          &(message.turret.rate->y_deg_s),
          mapping_.turret_y, mapping_.sign_turret_y,
          0,
          -options_.max_turret_rate_deg_s,
          options_.max_turret_rate_deg_s,
          options_.turret_linear_transition_point,
          options_.turret_linear_fine_percent);

      const bool do_fire =
          linux_input_->abs_info(mapping_.fire).scaled() > 0.0;
      message.turret.fire_control.fire.sequence = sequence_++;
      using FM = TurretCommand::Fire::Mode;
      message.turret.fire_control.fire.command = do_fire ? FM::kCont : FM::kOff;
    } else {
      MaybeMap(&command.translate_x_mm_s, mapping_.translate_x,
               mapping_.sign_translate_x,
               command_.translate_x_mm_s,
               -options_.max_translate_x_mm_s, options_.max_translate_x_mm_s);
      MaybeMap(&command.translate_y_mm_s, mapping_.translate_y,
               mapping_.sign_translate_y,
               command_.translate_y_mm_s,
               -options_.max_translate_y_mm_s, options_.max_translate_y_mm_s);
      MaybeMap(&command.rotate_deg_s, mapping_.rotate,
               mapping_.sign_rotate,
               command_.rotate_deg_s,
               -options_.max_rotate_deg_s, options_.max_rotate_deg_s);
      MaybeMap(&command.body_z_mm, mapping_.body_z,
               mapping_.sign_body_z,
               command_.body_z_mm,
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

    message.turret.fire_control.laser_on = laser_on_;
    message.turret.fire_control.agitator =
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

    SendMechMessage(message);
  }

  void DoDrive() {
    MechDriveMessage message;
    auto& command = message.drive;

    command.body_offset_mm.x = command_.body_x_mm;
    command.body_offset_mm.y = command_.body_y_mm;
    command.body_attitude_deg.pitch = command_.body_pitch_deg;
    command.body_attitude_deg.roll = command_.body_roll_deg;

    MaybeMap(&command.drive_mm_s.x, mapping_.translate_x,
             mapping_.sign_translate_x,
             command_.translate_x_mm_s,
             -options_.max_translate_x_mm_s, options_.max_translate_x_mm_s);
    MaybeMap(&command.drive_mm_s.y, mapping_.translate_y,
             mapping_.sign_translate_y,
             command_.translate_y_mm_s,
             -options_.max_translate_y_mm_s, options_.max_translate_y_mm_s);
    MaybeMapNonLinear(
        &command.turret_rate_dps.yaw, mapping_.turret_x,
        mapping_.sign_turret_x,
        0,
        -options_.max_turret_rate_deg_s,
        options_.max_turret_rate_deg_s,
        options_.turret_linear_transition_point,
        options_.turret_linear_fine_percent);
    MaybeMapNonLinear(
        &command.turret_rate_dps.pitch, mapping_.turret_y,
        mapping_.sign_turret_y,
        0,
        -options_.max_turret_rate_deg_s,
        options_.max_turret_rate_deg_s,
        options_.turret_linear_transition_point,
        options_.turret_linear_fine_percent);

    if (key_map_[mapping_.crouch]) {
      command.body_offset_mm.z = options_.min_body_z_mm;
    }

    const bool do_fire =
        linux_input_->abs_info(mapping_.fire).scaled() > 0.0;
    command.fire_control.fire.sequence = sequence_++;
    using FM = TurretCommand::Fire::Mode;
    command.fire_control.fire.command = do_fire ? FM::kCont : FM::kOff;
    command.fire_control.laser_on = laser_on_;
    command.fire_control.agitator =
        key_map_[mapping_.agitator] ?
        TurretCommand::AgitatorMode::kOn : TurretCommand::AgitatorMode::kOff;

    std::string message_str = SerializeCommand(message);
    socket_->send_to(boost::asio::buffer(message_str), target_);

    if (options_.verbose) {
      std::cout << boost::format(
          "x=%4.0f y=%4.0f tx=%4.0f ty=%4.0f bx=%4.0f by=%4.0f") %
          command.drive_mm_s.x %
          command.drive_mm_s.y %
          command.turret_rate_dps.yaw %
          command.turret_rate_dps.pitch %
          command.body_offset_mm.x %
          command.body_offset_mm.y;
      std::cout << "\r";
      std::cout.flush();
    }
  }

  const OptOptions options_;
  boost::asio::io_service& service_;
  udp::endpoint target_;
  LinuxInput* linux_input_ = nullptr;
  std::unique_ptr<udp::socket> socket_;
  Command command_;
  DeadlineTimer timer_;
  AxisMapping mapping_;

  bool laser_on_ = false;
  LinuxInput::Event event_;
  std::map<int, int> key_map_;
  int sequence_ = 0;
};

int work(int argc, char** argv) {
  boost::asio::io_service service;

  namespace po = boost::program_options;

  std::string joystick;
  double turret_pitch_rate_dps = 0.0;
  double turret_yaw_rate_dps = 0.0;

  CommanderOptions options;

  po::options_description desc("Allowable options");
  desc.add_options()
      ("help,h", "display usage message")
      ("joystick,j", po::value(&joystick),
       "send live commands from joystick at this device")
      ("turret.pitch_rate_dps", po::value(&turret_pitch_rate_dps), "")
      ("turret.yaw_rate_dps", po::value(&turret_yaw_rate_dps), "")
      ;

  ProgramOptionsArchive(&desc).Accept(&options);

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);

  if (vm.count("help")) {
    std::cerr << desc;
    return 0;
  }

  Commander commander(options, service);

  if (joystick.empty()) {
    MechMessage message;
    message.gait = options.cmd;
    TurretCommand& turret = message.turret;
    if (turret_pitch_rate_dps != 0.0 ||
        turret_yaw_rate_dps != 0.0) {
      turret.rate = TurretCommand::Rate();
      turret.rate->x_deg_s = turret_yaw_rate_dps;
      turret.rate->y_deg_s = turret_pitch_rate_dps;
    }
    commander.SendMechMessage(message);
  } else {
    LinuxInput* input = new LinuxInput(service, joystick);
    commander.SetLinuxInput(input);
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
