// Copyright 2015-2019 Josh Pieper, jjp@pobox.com.  All rights reserved.
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

#include "mech_warfare_command.h"

#include <linux/input.h>

#include <boost/asio/post.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/program_options.hpp>

#include <fmt/format.h>

#include "mjlib/base/json5_write_archive.h"
#include "mjlib/base/fail.h"
#include "mjlib/base/program_options_archive.h"
#include "mjlib/io/deadline_timer.h"

#include "base/logging.h"

namespace mjmech {
namespace mech {
namespace mw_command {

namespace {

using namespace mjmech::base;
typedef boost::asio::ip::udp udp;

bool IsAxis(int item) {
  if (item >= ABS_X && item <= ABS_MAX) { return true; }
  return false;
}

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
  int target_relative = -1;
  int body = -1;
  int freeze = -1;
  int pause = -1;
  int target_move = -1;
};

AxisMapping GetAxisMapping(const LinuxInput* input) {
  auto features = input->features(EV_ABS);
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

  if (input->name().find("Logitech Dual Action") != std::string::npos) {
    if (features.capabilities.test(ABS_Z)) {
      result.rotate = ABS_Z;
      result.sign_rotate = 1;

      result.body_roll = ABS_Z;
      result.sign_body_roll = 1;

      result.turret_x = ABS_Z;
      result.sign_turret_x = 1;
    }

    if (features.capabilities.test(ABS_RZ)) {
      result.body_z = ABS_RZ;
      result.sign_body_z = -1;

      result.body_pitch = ABS_RZ;
      result.sign_body_pitch = 1;

      result.turret_y = ABS_RZ;
      result.sign_turret_y = -1;
    }

    result.crouch = BTN_THUMB;
    // result.manual = BTN_PINKIE;
    result.target_relative = BTN_BASE;
    result.body = BTN_PINKIE;
    result.freeze = BTN_TOP2;
    result.pause = BTN_BASE4;
    result.target_move = BTN_THUMB2;

    result.laser = BTN_TOP;
    result.fire = BTN_BASE2;
    result.agitator = BTN_JOYSTICK;
  } else {
    // For now, we'll just hard-code the mapping used by jpieper's xbox
    // controller.  In the long term, or even medium term, this should
    // probably be configurable somehow.
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
    result.target_relative = ABS_Z;
    result.freeze = BTN_TL;
    result.pause = BTN_START;
    result.target_move = BTN_X;

    result.laser = BTN_WEST;
    result.fire = ABS_RZ;
    result.agitator = BTN_NORTH;
  }

  return result;
}


struct MechDriveMessage {
  DriveCommand drive;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(drive));
  }
};

template <typename T>
std::string SerializeCommand(const T& message) {
  return mjlib::base::Json5WriteArchive::Write(message);
}

}

class Commander::Impl {
 public:
  Impl(Commander::Parameters& params,
       const boost::asio::executor& executor)
      : parameters_(params),
        options_(params.opt),
        executor_(executor),
        timer_(executor) {
    mjlib::base::ProgramOptionsArchive(&po_options_).Accept(&params);
  }

  void AsyncStart(mjlib::io::ErrorCallback handler) {
    // Create joystick, if needed.
    if (!parameters_.joystick.empty()) {
      log_.infoStream() << "Opening joystick: " << parameters_.joystick;
      linux_input_.reset(new LinuxInput(executor_, parameters_.joystick));
      mapping_ = GetAxisMapping(linux_input_.get());
    }

    // Copy the default command.
    command_ = parameters_.cmd;

    // Resolve target
    std::string host;
    std::string port_str;
    std::string target = parameters_.target;
    const size_t colon = target.find_first_of(':');
    if (colon != std::string::npos) {
      host = target.substr(0, colon);
      port_str = target.substr(colon + 1);
    } else {
      host = target;
      port_str = "13356";
    }

    udp::resolver resolver(executor_);
    auto it = resolver.resolve(udp::resolver::query(host, port_str));
    target_ = *it;

    log_.infoStream() << "Will send commands to " << target_;

    socket_.reset(new udp::socket(executor_, udp::v4()));

    // Start reading
    if (linux_input_) {
      StartRead();
      // Do not send command when we have no joystick.
      StartTimer();
    }

    boost::asio::post(
        executor_,
        std::bind(std::move(handler), mjlib::base::error_code()));
  }

  void SendMechMessage(const MechMessage& message) {
    if (paused_) { return; }

    std::string message_str = SerializeCommand(message);
    socket_->send_to(boost::asio::buffer(message_str), target_);
  }


  void StartRead() {
    linux_input_->AsyncRead(
        &event_, std::bind(&Impl::HandleRead, this,
                           std::placeholders::_1));
  }

  void StartTimer() {
    timer_.expires_from_now(ConvertSecondsToDuration(options_.period_s));
    timer_.async_wait(std::bind(&Impl::HandleTimeout, this,
                                std::placeholders::_1));
  }

  void HandleRead(mjlib::base::error_code ec) {
    mjlib::base::FailIf(ec);
    StartRead();

    if (event_.ev_type == EV_KEY) {
      key_map_[event_.code] = event_.value;
      if (event_.code == mapping_.laser && event_.value) {
        laser_on_ = !laser_on_;
      }
      if (event_.code == mapping_.pause && event_.value) {
        paused_ = !paused_;
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

  bool key_pressed(int key) const {
    auto it = key_map_.find(key);
    if (it == key_map_.end()) { return false; }
    return it->second;
  }

  bool target_relative_enabled() const {
    if (IsAxis(mapping_.target_relative)) {
      return linux_input_->abs_info(mapping_.target_relative).scaled() > 0.0;
    }
    return key_pressed(mapping_.target_relative);
  }

  bool body_enabled() const {
    return key_pressed(mapping_.body);
  }

  bool freeze_enabled() const {
    return key_pressed(mapping_.freeze);
  }

  bool fire_enabled() const {
    if (IsAxis(mapping_.fire)) {
      return linux_input_->abs_info(mapping_.fire).scaled() > 0.0;
    }
    return key_pressed(mapping_.fire);
  }

  bool manual_enabled() const {
    return options_.manual_drive || key_pressed(mapping_.manual);
  }

  bool drive_enabled() const {
    return !body_enabled() && !manual_enabled();
  }

  bool target_enabled() const {
    return key_pressed(mapping_.target_move);
  }

  void HandleTimeout(mjlib::base::error_code ec) {
    mjlib::base::FailIf(ec);
    StartTimer();

    // The body mode acts independently of anything else and updates
    // the persistent body pose.
    if (body_enabled()) {
      MaybeMap(&command_.body_z_mm, mapping_.body_x,
               mapping_.sign_body_x, 0.0,
               options_.min_body_z_mm, options_.max_body_z_mm);
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

    if (!body_enabled()) {
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

    if (!body_enabled()) {
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
        TurretCommand::AgitatorMode::kOn : agitator_off_mode();

    if (options_.verbose) {
      std::cout << fmt::format(
          "x={:4.0f} y={:4.0f} r={:4.0f} z={:4.0f} bx={:4.0f} by={:4.0f} p={:3.1f} r={:3.1f} {}",
          command.translate_x_mm_s,
          command.translate_y_mm_s,
          command.rotate_deg_s,
          command.body_z_mm,
          command.body_x_mm,
          command.body_y_mm,
          command.body_pitch_deg,
          command.body_roll_deg,
          (body_enabled() ? "BODY" : "    "));
      if (message.turret.rate) {
        std::cout << fmt::format(
            " r={:4.0f},{:4.0f}",
            message.turret.rate->x_deg_s,
            message.turret.rate->y_deg_s);
      } else {
        std::cout << fmt::format(" r={:4s},{:4s}", "", "");
      }
      std::cout << "\r";
      std::cout.flush();
    }

    SendMechMessage(message);
  }

  void DoDrive() {
    MechDriveMessage message;
    auto& command = message.drive;

    command.body_offset_mm.x() = command_.body_x_mm;
    command.body_offset_mm.y() = command_.body_y_mm;
    command.body_attitude_deg.pitch = command_.body_pitch_deg;
    command.body_attitude_deg.roll = command_.body_roll_deg;

    if (target_enabled()) {
      double target_x_px_s = 0.0;
      MaybeMap(&target_x_px_s, mapping_.translate_x,
               mapping_.sign_translate_x,
               0.0, -options_.target_max_rate, options_.target_max_rate);
      double target_y_px_s = 0.0;
      MaybeMap(&target_y_px_s, mapping_.translate_y,
               -mapping_.sign_translate_y,
               0.0, -options_.target_max_rate, options_.target_max_rate);
      target_x_ += target_x_px_s * options_.period_s;
      target_y_ += target_y_px_s * options_.period_s;
      target_offset_signal_(static_cast<int>(target_x_),
                            static_cast<int>(target_y_));
    } else if (!body_enabled()) {
      MaybeMap(&command.drive_mm_s.x(), mapping_.translate_x,
               mapping_.sign_translate_x,
               command_.translate_x_mm_s,
               -options_.max_translate_x_mm_s, options_.max_translate_x_mm_s);
      MaybeMap(&command.drive_mm_s.y(), mapping_.translate_y,
               mapping_.sign_translate_y,
               command_.translate_y_mm_s,
               -options_.max_translate_y_mm_s, options_.max_translate_y_mm_s);

      if (target_relative_enabled()) {
        base::Point3D target_relative;
        target_relative.x() = target_x_ + 1280 / 2;
        target_relative.y() = target_y_ + 720 / 2;
        command.turret_target_relative = target_relative;
      } else {
        base::Euler rate_dps;
        MaybeMapNonLinear(
            &rate_dps.yaw, mapping_.turret_x,
            mapping_.sign_turret_x,
            0,
            -options_.max_turret_rate_deg_s,
            options_.max_turret_rate_deg_s,
            options_.turret_linear_transition_point,
            options_.turret_linear_fine_percent);
        MaybeMapNonLinear(
            &rate_dps.pitch, mapping_.turret_y,
            mapping_.sign_turret_y,
            0,
            -options_.max_turret_rate_deg_s,
            options_.max_turret_rate_deg_s,
            options_.turret_linear_transition_point,
            options_.turret_linear_fine_percent);
        command.turret_rate_dps = rate_dps;
      }
    }

    if (key_map_[mapping_.crouch]) {
      command.body_offset_mm.z() = options_.min_body_z_mm;
    }

    const bool do_fire = fire_enabled();
    command.fire_control.fire.sequence = sequence_++;
    using FM = TurretCommand::Fire::Mode;
    command.fire_control.fire.command = do_fire ? FM::kCont : FM::kOff;
    command.fire_control.laser_on = laser_on_;
    command.fire_control.agitator =
        key_map_[mapping_.agitator] ?
        TurretCommand::AgitatorMode::kOn : agitator_off_mode();
    command.freeze_rotation = freeze_enabled();

    std::string message_str = SerializeCommand(message);
    if (!paused_) {
      socket_->send_to(boost::asio::buffer(message_str), target_);
    }

    if (options_.verbose) {
      auto format_turret = [](const auto& command) {
        std::string result;
        if (command.turret_rate_dps) {
          result += fmt::format("rate={:4.0f}/{:4.0f}",
                                command.turret_rate_dps->yaw,
                                command.turret_rate_dps->pitch);
        } else if (command.turret_target_relative) {
          result += fmt::format("tgt={:.0f}/{:.0f}",
                                command.turret_target_relative->x(),
                                command.turret_target_relative->y());
        }
        return result;
      };

      std::cout << fmt::format(
          "x={:4.0f} y={:4.0f} t={} bx={:4.0f} by={:4.0f} {} {} {}",
          command.drive_mm_s.x(),
          command.drive_mm_s.y(),
          format_turret(command),
          command.body_offset_mm.x(),
          command.body_offset_mm.y(),
          (body_enabled() ? "BODY" : "    "),
          (do_fire ? "FIRE" : "    "),
          (command.freeze_rotation ? "FREEZE" : "      "));
      std::cout << "\r";
      std::cout.flush();
    }
  }

  TurretCommand::AgitatorMode agitator_off_mode() const {
    return options_.manual_agitator ?
        TurretCommand::AgitatorMode::kOff :
        TurretCommand::AgitatorMode::kAuto;
  }

  const Parameters& parameters_;
  const OptOptions& options_;
  boost::program_options::options_description po_options_;
  Command command_;
  boost::asio::executor executor_;
  udp::endpoint target_;
  std::unique_ptr<LinuxInput> linux_input_;
  std::unique_ptr<udp::socket> socket_;
  mjlib::io::DeadlineTimer timer_;
  AxisMapping mapping_;

  bool laser_on_ = false;
  LinuxInput::Event event_;
  std::map<int, int> key_map_;
  int sequence_ = 0;
  bool paused_ = true;

  base::LogRef log_ = base::GetLogInstance("commander");

  double target_x_ = 0.0;
  double target_y_ = 0.0;
  TargetOffsetSignal target_offset_signal_;
};

Commander::Commander(const boost::asio::executor& executor)
    : impl_(new Impl(parameters_, executor)) {
}

Commander::~Commander() {};

void Commander::AsyncStart(mjlib::io::ErrorCallback handler) {
  impl_->AsyncStart(std::move(handler));
}

void Commander::SendMechMessage(const MechMessage& msg) {
  impl_->SendMechMessage(msg);
}

boost::program_options::options_description* Commander::options() {
  return &impl_->po_options_;
}

Commander::TargetOffsetSignal* Commander::target_offset_signal() {
  return &impl_->target_offset_signal_;
}

}
}
}
