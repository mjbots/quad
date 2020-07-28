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

#include "mech/turret_control.h"

#include <boost/asio/post.hpp>

#include "mjlib/base/clipp_archive.h"
#include "mjlib/base/limit.h"
#include "mjlib/io/repeating_timer.h"

#include "base/logging.h"
#include "base/telemetry_registry.h"

#include "mech/moteus.h"

namespace pl = std::placeholders;

namespace mjmech {
namespace mech {

namespace {
constexpr int kWeaponDivider = 10;  // 40Hz weapon updates

enum WeaponRegister {
  kFirePwm = 0x010,
  kFireTime = 0x011,
  kLoaderPwm = 0x012,
  kLoaderTime = 0x013,
  kLaser = 0x014,
  kShotCount = 0x015,
  kArmed = 0x016,
};

struct CommandLog {
  boost::posix_time::ptime timestamp;

  const TurretControl::CommandData* command = &ignored_command;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(timestamp));
    const_cast<TurretControl::CommandData*>(command)->Serialize(a);
  }

  static inline TurretControl::CommandData ignored_command;
};

struct ControlLog {
  boost::posix_time::ptime timestamp;

  TurretControl::ControlData control;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(timestamp));
    a->Visit(MJ_NVP(control));
  }
};

struct ImageLog {
  boost::posix_time::ptime timestamp;
  std::vector<Eigen::Vector2d> targets;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(timestamp));
    a->Visit(MJ_NVP(targets));
  }
};
}

class TurretControl::Impl {
 public:
  Impl(base::Context& context,
       ClientGetter client_getter,
       ImuGetter imu_getter)
      : executor_(context.executor),
        client_getter_(client_getter),
        imu_getter_(imu_getter) {
    context.telemetry_registry->Register("imu", &imu_signal_);
    context.telemetry_registry->Register("turret", &turret_signal_);
    context.telemetry_registry->Register("command", &command_signal_);
    context.telemetry_registry->Register("control", &control_signal_);
    context.telemetry_registry->Register("image", &image_signal_);
    context.telemetry_registry->Register("weapon", &weapon_signal_);
  }

  void AsyncStart(mjlib::io::ErrorCallback callback) {
    camera_ = std::make_unique<CameraDriver>(parameters_.camera);
    camera_->image_signal()->connect(
        std::bind(&Impl::HandleImage_THREAD, this, pl::_1));
    client_ = client_getter_();
    imu_client_ = imu_getter_();

    PopulateStatusRequest();

    timer_.start(mjlib::base::ConvertSecondsToDuration(parameters_.period_s),
                 std::bind(&Impl::HandleTimer, this, pl::_1));

    boost::asio::post(
        executor_,
        std::bind(std::move(callback), mjlib::base::error_code()));
  }

  void Command(const CommandData& data) {
    const auto now = Now();

    const bool higher_priority = data.priority >= current_command_.priority;
    const bool stale =
        !current_command_timestamp_.is_not_a_date_time() &&
        (mjlib::base::ConvertDurationToSeconds(
            now - current_command_timestamp_) > parameters_.command_timeout_s);

    if (!higher_priority && !stale) {
      return;
    }

    current_command_ = data;
    current_command_timestamp_ = now;

    CommandLog command_log;
    command_log.timestamp = Now();
    command_log.command = &current_command_;
    command_signal_(&command_log);
  }

  // private

  void HandleImage_THREAD(const cv::Mat& image) {
    // WE ARE IN A BG THREAD.
    if (!target_tracker_) {
      target_tracker_ = std::make_unique<TargetTracker>(parameters_.tracker);
    }
    const auto result = target_tracker_->Track(image);

    // Bounce these results back to the main thread.
    boost::asio::post(
        executor_,
        std::bind(&Impl::HandleImage, this, result));
  }

  void HandleImage(TargetTracker::Result result) {
    image_data_.timestamp = Now();
    image_data_.targets = result.targets;
    image_signal_(&image_data_);
  }

  void PopulateStatusRequest() {
    status_request_ = {};
    for (int id : { 1, 2}) {
      status_request_.push_back({});
      auto& current = status_request_.back();
      current.id = id;
      current.request.ReadMultiple(moteus::Register::kMode, 4, 3);
      current.request.ReadMultiple(moteus::Register::kVoltage, 3, 1);
    }

    {
      status_weapon_request_ = status_request_;
      status_weapon_request_.push_back({});
      auto& current = status_weapon_request_.back();
      current.id = 7;
      current.request.ReadMultiple(WeaponRegister::kFirePwm, 7, 1);
    }
  }

  void HandleTimer(const mjlib::base::error_code& ec) {
    if (ec == boost::asio::error::operation_aborted) { return; }
    mjlib::base::FailIf(ec);

    if (!client_) { return; }
    if (outstanding_) { return; }

    timing_ = ControlTiming(executor_, timing_.cycle_start());

    outstanding_ = true;

    status_outstanding_ = 2;
    status_count_ = (status_count_ + 1) % kWeaponDivider;
    status_reply_.clear();

    const auto* this_request =
        (status_count_ == 0) ? &status_weapon_request_ : &status_request_;

    client_->AsyncTransmit(
        this_request, &status_reply_,
        std::bind(&Impl::HandleStatus, this, pl::_1));

    imu_client_->ReadImu(
        &imu_data_, std::bind(&Impl::HandleStatus, this, pl::_1));
  }

  void HandleStatus(const mjlib::base::error_code& ec) {
    mjlib::base::FailIf(ec);

    status_outstanding_--;
    if (status_outstanding_ > 0) { return; }

    imu_signal_(&imu_data_);

    UpdateStatus();

    if (status_count_ == 0) {
      weapon_signal_(&weapon_);
    }

    timing_.finish_status();

    RunControl();
    timing_.finish_control();
  }

  void UpdateServo(const mjlib::multiplex::AsioClient::IdRegisterValue& reply,
                   Status::GimbalServo* servo) {
    servo->id = reply.id;
    const double sign = servo_sign_.at(reply.id);

    const auto* maybe_value = std::get_if<moteus::Value>(&reply.value);
    if (!maybe_value) { return; }
    const auto& value = *maybe_value;
    switch (static_cast<moteus::Register>(reply.reg)) {
      case moteus::kMode: {
        servo->mode = moteus::ReadInt(value);
        break;
      }
      case moteus::kPosition: {
        servo->angle_deg = sign * moteus::ReadPosition(value);
        break;
      }
      case moteus::kVelocity: {
        servo->velocity_dps = sign * moteus::ReadPosition(value);
        break;
      }
      case moteus::kTorque: {
        servo->torque_Nm = sign * moteus::ReadTorque(value);
        break;
      }
      case moteus::kVoltage: {
        servo->voltage = moteus::ReadVoltage(value);
        break;
      }
      case moteus::kTemperature: {
        servo->temperature_C = moteus::ReadTemperature(value);
        break;
      }
      case moteus::kFault: {
        servo->fault = moteus::ReadInt(value);
        break;
      }
      default: {
        break;
      }
    }
  }

  void UpdateWeapon(const mjlib::multiplex::AsioClient::IdRegisterValue& reply,
                    Weapon* weapon) {
    weapon->timestamp = Now();

    // NOTE: These won't be correct for int8_t values, but we're not
    // requesting them so it shouldn't be a problem for now.
    const auto* maybe_value = std::get_if<moteus::Value>(&reply.value);
    if (!maybe_value) { return; }
    const auto& value = *maybe_value;
    switch (static_cast<WeaponRegister>(reply.reg)) {
      case WeaponRegister::kFirePwm: {
        weapon->fire_pwm = moteus::ReadInt(value);
        break;
      }
      case WeaponRegister::kFireTime: {
        weapon->fire_time_10ms = moteus::ReadInt(value);
        break;
      }
      case WeaponRegister::kLoaderPwm: {
        weapon->loader_pwm = moteus::ReadInt(value);
        break;
      }
      case WeaponRegister::kLoaderTime: {
        weapon->loader_time_10ms = moteus::ReadInt(value);
        break;
      }
      case WeaponRegister::kLaser: {
        weapon->laser_time_10ms = moteus::ReadInt(value);
        break;
      }
      case WeaponRegister::kShotCount: {
        weapon->shot_count = moteus::ReadInt(value);
        break;
      }
      case WeaponRegister::kArmed: {
        weapon->armed = moteus::ReadInt(value);
        break;
      }
    }
  }

  void UpdateStatus() {
    const double imu_servo_pitch_deg =
        imu_data_.euler_deg.pitch - status_.pitch_servo.angle_deg;
    {
      const double alpha = parameters_.period_s / parameters_.imu_servo_filter_s;
      status_.imu_servo_pitch_deg =
          (1.0 - alpha) * status_.imu_servo_pitch_deg +
          alpha * imu_servo_pitch_deg;
    }

    for (const auto& reply : status_reply_) {
      auto* const servo = [&]() -> Status::GimbalServo* {
        if (reply.id == 1) { return &status_.pitch_servo; }
        if (reply.id == 2) { return &status_.yaw_servo; }
        return nullptr;
      }();
      auto* const weapon = [&]() -> Weapon* {
        if (reply.id == 7) { return &weapon_; }
        return nullptr;
      }();
      if (servo) {
        UpdateServo(reply, servo);
      } else if (weapon) {
        UpdateWeapon(reply, weapon);
      }
    }

    status_.imu.pitch_deg = imu_data_.euler_deg.pitch;
    status_.imu.pitch_rate_dps = imu_data_.rate_dps.y();
    status_.imu.yaw_deg = imu_data_.euler_deg.yaw;
    status_.imu.yaw_rate_dps = imu_data_.rate_dps.z();

    {
      const double alpha = parameters_.period_s / parameters_.voltage_filter_s;
      status_.filtered_bus_V = alpha * status_.pitch_servo.voltage +
                               (1.0 - alpha) * status_.filtered_bus_V;
    }

    if (status_.mode != Mode::kFault) {
      std::string fault;

      for (auto* servo : { &status_.pitch_servo, &status_.yaw_servo }) {
        if (servo->fault) {
          if (!fault.empty()) { fault += ", "; }
          fault += fmt::format("servo {} fault: {}", servo->id, servo->fault);
        }
      }

      if (!fault.empty()) {
        Fault(fault);
      }
    }
  }

  void RunControl() {
    if (current_command_.mode != status_.mode) {
      MaybeChangeMode();
    }

    switch (status_.mode) {
      case Mode::kStop: { DoControl_Stop(); break; }
      case Mode::kActive: { DoControl_Active(); break; }
      case Mode::kFault: { DoControl_Fault(); break; }
    }
  }

  void MaybeChangeMode() {
    const auto old_mode = status_.mode;
    switch (current_command_.mode) {
      case Mode::kStop: {
        status_.mode = Mode::kStop;
        break;
      }
      case Mode::kActive: {
        if (status_.mode == Mode::kFault) { break; }

        status_.control = {};

        // Start controlling right where we are.
        status_.control.pitch.angle_deg = imu_data_.euler_deg.pitch;
        status_.control.yaw.angle_deg = imu_data_.euler_deg.yaw;

        status_.mode = Mode::kActive;
        break;
      }
      case Mode::kFault: {
        mjlib::base::AssertNotReached();
      }
    }

    if (old_mode != status_.mode &&
        old_mode == Mode::kFault) {
      status_.fault = "";
    }
  }

  void HandleCommand(const mjlib::base::error_code& ec) {
    timing_.finish_command();

    status_.timestamp = Now();
    status_.timing = timing_.status();
    turret_signal_(&status_);

    outstanding_ = false;
  }

  void DoControl_Stop() {
    ControlData control;
    Control(control);
  }

  void DoControl_Active() {
    auto [pitch_rate_dps, yaw_rate_dps] = [&]() {
      if (!current_command_.track_target) {
        return std::make_pair(current_command_.pitch_rate_dps,
                              current_command_.yaw_rate_dps);
      }
      if (Recent(image_data_.timestamp) && !image_data_.targets.empty()) {
        // Pick the target closest to the center.
        const auto to_track = PickTarget();
        return std::make_pair(
            (to_track.y() - 0.5) * -parameters_.target_gain_dps,
            (to_track.x() - 0.5) * parameters_.target_gain_dps);
      }
      return std::make_pair(0., 0.);
    }();

    const double max_rate_change_dps =
        parameters_.max_rate_accel_dps2 * parameters_.period_s;

    pitch_rate_dps = mjlib::base::Limit(
        pitch_rate_dps,
        status_.control.pitch.rate_dps - max_rate_change_dps,
        status_.control.pitch.rate_dps + max_rate_change_dps);
    yaw_rate_dps = mjlib::base::Limit(
        yaw_rate_dps,
        status_.control.yaw.rate_dps - max_rate_change_dps,
        status_.control.yaw.rate_dps + max_rate_change_dps);

    status_.control.pitch.rate_dps = pitch_rate_dps;
    status_.control.yaw.rate_dps = yaw_rate_dps;

    status_.control.pitch.angle_deg +=
        pitch_rate_dps * parameters_.period_s;
    status_.control.yaw.angle_deg +=
        yaw_rate_dps * parameters_.period_s;

    // Limit the pitch angle to within some region based on the pitch servo.
    const double imu_pitch_min =
        parameters_.servo_pitch_min_deg + status_.imu_servo_pitch_deg;
    const double imu_pitch_max =
        parameters_.servo_pitch_max_deg + status_.imu_servo_pitch_deg;

    const double old_pitch_deg = status_.control.pitch.angle_deg;
    status_.control.pitch.angle_deg =
        mjlib::base::Limit(old_pitch_deg, imu_pitch_min, imu_pitch_max);
    if (status_.control.pitch.angle_deg != old_pitch_deg) {
      pitch_rate_dps = 0.0;
    }


    // Wrap around our yaw angle to always be a limited distance from
    // the current actual.
    const double yaw_delta_deg =
        base::Degrees(
            base::WrapNegPiToPi(
                base::Radians(
                    status_.control.yaw.angle_deg - imu_data_.euler_deg.yaw)));
    status_.control.yaw.angle_deg = imu_data_.euler_deg.yaw + yaw_delta_deg;

    ControlData control;
    control.pitch.power = true;
    control.pitch.torque_Nm =
        pitch_pid_.Apply(
            imu_data_.euler_deg.pitch, status_.control.pitch.angle_deg,
            imu_data_.rate_dps.y(), pitch_rate_dps,
            1.0 / parameters_.period_s);

    control.yaw.power = true;
    control.yaw.torque_Nm =
        yaw_pid_.Apply(
            imu_data_.euler_deg.yaw, status_.control.yaw.angle_deg,
            imu_data_.rate_dps.z(), yaw_rate_dps,
            1.0 / parameters_.period_s);

    Control(control);
  }

  void DoControl_Fault() {
    DoControl_Stop();
  }

  void Fault(std::string_view message) {
    status_.mode = Mode::kFault;
    status_.fault = message;

    log_.warn("Fault: " + std::string(message));

    DoControl_Fault();
  }

  void AddServoCommand(int id, const ControlData::Servo& servo) {
    const double sign = servo_sign_.at(id);

    client_command_.push_back({});
    auto& request = client_command_.back();
    request.id = id;

    const auto mode =
        servo.power ? moteus::Mode::kPosition : moteus::Mode::kStopped;
    request.request.WriteSingle(moteus::kMode, static_cast<int8_t>(mode));

    if (servo.power) {
      request.request.WriteSingle(
          moteus::kCommandFeedforwardTorque,
          static_cast<float>(sign * servo.torque_Nm));

      // Static only to save allocations.
      static std::vector<moteus::Value> values;
      if (values.empty()) {
        values.push_back(moteus::WritePwm(0, moteus::kInt8));
        values.push_back(moteus::WritePwm(0, moteus::kInt8));
      }
      request.request.WriteMultiple(moteus::kCommandKpScale, values);
    }
  }

  bool Recent(boost::posix_time::ptime timestamp) {
    return base::ConvertDurationToSeconds(Now() - timestamp) <
      parameters_.target_timeout_s;
  }

  Eigen::Vector2d PickTarget() {
    // Pick the target closest to the center.
    auto it = std::min_element(
        image_data_.targets.begin(), image_data_.targets.end(),
        [&](const auto& lhs, const auto& rhs) {
          return ((lhs - Eigen::Vector2d(0.5, 0.5)).norm() <
                  (rhs - Eigen::Vector2d(0.5, 0.5)).norm());
        });
    BOOST_VERIFY(it != image_data_.targets.end());
    return *it;
  }

  void AddWeapon() {
    // Do our weapon.
    client_command_.push_back({});
    auto& request = client_command_.back();
    request.id = 7;

    request.request.WriteSingle(
        WeaponRegister::kLaser,
        current_command_.laser_enable ?
        static_cast<int16_t>(parameters_.laser_time_10ms) : 0);

    if (current_command_.trigger_sequence >= 1024 &&
        current_command_.trigger_sequence <= 2048 &&
        current_command_.trigger_sequence != last_trigger_sequence_) {
      auto old = last_trigger_sequence_;

      last_trigger_sequence_ = current_command_.trigger_sequence;

      // We don't want to fire just when starting up.
      if (old != 0) {
        // Static just to save allocations.
        static std::vector<moteus::Value> values;
        values.clear();
        values.push_back(
            static_cast<int16_t>(
                1000.0 * parameters_.fire_voltage / status_.filtered_bus_V));
        values.push_back(
            static_cast<int16_t>(parameters_.fire_time_10ms));
        values.push_back(
            static_cast<int16_t>(
                1000.0 * parameters_.loader_voltage / status_.filtered_bus_V));
        values.push_back(
            static_cast<int16_t>(parameters_.loader_time_10ms));

        request.request.WriteMultiple(WeaponRegister::kFirePwm, values);
      }
    }
  }

  void Control(const ControlData& control) {
    ControlLog control_log;
    control_log.timestamp = Now();
    control_log.control = control;
    control_signal_(&control_log);

    client_command_.clear();
    AddServoCommand(1, control.pitch);
    AddServoCommand(2, control.yaw);

    if (status_count_ == 0) {
      AddWeapon();
    }

    client_command_reply_.clear();
    client_->AsyncTransmit(
        &client_command_, &client_command_reply_,
        std::bind(&Impl::HandleCommand, this, pl::_1));
  }


  boost::posix_time::ptime Now() {
    return mjlib::io::Now(executor_.context());
  }

  boost::asio::any_io_executor executor_;
  ClientGetter client_getter_;
  ImuGetter imu_getter_;

  base::LogRef log_ = base::GetLogInstance("QuadrupedControl");

  Status status_;
  Weapon weapon_;

  int16_t last_trigger_sequence_ = 0;
  CommandData current_command_;
  boost::posix_time::ptime current_command_timestamp_;
  Parameters parameters_;

  using Client = mjlib::multiplex::AsioClient;
  Client* client_ = nullptr;
  ImuClient* imu_client_ = nullptr;

  mjlib::io::RepeatingTimer timer_{executor_};
  bool outstanding_ = false;
  int status_outstanding_ = 0;

  using Request = Client::Request;
  Request status_request_;
  Request status_weapon_request_;
  Client::Reply status_reply_;
  int status_count_ = 0;

  Request client_command_;
  Client::Reply client_command_reply_;

  AttitudeData imu_data_;
  ImageLog image_data_;
  boost::signals2::signal<void (const AttitudeData*)> imu_signal_;
  boost::signals2::signal<void (const Status*)> turret_signal_;
  boost::signals2::signal<void (const CommandLog*)> command_signal_;
  boost::signals2::signal<void (const ControlLog*)> control_signal_;
  boost::signals2::signal<void (const ImageLog*)> image_signal_;
  boost::signals2::signal<void (const Weapon*)> weapon_signal_;

  ControlTiming timing_{executor_, {}};

  std::map<int, double> servo_sign_ = {
    { 1, 1.0 },
    { 2, -1.0 },
  };

  mjlib::base::PID pitch_pid_{&parameters_.pitch, &status_.control.pitch.pid};
  mjlib::base::PID yaw_pid_{&parameters_.yaw, &status_.control.yaw.pid};

  std::unique_ptr<CameraDriver> camera_;
  std::unique_ptr<TargetTracker> target_tracker_;
};

TurretControl::TurretControl(base::Context& context,
                             ClientGetter client_getter,
                             ImuGetter imu_getter)
    : impl_(std::make_unique<Impl>(context, client_getter, imu_getter)) {}

TurretControl::~TurretControl() {}

void TurretControl::AsyncStart(mjlib::io::ErrorCallback callback) {
  impl_->AsyncStart(std::move(callback));
}

const TurretControl::Status& TurretControl::status() const {
  return impl_->status_;
}

const TurretControl::Weapon& TurretControl::weapon() const {
  return impl_->weapon_;
}

void TurretControl::Command(const CommandData& command) {
  impl_->Command(command);
}

clipp::group TurretControl::program_options() {
  return mjlib::base::ClippArchive().Accept(&impl_->parameters_).release();
}


}
}
