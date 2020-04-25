// Copyright 2015-2020 Josh Pieper, jjp@pobox.com.
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

#include <optional>

#include <boost/signals2.hpp>

#include "mjlib/base/visitor.h"

#include "base/point3d.h"

#include "mech/multiplex_client.h"
#include "mech/turret_command.h"

namespace mjmech {
namespace mech {

/// This class interacts with the turret, coordinating aiming and
/// firing.
class Turret : boost::noncopyable {
 public:
  template <typename Context>
  Turret(Context& context)
      : Turret(context.executor, false) {
    context.telemetry_registry->Register("turret",
                                        &turret_data_signal_);
    context.telemetry_registry->Register("turret_command",
                                        &turret_command_signal_);
  }

  ~Turret();

  void AsyncStart(mjlib::io::ErrorCallback);

  void SetMultiplexClient(MultiplexClient::Client*);

  void SetCommand(const TurretCommand&);
  void SetFireControl(const TurretCommand::FireControl&);

  void UpdateTrackedTarget(const std::optional<base::Point3D>& target);

  void StartBias();

  struct Parameters {
    int gimbal_address = 98;
    double period_s = 0.1;
    int command_update_decimate = 10;
    int error_disable_count = 3;
    double initial_disable_period_s = 1.0;
    double max_disable_period_s = 60.0;
    double auto_agitator_time_s = 0.5;
    double min_x_deg = -120;
    double max_x_deg = 120;
    double min_y_deg = -35;
    double max_y_deg = 35;
    double target_time_constant_s = 1.0;

    double fire_duration_s = 0.2;
    double agitator_pwm = 0.5;
    double fire_motor_pwm = 0.75;
    // TODO(josh.pieper): This should be in the camera somewhere.
    double pixels_per_degree = 1280 / 90.0;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(gimbal_address));
      a->Visit(MJ_NVP(period_s));
      a->Visit(MJ_NVP(command_update_decimate));
      a->Visit(MJ_NVP(error_disable_count));
      a->Visit(MJ_NVP(initial_disable_period_s));
      a->Visit(MJ_NVP(max_disable_period_s));
      a->Visit(MJ_NVP(auto_agitator_time_s));
      a->Visit(MJ_NVP(min_x_deg));
      a->Visit(MJ_NVP(max_x_deg));
      a->Visit(MJ_NVP(min_y_deg));
      a->Visit(MJ_NVP(max_y_deg));
      a->Visit(MJ_NVP(target_time_constant_s));
      a->Visit(MJ_NVP(fire_duration_s));
      a->Visit(MJ_NVP(agitator_pwm));
      a->Visit(MJ_NVP(fire_motor_pwm));
      a->Visit(MJ_NVP(pixels_per_degree));
    }
  };

  Parameters* parameters();

  struct Data {
    boost::posix_time::ptime timestamp;
    bool agitator_enabled = false;
    bool fire_enabled = false;
    int fire_count = 0;

    // The current state of the gimbal.
    TurretCommand::Imu imu;
    TurretCommand::Absolute absolute;

    // The current target_relative command, if that mode is active.
    std::optional<TurretCommand::TargetRelative> target_relative;
    TurretCommand::Rate target_relative_rate;
    boost::posix_time::ptime target_relative_last_time;

    int last_sequence = -1;
    bool last_rate = false;
    double disable_period_s = 0.0;
    double total_fire_time_s = 0.0;

    boost::posix_time::ptime auto_agitator_end;
    TurretCommand::Fire::Mode last_fire_command =
        TurretCommand::Fire::Mode::kOff;
    int auto_agitator_count = 0;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(timestamp));
      a->Visit(MJ_NVP(agitator_enabled));
      a->Visit(MJ_NVP(fire_enabled));
      a->Visit(MJ_NVP(fire_count));
      a->Visit(MJ_NVP(imu));
      a->Visit(MJ_NVP(absolute));
      a->Visit(MJ_NVP(target_relative));
      a->Visit(MJ_NVP(target_relative_rate));
      a->Visit(MJ_NVP(target_relative_last_time));
      a->Visit(MJ_NVP(last_sequence));
      a->Visit(MJ_NVP(last_rate));
      a->Visit(MJ_NVP(disable_period_s));
      a->Visit(MJ_NVP(total_fire_time_s));
      a->Visit(MJ_NVP(auto_agitator_end));
      a->Visit(MJ_ENUM(last_fire_command, TurretCommand::Fire::CommandMapper));
      a->Visit(MJ_NVP(auto_agitator_count));
    }
  };

  const Data& data() const;

 private:
  Turret(const boost::asio::executor& executor, bool);

  struct CommandLog {
    boost::posix_time::ptime timestamp;

    TurretCommand command;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(timestamp));
      a->Visit(MJ_NVP(command));
    }
  };

  boost::signals2::signal<void (const Data*)> turret_data_signal_;
  boost::signals2::signal<void (const CommandLog*)> turret_command_signal_;

  class Impl;
  std::unique_ptr<Impl> impl_;
};

}
}
