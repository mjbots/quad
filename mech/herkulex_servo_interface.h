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

#pragma once

#include <optional>

#include <fmt/format.h>

#include "mjlib/base/fail.h"
#include "mjlib/base/program_options_archive.h"

#include "servo_interface.h"

namespace mjmech {
namespace mech {
template <typename Servo>
class HerkuleXServoInterface : public ServoInterface {
 public:
  typedef HerkuleXConstants HC;

  HerkuleXServoInterface(Servo* servo) : servo_(servo) {
    mjlib::base::ProgramOptionsArchive(&options_).Accept(&parameters_);
  }

  ~HerkuleXServoInterface() override {}

  Servo* servo() { return servo_; }
  const Servo* servo() const { return servo_; }

  struct Parameters {
    double pose_time_s = 0.1;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(pose_time_s));
    }
  };

  Parameters* parameters() { return &parameters_; }
  boost::program_options::options_description* options() { return &options_; }

  virtual void SetPose(const std::vector<Joint>& joints,
                       mjlib::io::ErrorCallback handler) override {
    struct Target {
      uint8_t address;
      uint16_t position;
      uint8_t leds;
    };
    auto get_angle = [&](const auto& joint) {
      if (std::isfinite(joint.goal_deg)) {
        return joint.goal_deg;
      }
      return joint.angle_deg;
    };
    std::vector<Target> targets;
    for (const auto& joint: joints) {
      targets.emplace_back(Target{
          MapAddress(joint.address),
              HerkuleXBase::AngleToCount(get_angle(joint)), 0});
    }
    servo_->SJog(targets, parameters_.pose_time_s, std::move(handler));
  }

  virtual void EnablePower(PowerState power_state,
                           const std::vector<int>& ids_in,
                           mjlib::io::ErrorCallback handler) override {
    uint8_t value = [power_state]() {
      switch (power_state) {
        case kPowerFree: { return 0x00; }
        case kPowerBrake: { return 0x40; }
        case kPowerEnable: { return 0x60; }
      }
      BOOST_ASSERT(false);
    }();

    std::vector<int> ids(ids_in);
    if (ids.empty()) { ids.push_back(Servo::BROADCAST); }

    HandlePowerAck(mjlib::base::error_code(), ids, value, std::move(handler));
  }

  void HandlePowerAck(mjlib::base::error_code ec,
                      const std::vector<int>& ids,
                      uint8_t value,
                      mjlib::io::ErrorCallback handler) {
    if (ec) { handler(ec); return; }

    if (ids.empty()) {
      handler(mjlib::base::error_code());
      return;
    }

    std::vector<int> new_ids = ids;
    int to_send = new_ids.back();
    new_ids.pop_back();

    servo_->RamWrite(
        to_send, HC::torque_control(), value,
        [this, new_ids, value, handler=std::move(handler)](const auto& _1) mutable {
          this->HandlePowerAck(_1, new_ids, value, std::move(handler));
        });
  }

  static std::string FormatIDs(const std::vector<int>& ids) {
    std::string result;
    result += "[";
    for (int id: ids) { result += fmt::format("{}", id); }
    result += "]";
    return result;
  }

  void GetStatus(const std::vector<int>&,
                 const StatusOptions&,
                 StatusHandler) override {
    mjlib::base::AssertNotReached();
  }

  void ClearErrors(const std::vector<int>&,
                   mjlib::io::ErrorCallback) override {
    mjlib::base::AssertNotReached();
  }

  void Update(PowerState,
              const StatusOptions&,
              const std::vector<Joint>* command,
              std::vector<JointStatus>* result,
              mjlib::io::ErrorCallback) override {
    mjlib::base::AssertNotReached();
  }

 private:
  static uint8_t MapAddress(int address) {
    if (address < 0 || address > 0xfe) {
      mjlib::base::Fail("invalid address");
    }
    return static_cast<uint8_t>(address);
  }

  Parameters parameters_;
  boost::program_options::options_description options_;
  Servo* const servo_;
};
}
}
