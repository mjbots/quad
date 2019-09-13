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
    servo_->SJog(targets, parameters_.pose_time_s, handler);
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

    HandlePowerAck(mjlib::base::error_code(), ids, value, handler);
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

    servo_->RamWrite(to_send, HC::torque_control(), value,
                     std::bind(&HerkuleXServoInterface::HandlePowerAck, this,
                               std::placeholders::_1,
                               new_ids, value, handler));
  }

  virtual void GetPose(
      const std::vector<int>& ids, PoseHandler handler) override {
    DoGetPose(mjlib::base::error_code(), 0,
              std::nullopt,
              ids, std::vector<Joint>{}, handler);
  }

  void DoGetPose(mjlib::base::error_code ec,
                 int value,
                 std::optional<int> address,
                 const std::vector<int>& ids,
                 const std::vector<Joint>& current_result,
                 PoseHandler handler) {
    if (ec && ec != boost::asio::error::operation_aborted) {
      if (address) {
        ec.Append(fmt::format("when getting pose from servo {}", *address));
      }
      handler(ec, {});
      return;
    }

    auto result = current_result;
    if (!ec && address) {
      result.emplace_back(Joint{*address, HerkuleXBase::CountsToAngleDeg(value)});
    }
    if (ids.empty()) { handler(mjlib::base::error_code(), result); return; }

    int to_send = ids.back();
    std::vector<int> new_ids = ids;
    new_ids.pop_back();
    servo_->RamRead(to_send, HC::position(),
                    std::bind(&HerkuleXServoInterface::DoGetPose, this,
                              std::placeholders::_1,
                              std::placeholders::_2,
                              to_send,
                              new_ids, result, handler));
  }

  virtual void GetTemperature(
      const std::vector<int>& ids, TemperatureHandler handler) override {
    DoGetTemperature(mjlib::base::error_code(), 0,
                     std::nullopt,
                     ids, std::vector<Temperature>{}, handler);
  }

  void DoGetTemperature(mjlib::base::error_code ec,
                        int value,
                        std::optional<int> address,
                        const std::vector<int>& ids,
                        const std::vector<Temperature>& current_result,
                        TemperatureHandler handler) {
    if (ec && ec != boost::asio::error::operation_aborted) {
      if (address) {
        ec.Append(fmt::format("when getting temp from servo {}", *address));
      }
      handler(ec, {});
      return;
    }

    auto result = current_result;
    if (!ec && address) {
      result.emplace_back(
          Temperature{*address, HerkuleXBase::CountsToTemperatureC(value)});
    }
    if (ids.empty()) { handler(mjlib::base::error_code(), result); return; }

    int to_send = ids.back();
    std::vector<int> new_ids = ids;
    new_ids.pop_back();
    servo_->RamRead(to_send, HC::temperature_c(),
                    std::bind(&HerkuleXServoInterface::DoGetTemperature, this,
                              std::placeholders::_1,
                              std::placeholders::_2,
                              to_send,
                              new_ids, result, handler));
  }

  virtual void GetVoltage(
      const std::vector<int>& ids, VoltageHandler handler) override {
    DoGetVoltage(mjlib::base::error_code(), 0,
                 std::nullopt,
                 ids, std::vector<Voltage>{}, handler);
  }

  static std::string FormatIDs(const std::vector<int>& ids) {
    std::string result;
    result += "[";
    for (int id: ids) { result += fmt::format("{}", id); }
    result += "]";
    return result;
  }

  void DoGetVoltage(mjlib::base::error_code ec,
                    int value,
                    std::optional<int> address,
                    const std::vector<int>& ids,
                    const std::vector<Voltage>& current_result,
                    VoltageHandler handler) {
    if (ec && ec != boost::asio::error::operation_aborted) {
      if (address) {
        ec.Append(fmt::format("when getting voltage from servo  {}",
                              *address));
      }
      handler(ec, {});
      return;
    }

    auto result = current_result;
    if (!ec && address) {
      result.emplace_back(
          Voltage{*address, HerkuleXBase::CountsToVoltage(value)});
    }

    if (ids.empty()) { handler(mjlib::base::error_code(), result); return; }

    int to_send = ids.back();
    std::vector<int> new_ids = ids;
    new_ids.pop_back();
    servo_->RamRead(to_send, HC::voltage(),
                    std::bind(&HerkuleXServoInterface::DoGetVoltage, this,
                              std::placeholders::_1,
                              std::placeholders::_2,
                              to_send,
                              new_ids, result, handler));
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
