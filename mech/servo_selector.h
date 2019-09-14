// Copyright 2019 Josh Pieper, jjp@pobox.com.  All rights reserved.
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

#include <boost/program_options.hpp>

#include "mjlib/base/program_options_archive.h"

#include "mech/servo_interface.h"

namespace mjmech {
namespace mech {

class ServoSelector : public ServoInterface {
 public:
  ServoSelector() {
    mjlib::base::ProgramOptionsArchive(&options_).Accept(&parameters_);
  }
  ~ServoSelector() override {}

  struct Parameters {
    std::string servo;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(servo));
    }
  };

  Parameters* parameters() { return &parameters_; }
  boost::program_options::options_description* options() { return &options_; }

  void AsyncStart(mjlib::io::ErrorCallback handler) {
    BOOST_ASSERT(!servo_types_.empty());
    if (parameters_.servo.empty()) {
      selected_ = servo_types_.begin()->second;
    } else {
      auto it = servo_types_.find(parameters_.servo);
      if (it == servo_types_.end()) {
        mjlib::base::Fail("Unknown servo type: " + parameters_.servo);
      }
      selected_ = it->second;
    }

    handler(mjlib::base::error_code());
  }

  void AddInterface(const std::string& name, ServoInterface* servo) {
    servo_types_[name] = servo;
  }

  void SetPose(const std::vector<Joint>& data,
               mjlib::io::ErrorCallback handler) override {
    selected_->SetPose(data, handler);
  }

  void EnablePower(PowerState power_state, const std::vector<int>& ids,
                   mjlib::io::ErrorCallback handler) override {
    selected_->EnablePower(power_state, ids, handler);
  }

  void GetStatus(const std::vector<int>& ids,
                 const StatusOptions& status_options,
                 StatusHandler handler) override {
    selected_->GetStatus(ids, status_options, handler);
  }

  void ClearErrors(const std::vector<int>& ids,
                   mjlib::io::ErrorCallback callback) override {
    selected_->ClearErrors(ids, callback);
  }

  void Update(
      PowerState power_state,
      const StatusOptions& status_options,
      const std::vector<Joint>* command,
      std::vector<JointStatus>* result,
      mjlib::io::ErrorCallback callback) override {
    selected_->Update(power_state, status_options, command,
                      result, callback);
  }

 private:
  Parameters parameters_;
  boost::program_options::options_description options_;
  std::map<std::string, ServoInterface*> servo_types_;
  ServoInterface* selected_ = nullptr;
};

}
}
