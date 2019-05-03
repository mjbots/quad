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

#include "mech/servo_monitor.h"

#include <optional>

#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>

#include <fmt/format.h>

#include "mjlib/base/fail.h"
#include "mjlib/base/system_error.h"
#include "mjlib/io/deadline_timer.h"

#include "base/common.h"
#include "base/logging.h"
#include "base/now.h"

namespace pl = std::placeholders;

namespace mjmech {
namespace mech {

class ServoMonitor::Impl : boost::noncopyable {
 public:
  Impl(ServoMonitor* parent,
       boost::asio::io_service& service,
       ServoInterface* servo)
      : parent_(parent),
        service_(service),
        servo_(servo),
        timer_(service) {}

  void SetupInitialServos() {
    std::vector<int> ids = SplitServoIds(parameters_.servos);

    for (int id: ids) { servo_data_[id]; }
  }

  void Start() {
    timer_.expires_from_now(
        base::ConvertSecondsToDuration(parameters_.period_s));
    timer_.async_wait(std::bind(&Impl::HandleTimeout, this, pl::_1));
  }

  void HandleTimeout(mjlib::base::error_code ec) {
    mjlib::base::FailIf(ec);
    Start();

    log_.debug("HandleTimeout");

    // We shouldn't have any outstanding requests by the time we get
    // here.  If we do, then our timeout is probably too short.
    if (outstanding_) {
      log_.debug("skipping poll because we are too slow");
      return;
    }
    // Pick out the next servo to use.  Find one that exists and isn't
    // on parole.
    std::optional<int> next_servo = FindNext();

    if (!next_servo) {
      // Nothing is ready, let's just stick around in this state until
      // the next polling period.
      log_.debug("nothing is ready");
      return;
    }

    log_.debug(fmt::format("going to query {}", *next_servo));

    state_.last_servo = *next_servo;

    // Request status.
    outstanding_ = true;
    servo_->GetStatus(
        {state_.last_servo},
        []() {
          ServoInterface::StatusOptions s;
          s.temperature = true;
          s.voltage = true;
          s.error = true;
          return s;
        }(),
        std::bind(&Impl::HandleStatus, this,
                  state_.last_servo, pl::_1, pl::_2));
  }

  void HandleStatus(int requested_servo,
                    mjlib::base::error_code ec,
                    const std::vector<ServoInterface::JointStatus>& responses) {
    BOOST_ASSERT(outstanding_);
    outstanding_ = false;

    log_.debug("HandleStatus");

    const auto now = base::Now(service_);
    if (ec == boost::asio::error::operation_aborted) {
      log_.debug(fmt::format("{} servo timed out", requested_servo));
      UpdateServoTimeout(requested_servo, now);
      return;
    }
    mjlib::base::FailIf(ec);

    log_.debug(fmt::format("{} got a result!", requested_servo));

    auto& servo = servo_data_[requested_servo];

    // Yay, we got a result!
    UpdateServoFound(requested_servo, now);
    BOOST_ASSERT(responses.size() == 1);
    const auto& response = responses.front();

    servo.torque_on = response.torque_on;
    if (response.voltage) {
      servo.voltage_V = *response.voltage;
    }
    if (response.temperature_C) {
      servo.temperature_C = *response.temperature_C;
    }

    CheckFaults(response);

    EmitData();
  }

  void UpdateServoTimeout(int requested_servo,
                          boost::posix_time::ptime now) {
    auto& servo = servo_data_[requested_servo];

    // We got a timeout.  Update our parole state.
    if (servo.parole_time_s == 0.0 ||
        servo.next_update.is_not_a_date_time()) {
      servo.parole_time_s = parameters_.parole_min_time_s;
      servo.next_update = now;
    } else {
      servo.parole_time_s = std::min(parameters_.parole_max_time_s,
                                     servo.parole_time_s * 2);
    }

    servo.next_update += base::ConvertSecondsToDuration(servo.parole_time_s);
  }

  void UpdateServoFound(int requested_servo,
                        boost::posix_time::ptime now) {
    auto& servo = servo_data_[requested_servo];

    servo.parole_time_s = 0.0;
    servo.next_update = boost::posix_time::ptime();
    servo.last_update = now;
  }

  std::optional<int> FindNext() const {
    const auto now = base::Now(service_);

    int start = state_.last_servo;
    int current = start;
    do {
      auto it = servo_data_.upper_bound(current);
      if (it != servo_data_.end()) {
        // Is this a valid candidate?
        const auto& servo = it->second;
        if (servo.next_update.is_not_a_date_time() ||
            now > servo.next_update) {
          return it->first;
        }
        // Nope, he is on parole.  Keep looking.
        current = it->first;
      } else {
        // We hit the end, go back to the start.
        current = -1;
      }

    } while (current != start);

    // Oops, nothing is ready.
    return std::nullopt;
  }

  void EmitData() {
    ServoData data;
    data.timestamp = base::Now(service_);

    for (const auto& pair: servo_data_) {
      ServoData::Servo servo;
      servo.address = pair.first;
      servo.last_update = pair.second.last_update;
      servo.voltage_V = pair.second.voltage_V;
      servo.temperature_C = pair.second.temperature_C;
      servo.torque_on = pair.second.torque_on;
      data.servos.push_back(servo);
    }

    parent_->servo_data_signal_(&data);
  }

  void CheckFaults(const ServoInterface::JointStatus& response) {
    if (response.error) {
      log_.warn(fmt::format("ServoMonitor: {}, {:02X}",
                            response.address, response.error));
      servo_->ClearErrors({response.address},
                          std::bind(&Impl::HandleClear, this, pl::_1));
    }

    if (!response.torque_on && expect_torque_on_) {
      log_.warn(fmt::format("{:02X} has motor unexpectedly off!",
                            response.address));
    }
  }

  void HandleClear(mjlib::base::error_code ec) {
    mjlib::base::FailIf(ec);
  }

  ServoMonitor* const parent_;
  boost::asio::io_service& service_;
  ServoInterface* const servo_;

  base::LogRef log_ = base::GetLogInstance("ServoMonitor");

  mjlib::io::DeadlineTimer timer_;

  struct Servo {
    boost::posix_time::ptime last_update;
    double voltage_V = 0;
    double temperature_C = 0;
    bool torque_on = false;

    boost::posix_time::ptime next_update;
    double parole_time_s = 0;
  };

  std::map<int, Servo> servo_data_;

  struct State {
    int last_servo = -1;
  } state_;

  bool outstanding_ = false;

  bool expect_torque_on_ = false;

  Parameters parameters_;
};

ServoMonitor::ServoMonitor(boost::asio::io_service& service,
                           ServoInterface* servo)
    : impl_(new Impl(this, service, servo)) {}

ServoMonitor::~ServoMonitor() {}

void ServoMonitor::AsyncStart(mjlib::io::ErrorCallback handler) {
  impl_->SetupInitialServos();
  impl_->Start();
  impl_->service_.post(std::bind(handler, mjlib::base::error_code()));
}

ServoMonitor::Parameters* ServoMonitor::parameters() {
  return &impl_->parameters_;
}

void ServoMonitor::ExpectTorqueOn() {
  impl_->expect_torque_on_ = true;
}

void ServoMonitor::ExpectTorqueOff() {
  impl_->expect_torque_on_ = false;
}

std::vector<int> ServoMonitor::SplitServoIds(const std::string& input) {
  // Make a single query of a bunch of servo IDs so that the
  // ServoMonitor will know who to talk to.
  std::vector<std::string> fields;
  boost::split(fields, input, boost::is_any_of(","));
  std::vector<int> ids;
  for (const auto& field: fields) {
    if (field.empty()) { continue; }
    size_t pos = field.find_first_of('-');
    if (pos == std::string::npos) {
      ids.push_back(std::stoi(field));
    } else {
      int start = std::stoi(field.substr(0, pos));
      int end = std::stoi(field.substr(pos + 1));
      if (start < 0 || start > 254 ||
          end < 0 || end > 254) {
        throw mjlib::base::system_error::einval("invalid servo spec");
      }
      for (int i = start; i <= end; i++) { ids.push_back(i); }
    }
  }

  return ids;
}

}
}
