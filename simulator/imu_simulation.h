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

#pragma once

#include <boost/asio/io_service.hpp>

#include "base/deadline_timer.h"
#include "base/error_code.h"

namespace mjmech {
namespace simulator {

class I2CDevice {
 public:
  virtual uint8_t address() const = 0;
  virtual uint8_t Read(uint8_t reg) = 0;
  virtual void Write(uint8_t reg, uint8_t value) = 0;
};

class MAX21000 : public I2CDevice {
 public:
  MAX21000(boost::asio::io_service& service,
           dart::dynamics::Frame* frame)
      : service_(service),
        timer_(service_),
        frame_(frame) {}

  uint8_t address() const override { return 0x59; }

  uint8_t Read(uint8_t reg) {
    switch (reg) {
      case 0x20: { return 0xb1; }
      case 0x22: {
        const uint8_t result = data_ ? 0x0f : 0x00;
        data_ = false;
        return result;
      }
    }
    return 0;
  }

  void Write(uint8_t reg, uint8_t value) {
    switch (reg) {
      case 0x00: {
        enabled_ = (value & 0x0f) == 0x0f;
        const int fs = value >> 6;
        full_scale_dps_ = [&]() {
          switch (fs) {
            case 0: { return 2000.0; }
            case 1: { return 1000.0; }
            case 2: { return 500.0; }
            case 3: { return 250.0; }
          }
          return 0.0;
        }();
        break;
      }
      case 0x01: {
        // Set the bandwidth.  Ignore for now.
        break;
      }
      case 0x02: {
        rate_hz_ = [&]() {
          if (value <= 99) {
            return 10000.0 / (value + 1);
          } else if (value <= 179) {
            return 10000.0 / (100 + 5 * (value - 99));
          } else {
            return 10000.0 / (500 + 20 * (value - 179));
          }
        }();
        break;
      }
    }
    if (enabled_ && rate_hz_ != 0.0) {
      timer_.expires_at(base::Now(service_));
      StartTimer();
    }
  }

  void StartTimer() {
    timer_.expires_at(
        timer_.expires_at() +
        base::ConvertSecondsToDuration(1.0 / rate_hz_));
    timer_.async_wait(
        std::bind(&MAX21000::HandleTimer, this,
                  std::placeholders::_1));
  }

  void HandleTimer(base::ErrorCode ec) {
    if (ec == boost::asio::error::operation_aborted) { return; }
    if (enabled_ && rate_hz_ != 0.0) { StartTimer(); }
    Sample();
  }

  void Sample() {
    data_ = true;
  }

 private:
  boost::asio::io_service& service_;
  base::DeadlineTimer timer_;
  dart::dynamics::Frame* const frame_;
  double full_scale_dps_ = 0.0;
  bool enabled_ = false;
  double rate_hz_ = 0.0;

  bool data_ = false;
};

class MMA8451Q : public I2CDevice {
 public:
  MMA8451Q(boost::asio::io_service& service,
           dart::dynamics::Frame* frame,
           const Eigen::Vector3d& offset)
      : service_(service),
        timer_(service_),
        frame_(frame),
        offset_(offset) {}

  uint8_t address() const override { return 0x1d; }

  uint8_t Read(uint8_t reg) {
    const auto convert = [this](double value) {
      return std::max(-32768,
                      std::min(32767,
                               static_cast<int>(value / sensitivity_)));
    };

    switch (reg) {
      case 0x00: {
        const uint8_t result = data_ ? 0x0f : 0x00;
        data_ = false;
        return result;
      }
      case 0x01: {
        const int16_t value = convert(measurement_g_.x);
        return value >> 8;
      }
      case 0x02: {
        const int16_t value = convert(measurement_g_.x);
        return value & 0xff;
      }

      case 0x03: {
        const int16_t value = convert(measurement_g_.y);
        return value >> 8;
      }
      case 0x04: {
        const int16_t value = convert(measurement_g_.y);
        return value & 0xff;
      }

      case 0x05: {
        const int16_t value = convert(measurement_g_.z);
        return value >> 8;
      }
      case 0x06: {
        const int16_t value = convert(measurement_g_.z);
        return value & 0xff;
      }

    }
    return 0;
  }

  void Write(uint8_t reg, uint8_t value) {
    switch (reg) {
      case 0x0e: {
        const int fs = value & 0x03;
        sensitivity_ = [&]() {
          switch (fs) {
            case 0: { return 1.0 / 4096 / 4; }
            case 1: { return 1.0 / 2048 / 4; }
            case 2: { return 1.0 / 1024 / 4; }
          }
          return 0.0;
        }();
        break;
      }
      case 0x2a: {
        enabled_ = (value & 0x01) ? true : false;
        const int dr = (value >> 3) & 0x07;
        rate_hz_ = [&]() {
          switch (dr) {
            case 0: { return 800.0; }
            case 1: { return 400.0; }
            case 2: { return 200.0; }
            case 3: { return 100.0; }
            case 4: { return 50.0; }
            case 5: { return 12.5; }
            case 6: { return 6.25; }
            case 7: { return 1.56; }
          }
          return 0.0;
        }();
        break;
      }
    }

    if (rate_hz_ != 0.0 && enabled_) {
      timer_.expires_at(base::Now(service_));
      StartTimer();
    }
  }

  void StartTimer() {
    timer_.expires_at(timer_.expires_at() +
                      base::ConvertSecondsToDuration(1.0 / rate_hz_));
    timer_.async_wait(
        std::bind(&MMA8451Q::HandleTimer, this,
                  std::placeholders::_1));
  }

  void HandleTimer(base::ErrorCode ec) {
    if (ec == boost::asio::error::operation_aborted) { return; }
    if (enabled_ && rate_hz_ != 0.0) { StartTimer(); }
    Sample();
  }

  void Sample() {
    data_ = true;
    measurement_g_.x = 0.0;
    measurement_g_.y = 0.0;
    measurement_g_.z = 1.0;
  }

 private:
  boost::asio::io_service& service_;
  base::DeadlineTimer timer_;
  dart::dynamics::Frame* const frame_;
  const Eigen::Vector3d offset_;

  double rate_hz_ = 0.0;
  bool enabled_ = false;
  double sensitivity_ = 0.0;
  bool data_ = false;

  base::Point3D measurement_g_;
};


}
}
