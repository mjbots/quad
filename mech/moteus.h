// Copyright 2015-2020 Josh Pieper, jjp@pobox.com.  All rights reserved.
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

#include "mjlib/base/limit.h"
#include "mjlib/multiplex/format.h"

namespace mjmech {
namespace mech {
namespace moteus {

enum {
  kCurrentRegisterMapVersion = 3,
};

enum Ids : uint8_t {
  kBroadcastId = 0x7f,
};

enum Register : uint32_t {
  kMode = 0x000,
  kPosition = 0x001,
  kVelocity = 0x002,
  kTorque = 0x003,
  kQCurrent = 0x004,
  kDCurrent = 0x005,
  kRezeroState = 0x00c,
  kVoltage = 0x00d,
  kTemperature = 0x00e,
  kFault = 0x00f,

  kPwmPhaseA = 0x010,
  kPwmPhaseB = 0x011,
  kPwmPhaseC = 0x012,

  kVoltagePhaseA = 0x014,
  kVoltagePhaseB = 0x015,
  kVoltagePhaseC = 0x016,

  kVFocTheta = 0x018,
  kVFocVoltage = 0x019,
  kVoltageDqD = 0x01a,
  kVoltageDqQ = 0x01b,

  kCommandQCurrent = 0x01c,
  kCommandDCurrent = 0x01d,

  kCommandPosition = 0x020,
  kCommandVelocity = 0x021,
  kCommandFeedforwardTorque = 0x022,
  kCommandKpScale = 0x023,
  kCommandKdScale = 0x024,
  kCommandPositionMaxCurrent = 0x025,
  kCommandStopPosition = 0x026,
  kCommandTimeout = 0x027,

  kRegisterMapVersion = 0x102,
  kSerialNumber = 0x120,
  kSerialNumber1 = 0x120,
  kSerialNumber2 = 0x121,
  kSerialNumber3 = 0x122,
};

enum class Mode {
    kStopped = 0,
    kFault = 1,
    kEnabling = 2,
    kCalibrating = 3,
    kCalibrationComplete = 4,
    kPwm = 5,
    kVoltage = 6,
    kVoltageFoc = 7,
    kVoltageDq = 8,
    kCurrent = 9,
    kPosition = 10,
    kPositionTimeout = 11,
    kZeroVelocity = 12,
};

using Value = mjlib::multiplex::Format::Value;

template <typename T>
Value ScaleSaturate(double value, double scale) {
  if (!std::isfinite(value)) {
    return std::numeric_limits<T>::min();
  }

  const double scaled = value / scale;
  const auto max = std::numeric_limits<T>::max();
  // We purposefully limit to +- max, rather than to min.  The minimum
  // value for our two's complement types is reserved for NaN.
  return mjlib::base::Limit<T>(static_cast<T>(scaled), -max, max);
}

enum RegisterTypes {
  kInt8 = 0,
  kInt16 = 1,
  kInt32 = 2,
  kFloat = 3,
};

inline Value ScaleMapping(double value,
                   double int8_scale, double int16_scale, double int32_scale,
                   RegisterTypes type) {
  switch (type) {
    case kInt8: return ScaleSaturate<int8_t>(value, int8_scale);
    case kInt16: return ScaleSaturate<int16_t>(value, int16_scale);
    case kInt32: return ScaleSaturate<int32_t>(value, int32_scale);
    case kFloat: return static_cast<float>(value);
  }
  MJ_ASSERT(false);
  return Value(static_cast<int8_t>(0));
}

inline Value WritePosition(double value, RegisterTypes reg) {
  return ScaleMapping(value / 360.0, 0.01, 0.001, 0.00001, reg);
}

inline Value WriteVelocity(double value, RegisterTypes reg) {
  return ScaleMapping(value / 360.0, 0.1, 0.001, 0.00001, reg);
}

inline Value WriteTorque(double value, RegisterTypes reg) {
  return ScaleMapping(value, 0.5, 0.01, 0.001, reg);
}

inline Value WritePwm(double value, RegisterTypes reg) {
  return ScaleMapping(value, 1.0 / 127.0,
                      1.0 / 32767.0,
                      1.0 / 2147483647.0,
                      reg);
}

struct ValueScaler {
  double int8_scale;
  double int16_scale;
  double int32_scale;

  double operator()(int8_t value) const {
    if (value == std::numeric_limits<int8_t>::min()) {
      return std::numeric_limits<double>::quiet_NaN();
    }
    return value * int8_scale;
  }

  double operator()(int16_t value) const {
    if (value == std::numeric_limits<int16_t>::min()) {
      return std::numeric_limits<double>::quiet_NaN();
    }
    return value * int16_scale;
  }

  double operator()(int32_t value) const {
    if (value == std::numeric_limits<int32_t>::min()) {
      return std::numeric_limits<double>::quiet_NaN();
    }
    return value * int32_scale;
  }

  double operator()(float value) const {
    return value;
  }
};

inline double ReadScale(Value value,
                        double int8_scale,
                        double int16_scale,
                        double int32_scale) {
  return std::visit(ValueScaler{int8_scale, int16_scale, int32_scale}, value);
}

inline double ReadPosition(Value value) {
  return ReadScale(value, 0.01, 0.001, 0.00001) * 360.0;
}

inline double ReadTorque(Value value) {
  return ReadScale(value, 0.5, 0.01, 0.001);
}

inline double ReadVoltage(Value value) {
  return ReadScale(value, 0.5, 0.1, 0.001);
}

inline double ReadTemperature(Value value) {
  return ReadScale(value, 1.0, 0.1, 0.001);
}

inline int ReadInt(Value value) {
  return std::visit([](auto v) { return static_cast<int>(v); }, value);
}

}
}
}
