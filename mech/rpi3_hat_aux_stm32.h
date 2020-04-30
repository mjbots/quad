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

#pragma once

#include <memory>
#include <string>

#include <boost/asio/executor.hpp>

#include "mjlib/io/async_types.h"

#include "mech/attitude_data.h"
#include "mech/aux_stm32.h"

namespace mjmech {
namespace mech {

/// Operate the auxiliary stm32 on the quad pi3 hat.  This includes:
///  * the standard bitrate CAN
///  * the IMU
///  * the RF interface
class Rpi3HatAuxStm32 : public AuxStm32 {
 public:
  struct Mounting {
    double yaw_deg = 0.0;
    double pitch_deg = 90.0;
    double roll_deg = -90.0;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(yaw_deg));
      a->Visit(MJ_NVP(pitch_deg));
      a->Visit(MJ_NVP(roll_deg));
    }
  };

  struct Options {
    int speed = 10000000;
    int cpu_affinity = -1;
    Mounting mounting;
    uint32_t rf_id = 5678;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(speed));
      a->Visit(MJ_NVP(cpu_affinity));
      a->Visit(MJ_NVP(mounting));
      a->Visit(MJ_NVP(rf_id));
    }
  };

  Rpi3HatAuxStm32(const boost::asio::executor&, const Options&);
  ~Rpi3HatAuxStm32() override;

  void AsyncStart(mjlib::io::ErrorCallback);

  // ************************
  // ImuClient

  /// Read the next available IMU sample and store it in @p data.
  ///
  /// @param callback will be invoked when the operation completes or
  /// fails.
  void ReadImu(AttitudeData* data, mjlib::io::ErrorCallback callback) override;

  // ***********************
  // RfClient

  void AsyncWaitForSlot(uint16_t* bitfield, mjlib::io::ErrorCallback) override;

  Slot rx_slot(int slot_idx) override;

  void tx_slot(int slot_id, const Slot&) override;

  Slot tx_slot(int slot_idx) override;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}
}
