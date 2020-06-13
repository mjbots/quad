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

#include "mjlib/base/visitor.h"
#include "mjlib/io/async_types.h"
#include "mjlib/multiplex/asio_client.h"
#include "mjlib/multiplex/register.h"

#include "mech/attitude_data.h"
#include "mech/pi3hat_interface.h"

namespace mjmech {
namespace mech {

/// Provide an interface to the pi3hat.
///
/// NOTE: This treates the AsioClient as the primary interface.  IMU
/// and RF requests will only be serviced when AsyncRegister or
/// AsyncRegisterMultiple are called.
class Pi3hatWrapper : public Pi3hatInterface {
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
    // If set to a non-negative number, bind the time sensitive thread
    // to the given CPU.
    int cpu_affinity = -1;

    int spi_speed_hz = 10000000;

    // When waiting for CAN data, wait this long before timing out.
    double query_timeout_s = 0.001;

    Mounting mounting;
    uint32_t rf_id = 5678;
    double power_poll_period_s = 0.1;
    double shutdown_timeout_s = 15.0;
    uint32_t imu_rate_hz = 400;
    bool attitude_detail = false;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(cpu_affinity));
      a->Visit(MJ_NVP(spi_speed_hz));
      a->Visit(MJ_NVP(query_timeout_s));
      a->Visit(MJ_NVP(mounting));
      a->Visit(MJ_NVP(rf_id));
      a->Visit(MJ_NVP(power_poll_period_s));
      a->Visit(MJ_NVP(shutdown_timeout_s));
      a->Visit(MJ_NVP(imu_rate_hz));
      a->Visit(MJ_NVP(attitude_detail));
    }
  };

  Pi3hatWrapper(const boost::asio::executor&, const Options&);
  ~Pi3hatWrapper();

  void AsyncStart(mjlib::io::ErrorCallback);

  // ************************
  // mp::AsioClient

  /// Request a request be made to one or more servos (and optionally
  /// have a reply sent back).
  void AsyncRegister(const IdRequest&, SingleReply*,
                     mjlib::io::ErrorCallback) override;

  void AsyncRegisterMultiple(const std::vector<IdRequest>&,
                             Reply*,
                             mjlib::io::ErrorCallback) override;

  mjlib::io::SharedStream MakeTunnel(
      uint8_t id,
      uint32_t channel,
      const TunnelOptions& options) override;

  struct Stats {
    template <typename Archive>
    void Serialize(Archive* a) {
    }
  };
  Stats stats() const;

  // ************************
  // ImuClient

  /// Read the next available IMU sample and store it in @p data.
  ///
  /// @param callback will be invoked when the operation completes or
  /// fails.
  void ReadImu(AttitudeData* data, mjlib::io::ErrorCallback callback) override;

  // ***********************
  // RfClient

  void AsyncWaitForSlot(
      int* remote, uint16_t* bitfield, mjlib::io::ErrorCallback) override;

  Slot rx_slot(int remote, int slot_idx) override;

  void tx_slot(int remote, int slot_id, const Slot&) override;

  Slot tx_slot(int remote, int slot_idx) override;

  void Cycle(AttitudeData*,
             const std::vector<IdRequest>& request,
             Reply* reply,
             mjlib::io::ErrorCallback callback) override;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}
}
