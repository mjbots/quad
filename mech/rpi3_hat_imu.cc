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

#include "mech/rpi3_hat_imu.h"

#include <sched.h>
#include <linux/serial.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/select.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <thread>

#include <boost/asio/io_context.hpp>
#include <boost/asio/post.hpp>

#include "mjlib/base/json5_write_archive.h"
#include "mjlib/base/string_span.h"
#include "mjlib/base/system_error.h"
#include "mjlib/io/now.h"

#include "mech/rpi3_raw_spi.h"

namespace mjmech {
namespace mech {

namespace {
/// This is the format exported by register 34 on the hat.
struct DeviceAttitudeData {
  uint8_t present = 0;
  uint8_t update_time_10us = 0;
  float w = 0;
  float x = 0;
  float y = 0;
  float z = 0;
  float x_dps = 0;
  float y_dps = 0;
  float z_dps = 0;
  float a_x_mps2 = 0;
  float a_y_mps2 = 0;
  float a_z_mps2 = 0;
  float bias_x_dps = 0;
  float bias_y_dps = 0;
  float bias_z_dps = 0;
  float uncertainty_w = 0;
  float uncertainty_x = 0;
  float uncertainty_y = 0;
  float uncertainty_z = 0;
  float uncertainty_bias_x_dps = 0;
  float uncertainty_bias_y_dps = 0;
  float uncertainty_bias_z_dps = 0;
  uint8_t padding[4] = {};
} __attribute__((packed));
}

class Rpi3HatImu::Impl {
 public:
  Impl(const boost::asio::executor& executor, const Options& options)
      : executor_(executor),
        options_(options) {
    thread_ = std::thread(std::bind(&Impl::Run, this));
  }

  ~Impl() {
    child_context_.stop();
    thread_.join();
  }

  void AsyncStart(mjlib::io::ErrorCallback callback) {
    boost::asio::post(
        executor_,
        std::bind(std::move(callback), mjlib::base::error_code()));
  }

  void ReadImu(AttitudeData* data, mjlib::io::ErrorCallback callback) {
    boost::asio::post(
        child_context_,
        [this, data, callback=std::move(callback)]() mutable {
          this->Child_ReadImu(data, std::move(callback));
        });
  }

 private:
  void Run() {
    {
      struct sched_param param = {};
      param.sched_priority = 99;

      mjlib::base::system_error::throw_if(
          ::sched_setscheduler(0, SCHED_RR, &param) < 0,
          "error setting real time");
    }
    if (options_.cpu_affinity >= 0) {
      cpu_set_t cpuset = {};
      CPU_ZERO(&cpuset);
      CPU_SET(options_.cpu_affinity, &cpuset);

      mjlib::base::system_error::throw_if(
          ::sched_setaffinity(0, sizeof(cpu_set_t), &cpuset) < 0,
          "error setting affinity");
    }

    spi_ = std::make_unique<Rpi3RawSpi>([&]() {
        Rpi3RawSpi::Options options;
        options.speed_hz = options_.speed;
        return options;
      }());

    boost::asio::io_context::work work(child_context_);
    child_context_.run();
  }

  void Child_ReadImu(AttitudeData* data, mjlib::io::ErrorCallback callback) {
    // Busy loop until data is available.
    while (true) {
      device_data_ = {};

      while ((device_data_.present & 0x01) == 0) {
        spi_->Read(0, 34, mjlib::base::string_span(
                       reinterpret_cast<char*>(&device_data_), sizeof(device_data_)));
      }

      // TODO: The child context won't have the correct time in a debug
      // time world.
      data->timestamp = mjlib::io::Now(child_context_);
      const auto& dd = device_data_;
      data->attitude = { dd.w, dd.x, dd.y, dd.z };
      data->rate_dps = { dd.x_dps, dd.y_dps, dd.z_dps };
      data->euler_deg = (180.0 / M_PI) * data->attitude.euler_rad();
      data->accel_mps2 = { dd.a_x_mps2, dd.a_y_mps2, dd.a_z_mps2 };
      data->bias_dps = { dd.bias_x_dps, dd.bias_y_dps, dd.bias_z_dps };
      data->attitude_uncertainty = {
        dd.uncertainty_w,
        dd.uncertainty_x,
        dd.uncertainty_y,
        dd.uncertainty_z,
      };
      data->bias_uncertainty_dps = {
        dd.uncertainty_bias_x_dps,
        dd.uncertainty_bias_y_dps,
        dd.uncertainty_bias_z_dps,
      };

      if (std::abs(data->attitude.norm() - 1.0) > 1.0) {
        if (0) {
          // This data is probably corrupt.  Exit now so that we can look
          // at the problem on a scope.
          std::cerr << "Corrupt IMU data received!\n";
          for (size_t i = 0; i < sizeof(device_data_); i++) {
            std::cerr << fmt::format(
                "{:02x} ",
                reinterpret_cast<const uint8_t*>(&device_data_)[i]);
          }
          std::cerr << "\n";
          std::cerr << mjlib::base::Json5WriteArchive::Write(*data) << "\n";

          std::exit(1);
        } else {
          // Lets just try this again.
          continue;
        }
      }
      break;
    }

    boost::asio::post(
        executor_,
        std::bind(std::move(callback), mjlib::base::error_code()));
  }

  boost::asio::executor executor_;
  const Options options_;

  std::thread thread_;

  // Only accessed from child thread.
  boost::asio::io_context child_context_;
  std::unique_ptr<Rpi3RawSpi> spi_;
  DeviceAttitudeData device_data_;
};

Rpi3HatImu::Rpi3HatImu(const boost::asio::executor& executor, const Options& options)
    : impl_(std::make_unique<Impl>(executor, options)) {}

Rpi3HatImu::~Rpi3HatImu() {}

void Rpi3HatImu::AsyncStart(mjlib::io::ErrorCallback callback) {
  impl_->AsyncStart(std::move(callback));
}

void Rpi3HatImu::ReadImu(AttitudeData* data, mjlib::io::ErrorCallback callback) {
  impl_->ReadImu(data, std::move(callback));
}

}
}
