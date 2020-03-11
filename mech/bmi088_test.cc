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

#include <fmt/format.h>

#include <boost/program_options.hpp>

#include "mjlib/base/time_conversions.h"
#include "mjlib/telemetry/file_writer.h"

#include "base/telemetry_log_registrar.h"

#include "mech/attitude_data.h"
#include "mech/imu_data.h"
#include "mech/rpi3_raw_spi.h"

namespace po = boost::program_options;

namespace mjmech {
namespace mech {

int do_main(int argc, char** argv) {
  std::string spi_device = "/dev/spidev0.0";
  int speed = 10000000;
  std::string log_file = "";

  po::options_description desc("Allowable options");

  desc.add_options()
      ("help,h", "display usage message")
      ("device,d", po::value(&spi_device), "SPI device")
      ("speed,s", po::value(&speed), "SPI speed")
      ("log,l", po::value(&log_file), "log file name")
      ;

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);

  if (vm.count("help")) {
    std::cout << desc;
    return 0;
  }

  mjlib::telemetry::FileWriter log{[]() {
      mjlib::telemetry::FileWriter::Options options;
      options.blocking = false;
      return options;
    }()};
  base::TelemetryLogRegistrar registrar{&log};

  boost::signals2::signal<void(const ImuData*)> imu_signal;
  ImuData imu_data;
  boost::signals2::signal<void(const AttitudeData*)> att_signal;
  AttitudeData att_data;

  registrar.Register("imu", &imu_signal);
  registrar.Register("attitude", &att_signal);

  if (!log_file.empty()) { log.Open(log_file); }

  mech::Rpi3RawSpi spi([&]() {
      mech::Rpi3RawSpi::Options options;
      options.speed_hz = speed;
      return options;
    }());

  struct Bmi088Data {
    uint16_t present = 0;
    float gx_dps = 0;
    float gy_dps = 0;
    float gz_dps = 0;
    float ax_m_s2 = 0;
    float ay_m_s2 = 0;
    float az_m_s2 = 0;
  } __attribute__((packed));

  struct AttitudeData {
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
  } __attribute__((packed));

  Bmi088Data bmi088_data;
  AttitudeData attitude;
  boost::posix_time::ptime last_update;

  while (true) {
    bmi088_data = {};
    spi.Read(0, 33, mjlib::base::string_span(
                 reinterpret_cast<char*>(&bmi088_data), sizeof(bmi088_data)));
    if ((bmi088_data.present & 0x01) == 0) {
      ::usleep(50);
      continue;
    }

    spi.Read(0, 34, mjlib::base::string_span(
                 reinterpret_cast<char*>(&attitude), sizeof(attitude)));

    const auto now = boost::posix_time::microsec_clock::universal_time();
    imu_data.timestamp = now;
    imu_data.rate_dps = { bmi088_data.gx_dps,
                          bmi088_data.gy_dps,
                          bmi088_data.gz_dps };
    imu_data.accel_mps2 = { bmi088_data.ax_m_s2,
                            bmi088_data.ay_m_s2,
                            bmi088_data.az_m_s2 };
    imu_signal(&imu_data);

    att_data.timestamp = now;
    att_data.attitude = {attitude.w, attitude.x, attitude.y, attitude.z};
    att_data.rate_dps = {attitude.x_dps, attitude.y_dps, attitude.z_dps};
    att_data.euler_deg = (180.0 / M_PI) * att_data.attitude.euler_rad();
    att_data.accel_mps2 = { attitude.a_x_mps2, attitude.a_y_mps2, attitude.a_z_mps2 };

    att_data.bias_dps = {attitude.bias_x_dps, attitude.bias_y_dps, attitude.bias_z_dps};
    att_data.attitude_uncertainty = {
      attitude.uncertainty_w,
      attitude.uncertainty_x,
      attitude.uncertainty_y,
      attitude.uncertainty_z,
    };
    att_data.bias_uncertainty_dps = {
      attitude.uncertainty_bias_x_dps,
      attitude.uncertainty_bias_y_dps,
      attitude.uncertainty_bias_z_dps,
    };

    att_signal(&att_data);

    if (last_update.is_not_a_date_time() ||
        mjlib::base::ConvertDurationToSeconds(now - last_update) > 0.1) {
      std::cout << fmt::format(
          "y={:5.1f} p={:5.1f} r={:5.1f}  dps=({:5.1f},{:5.1f},{:5.1f}) "
          "a=({:4.1f},{:4.1f},{:4.1f})  \r",
          att_data.euler_deg.yaw,
          att_data.euler_deg.pitch,
          att_data.euler_deg.roll,
          att_data.rate_dps.x(),
          att_data.rate_dps.y(),
          att_data.rate_dps.z(),
          imu_data.accel_mps2.x(),
          imu_data.accel_mps2.y(),
          imu_data.accel_mps2.z());
      std::cout.flush();
    }

    if (std::abs(att_data.attitude.norm() - 1.0) > 1.0) {
      std::cout << "\n\nBAD DATA\n\n";
    }
  }

  ::usleep(50);
  return 0;
}

}
}


int main(int argc, char** argv) {
  return mjmech::mech::do_main(argc, argv);
}
