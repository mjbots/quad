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

#include <boost/program_options.hpp>

#include "base/telemetry_log_registrar.h"

#include "mech/imu_data.h"
#include "mech/spidev.h"

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

  mech::SpiDev spi{spi_device, speed};

  base::TelemetryLog log;
  base::TelemetryLogRegistrar registrar{&log};

  boost::signals2::signal<void(const ImuData*)> imu_signal;
  ImuData imu_data;

  registrar.Register("imu", &imu_signal);

  log.SetRealtime(true);
  if (!log_file.empty()) { log.Open(log_file); }

  struct Bmi088Data {
    uint16_t present = 0;
    float gx_dps = 0;
    float gy_dps = 0;
    float gz_dps = 0;
    float ax_m_s2 = 0;
    float ay_m_s2 = 0;
    float az_m_s2 = 0;
  } __attribute__((packed));
  Bmi088Data bmi088_data;

  while (true) {
    bmi088_data = {};
    spi.Read(33, mjlib::base::string_span(
                 reinterpret_cast<char*>(&bmi088_data), sizeof(bmi088_data)));
    if (bmi088_data.present & 0x01) {
      imu_data.timestamp = boost::posix_time::microsec_clock::universal_time();
      imu_data.rate_deg_s = { bmi088_data.gx_dps,
                              bmi088_data.gy_dps,
                              bmi088_data.gz_dps };
      imu_data.accel_mps2 = { bmi088_data.ax_m_s2,
                              bmi088_data.ay_m_s2,
                              bmi088_data.az_m_s2 };
      imu_signal(&imu_data);
    }
    ::usleep(50);
  }
}

}
}


int main(int argc, char** argv) {
  return mjmech::mech::do_main(argc, argv);
}
