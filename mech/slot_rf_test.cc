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

#include <clipp/clipp.h>

#include "mjlib/base/clipp.h"

#include "mech/attitude_data.h"
#include "mech/imu_data.h"
#include "mech/rpi3_raw_spi.h"

namespace mjmech {
namespace mech {
namespace {

int do_main(int argc, char** argv) {
  int speed = 10000000;
  std::string log_file = "";

  auto group = clipp::group(
      (clipp::option("s", "speed") & clipp::value("", speed)) % "SPI speed",
      (clipp::option("l", "log") & clipp::value("", log_file)) % "log file name"
                            );

  mjlib::base::ClippParse(argc, argv, group);

  Rpi3RawSpi spi([&]() {
      Rpi3RawSpi::Options options;
      options.speed_hz = speed;
      return options;
    }());

  uint32_t old_slot_bitfield = 0;
  while (true) {
    uint32_t slot_bitfield = 0;
    spi.Read(0, 56, mjlib::base::string_span(
                 reinterpret_cast<char*>(&slot_bitfield), sizeof(slot_bitfield)));
    if (slot_bitfield == old_slot_bitfield) {
      ::usleep(10000);
      continue;
    }

    const uint32_t difference = slot_bitfield ^ old_slot_bitfield;
    old_slot_bitfield = slot_bitfield;

    std::ostringstream str;
    for (int slot_num = 0; slot_num < 15; slot_num++) {
      if (difference & (0x3 << (slot_num * 2))) {
        char buf[21] = {};
        spi.Read(0, 64 + slot_num, buf);

        const uint8_t size = static_cast<uint8_t>(buf[4]);
        if (size < 16) {
          str << fmt::format("{}:", slot_num);
          for (int i = 0; i < size; i++) {
            str << fmt::format("{:02x}", static_cast<int>(buf[5 + i]));
          }
          str << " ";
        }
      }
    }
    std::cout << str.str() << "\n";
  }

  return 0;
}

}
}
}

int main(int argc, char** argv) {
  return mjmech::mech::do_main(argc, argv);
}
