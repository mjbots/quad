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

#include "mjlib/base/string_span.h"

#include "base/system_fd.h"
#include "base/system_mmap.h"

#include "mech/rpi3_gpio.h"

namespace mjmech {
namespace mech {

/// This class interacts with the SPI0 device on a raspberry pi
/// using the BCM2835/6/7's registers directly.  The kernel driver
/// must not be active, and this must be run as root or otherwise have
/// access to /dev/mem.
class Rpi3RawSpi {
 public:
  struct Options {
    int speed_hz = 10000000;
    int cs_hold_us = 4;
    int address_hold_us = 8;

    Options() {}
  };

  Rpi3RawSpi(const Options& = Options());
  ~Rpi3RawSpi();

  Rpi3RawSpi(const Rpi3RawSpi&) = delete;
  Rpi3RawSpi& operator=(const Rpi3RawSpi&) = delete;

  void Write(int cs, int address, std::string_view data);

  void Read(int cs, int address, mjlib::base::string_span data);

 private:
  // This is the memory layout of the SPI peripheral.
  struct Bcm2835Spi {
    uint32_t cs;
    uint32_t fifo;
    uint32_t clk;
    uint32_t dlen;
    uint32_t ltoh;
    uint32_t dc;
  };

  const Options options_;
  base::SystemFd fd_;
  base::SystemMmap spi_mmap_;
  volatile Bcm2835Spi* spi_ = nullptr;

  std::unique_ptr<Rpi3Gpio> gpio_;
};

}
}
