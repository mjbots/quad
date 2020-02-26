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

namespace mjmech {
namespace mech {

/// This class interacts with the AUX SPI1 device on a raspberry pi
/// using the BCM2835/6/7's registers directly.  The kernel driver
/// must not be active, and this must be run as root or otherwise have
/// access to /dev/mem.
class Rpi3RawAuxSpi {
 public:
  struct Options {
    int speed_hz = 10000000;
    int cs_hold_us = 4;
    int address_hold_us = 8;

    Options() {}
  };

  Rpi3RawAuxSpi(const Options& = Options());
  ~Rpi3RawAuxSpi();

  Rpi3RawAuxSpi(const Rpi3RawAuxSpi&) = delete;
  Rpi3RawAuxSpi& operator=(const Rpi3RawAuxSpi&) = delete;

  void Write(int cs, int address, std::string_view data);

  void Read(int cs, int address, mjlib::base::string_span data);

 private:
  // This is the memory layout of the SPI peripheral.
  struct Bcm2835AuxSpi {
    uint32_t cntl0;
    uint32_t cntl1;
    uint32_t stat;
    uint32_t peek;
    uint32_t ign1[4];
    uint32_t io;
    uint32_t ign2;
    uint32_t ign3;
    uint32_t ign4;
    uint32_t txhold;
  };

  const Options options_;
  base::SystemFd fd_;
  base::SystemMmap spi_mmap_;
  volatile uint32_t* auxenb_ = nullptr;
  volatile Bcm2835AuxSpi* spi_ = nullptr;

  class Gpio;
  std::unique_ptr<Gpio> gpio_;
};

}
}
