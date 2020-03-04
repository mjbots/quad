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

#include "mech/rpi3_raw_aux_spi.h"

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include "mjlib/base/system_error.h"

namespace mjmech {
namespace mech {

namespace {
int64_t GetNow() {
  struct timespec ts = {};
  ::clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<int64_t>(ts.tv_sec) * 1000000000ll +
      static_cast<int64_t>(ts.tv_nsec);
}

void BusyWaitUs(int64_t us) {
  const auto start = GetNow();
  const auto end = start + us * 1000;
  while (GetNow() <= end);
}

constexpr uint32_t RASPI_23_PERI_BASE = 0x3F000000;
constexpr uint32_t AUX_BASE           = 0x00215000;
constexpr uint32_t kSpi1CS0 = 18;
constexpr uint32_t kSpi1CS1 = 17;
constexpr uint32_t kSpi1CS2 = 16;
constexpr uint32_t kSpi1CS[] = {
  kSpi1CS0,
  kSpi1CS1,
  kSpi1CS2,
};

constexpr int AUXSPI_STAT_TX_FULL = 1 << 10;
constexpr int AUXSPI_STAT_TX_EMPTY = 1 << 9;
constexpr int AUXSPI_STAT_RX_EMPTY = 1 << 7;
constexpr int AUXSPI_STAT_BUSY = 1 << 6;
}

Rpi3RawAuxSpi::Rpi3RawAuxSpi(const Options& options) {
  fd_ = ::open("/dev/mem", O_RDWR | O_SYNC);
  mjlib::base::system_error::throw_if(
      fd_ < 0, "rpi3_aux_spi: could not open /dev/mem");

  spi_mmap_ = base::SystemMmap(fd_, 4096, RASPI_23_PERI_BASE + AUX_BASE);
  auxenb_ = reinterpret_cast<volatile uint32_t*>(
      static_cast<char*>(spi_mmap_.ptr()) + 0x04);
  spi_ = reinterpret_cast<volatile Bcm2835AuxSpi*>(
      static_cast<char*>(spi_mmap_.ptr()) + 0x80);

  gpio_ = std::make_unique<Rpi3Gpio>(fd_);

  gpio_->SetGpioOutput(kSpi1CS0, true);
  gpio_->SetGpioOutput(kSpi1CS1, true);
  gpio_->SetGpioOutput(kSpi1CS2, true);

  gpio_->SetGpioMode(kSpi1CS0, Rpi3Gpio::OUTPUT); // We'll do CS in SW
  gpio_->SetGpioMode(kSpi1CS1, Rpi3Gpio::OUTPUT);
  gpio_->SetGpioMode(kSpi1CS2, Rpi3Gpio::OUTPUT);
  gpio_->SetGpioMode(19, Rpi3Gpio::ALT_4);
  gpio_->SetGpioMode(20, Rpi3Gpio::ALT_4);
  gpio_->SetGpioMode(21, Rpi3Gpio::ALT_4);

  // Start by disabling it to try and get to a known good state.
  *auxenb_ &= ~0x02;

  BusyWaitUs(10);

  // Enable the SPI peripheral.
  *auxenb_ |= 0x02;  // SPI1 enable

  spi_->cntl1 = 0;
  spi_->cntl0 = (1 << 9); // clear fifos

  // Configure the SPI peripheral.
  const int clkdiv =
      std::max(0, std::min(4095, 250000000 / 2 / options.speed_hz - 1));
  spi_->cntl0 = (0
      | (clkdiv << 20)
      | (7 << 17) // chip select defaults
      | (0 << 16) // post-input mode
      | (0 << 15) // variable CS
      | (1 << 14) // variable width
      | (0 << 12) // DOUT hold time
      | (1 << 11) // enable
      | (1 << 10) // in rising?
      | (0 << 9) // clear fifos
      | (0 << 8) // out rising
      | (0 << 7) // invert SPI CLK
      | (1 << 6) // MSB first
      | (0 << 0) // shift length
  );

  spi_->cntl1 = (0
      | (7 << 8) // CS high time
      | (0 << 7) // tx empty IRQ
      | (0 << 6) // done IRQ
      | (1 << 1) // shift in MS first
      | (0 << 0) // keep input
  );
}

Rpi3RawAuxSpi::~Rpi3RawAuxSpi() {}

void Rpi3RawAuxSpi::Write(int cs, int address, std::string_view data) {
  BusyWaitUs(options_.cs_hold_us);
  Rpi3Gpio::ActiveLow cs_holder(gpio_.get(), kSpi1CS[cs]);
  BusyWaitUs(options_.cs_hold_us);

  const uint32_t value = 0
      | (0 << 29) // CS
      | (16 << 24) // data width
      | ((address & 0xffff) << 8) // data
      ;
  if (data.size()) {
    spi_->txhold = value;
  } else {
    spi_->io = value;
  }

  while ((spi_->stat & AUXSPI_STAT_TX_EMPTY) == 0);

  if (data.empty()) { return; }

  // Wait our address hold time.
  BusyWaitUs(options_.address_hold_us);

  size_t offset = 0;
  while (offset < data.size()) {
    while (spi_->stat & AUXSPI_STAT_TX_FULL);
    const uint32_t data_value = 0
        | (0 << 29) // CS
        | (8 << 24) // data width
        | (data[offset] << 16) // data
        ;
    if (offset + 1 == data.size()) {
      spi_->io = data_value;
    } else {
      spi_->txhold = data_value;
    }
    offset++;
  }

  // Discard anything in the RX fifo.
  while ((spi_->stat & AUXSPI_STAT_RX_EMPTY) == 0) {
    (void) spi_->io;
  }

  // Wait until we are no longer busy.
  while (spi_->stat & AUXSPI_STAT_BUSY);
}

void Rpi3RawAuxSpi::Read(int cs, int address, mjlib::base::string_span data) {
  BusyWaitUs(options_.cs_hold_us);
  Rpi3Gpio::ActiveLow cs_holder(gpio_.get(), kSpi1CS[cs]);
  BusyWaitUs(options_.cs_hold_us);

  const uint32_t value = 0
      | (0 << 29) // CS
      | (16 << 24) // data width
      | ((address & 0xffff) << 8) // data
      ;
  if (data.size()) {
    spi_->txhold = value;
  } else {
    spi_->io = value;
  }

  while ((spi_->stat & AUXSPI_STAT_TX_EMPTY) == 0);

  if (data.empty()) { return; }

  // Wait our address hold time.
  BusyWaitUs(options_.address_hold_us);

  // Discard the rx fifo.
  while ((spi_->stat & AUXSPI_STAT_RX_EMPTY) == 0) {
    (void) spi_->io;
  }

  // Now we write out dummy values, reading values in.
  std::size_t remaining_read = data.size();
  std::size_t remaining_write = remaining_read;
  char* ptr = data.data();
  while (remaining_read) {
    // Make sure we don't write more than we have read spots remaining
    // so that we can never overflow the RX fifo.
    const bool can_write = (remaining_read - remaining_write) < 3;
    if (can_write &&
        remaining_write && (spi_->stat & AUXSPI_STAT_TX_FULL) == 0) {
      const uint32_t data = 0
          | (0 << 29) // CS
          | (8 << 24) // data width
          | (0) // data
          ;
      remaining_write--;
      if (remaining_write == 0) {
        spi_->io = data;
      } else {
        spi_->txhold = data;
      }
    }

    if (remaining_read && (spi_->stat & AUXSPI_STAT_RX_EMPTY) == 0) {
      *ptr = spi_->io & 0xff;
      ptr++;
      remaining_read--;
    }
  }
}

}
}
