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

#include "mech/rpi3_raw_spi.h"

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
constexpr uint32_t kSpi0CS0 = 8;
constexpr uint32_t kSpi0CS1 = 7;
constexpr uint32_t kSpi0CS[] = {kSpi0CS0, kSpi0CS1};

constexpr uint32_t SPI_BASE = 0x204000;
constexpr uint32_t SPI_CS_TA = 1 << 7;
constexpr uint32_t SPI_CS_DONE = 1 << 16;
constexpr uint32_t SPI_CS_RXD = 1 << 17;
constexpr uint32_t SPI_CS_TXD = 1 << 18;
}

Rpi3RawSpi::Rpi3RawSpi(const Options& options) {
  fd_ = ::open("/dev/mem", O_RDWR | O_SYNC);
  mjlib::base::system_error::throw_if(
      fd_ < 0, "rpi3_raw_spi: could not open /dev/mem");

  spi_mmap_ = base::SystemMmap(fd_, 4096, RASPI_23_PERI_BASE + SPI_BASE);
  spi_ = reinterpret_cast<volatile Bcm2835Spi*>(
      static_cast<char*>(spi_mmap_.ptr()));

  gpio_ = std::make_unique<Rpi3Gpio>(fd_);

  gpio_->SetGpioOutput(kSpi0CS0, true);
  gpio_->SetGpioOutput(kSpi0CS1, true);

  gpio_->SetGpioMode(kSpi0CS0, Rpi3Gpio::OUTPUT); // We'll do CS in SW
  gpio_->SetGpioMode(kSpi0CS1, Rpi3Gpio::OUTPUT);
  gpio_->SetGpioMode(9, Rpi3Gpio::ALT_0);
  gpio_->SetGpioMode(10, Rpi3Gpio::ALT_0);
  gpio_->SetGpioMode(11, Rpi3Gpio::ALT_0);

  spi_->cs = (
      0
      | (0 << 25) // LEn_LONG
      | (0 << 24) // DMA_LEN
      | (0 << 23) // CSPOL2
      | (0 << 22) // CSPOL1
      | (0 << 21) // CSPOL0
      | (0 << 13) // LEN
      | (0 << 12) // REN
      | (0 << 11) // ADCS
      | (0 << 10) // INTR
      | (0 << 9) // INTD
      | (0 << 8) // DMAEN
      | (0 << 7) // TA
      | (0 << 6) // CSPOL
      | (3 << 4) // CLEAR
      | (0 << 3) // CPOL
      | (0 << 2) // CPHA
      | (0 << 0) // CS
  );

  // Configure the SPI peripheral.
  const int clkdiv =
      std::max(0, std::min(65535, 400000000 / options.speed_hz));
  spi_->clk = clkdiv;
}

Rpi3RawSpi::~Rpi3RawSpi() {}

namespace {
void write_u8(volatile uint32_t& address, uint8_t value) {
  volatile uint8_t* u8_addr = reinterpret_cast<volatile uint8_t*>(&address);
  *u8_addr = value;
}

uint8_t read_u8(volatile uint32_t& address) {
  volatile uint8_t* u8_addr = reinterpret_cast<volatile uint8_t*>(&address);
  return *u8_addr;
}
}

void Rpi3RawSpi::Write(int cs, int address, std::string_view data) {
  BusyWaitUs(options_.cs_hold_us);
  Rpi3Gpio::ActiveLow cs_holder(gpio_.get(), kSpi0CS[cs]);
  BusyWaitUs(options_.cs_hold_us);

  spi_->cs |= SPI_CS_TA;

  write_u8(spi_->fifo, (address & 0xff00) >> 8);
  write_u8(spi_->fifo, (address & 0x00ff));

  while ((spi_->cs & SPI_CS_DONE) == 0) {
    if (spi_->cs & SPI_CS_RXD) {
      (void) spi_->fifo;
    }
  }

  spi_->cs |= SPI_CS_DONE;

  if (!data.empty()) {
    // Wait our address hold time.
    BusyWaitUs(options_.address_hold_us);

    size_t offset = 0;
    while (offset < data.size()) {
      while ((spi_->cs & SPI_CS_TXD) == 0);
      write_u8(spi_->fifo, data[offset]);
      offset++;
    }

    // Wait until we are no longer busy.
    while ((spi_->cs & SPI_CS_DONE) == 0) {
      if (spi_->cs & SPI_CS_RXD) {
        (void) spi_->fifo;
      }
    }
  }

  spi_->cs &= ~SPI_CS_TA;
}

void Rpi3RawSpi::Read(int cs, int address, mjlib::base::string_span data) {
  BusyWaitUs(options_.cs_hold_us);
  Rpi3Gpio::ActiveLow cs_holder(gpio_.get(), kSpi0CS[cs]);
  BusyWaitUs(options_.cs_hold_us);

  spi_->cs |= SPI_CS_TA;

  write_u8(spi_->fifo, (address & 0xff00) >> 8);
  write_u8(spi_->fifo, (address & 0x00ff));

  while ((spi_->cs & SPI_CS_DONE) == 0) {
    if (spi_->cs & SPI_CS_RXD) {
      (void) spi_->fifo;
    }
  }

  if (!data.empty()) {
    // Wait our address hold time.
    BusyWaitUs(options_.address_hold_us);

    // Discard the rx fifo.
    while (spi_->cs & SPI_CS_RXD) {
      (void) spi_->fifo;
    }

    // Now we write out dummy values, reading values in.
    std::size_t remaining_read = data.size();
    std::size_t remaining_write = remaining_read;
    char* ptr = data.data();
    while (remaining_read) {
      // Make sure we don't write more than we have read spots remaining
      // so that we can never overflow the RX fifo.
      const bool can_write = (remaining_read - remaining_write) < 16;
      if (can_write &&
          remaining_write && (spi_->cs & SPI_CS_TXD) != 0) {
        write_u8(spi_->fifo, 0);
        remaining_write--;
      }

      if (remaining_read && (spi_->cs & SPI_CS_RXD) != 0) {
        *ptr = read_u8(spi_->fifo);
        ptr++;
        remaining_read--;
      }
    }
  }

  spi_->cs &= ~SPI_CS_TA;
}

}
}

extern "C" {
  uint32_t ReadU32(uint32_t address) {
    uint32_t* ptr = reinterpret_cast<uint32_t*>(address);
    return *ptr;
  }
}
