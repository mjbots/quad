// Copyright 2019 Josh Pieper, jjp@pobox.com.
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

#include "mech/rpi3_raw_uart.h"

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include <cmath>

#include <fmt/format.h>

#include "mjlib/base/system_error.h"

namespace mjmech {
namespace mech {

namespace {
constexpr uint32_t RASPI_23_PERI_BASE = 0x3F000000;
constexpr uint32_t GPIO_BASE = 0x00200000;
constexpr uint32_t UART0_BASE = 0x00201000;

constexpr int kMaxBaudRate = 3000000;

class Gpio {
 public:
  // static constexpr uint32_t INPUT = 0;
  // static constexpr uint32_t OUTPUT = 1;
  static constexpr uint32_t ALT_0 = 4;
  // static constexpr uint32_t ALT_1 = 5;
  // static constexpr uint32_t ALT_2 = 6;
  // static constexpr uint32_t ALT_3 = 7;
  // static constexpr uint32_t ALT_4 = 3;
  // static constexpr uint32_t ALT_5 = 2;

  Gpio(int dev_mem_fd)
      : mmap_(dev_mem_fd, 4096, RASPI_23_PERI_BASE + GPIO_BASE),
        gpio_(reinterpret_cast<volatile uint32_t*>(mmap_.ptr())) {}

  void SetGpioMode(uint32_t gpio, uint32_t function) {
    uint32_t reg_offset = gpio / 10;
    uint32_t bit = (gpio % 10) * 3;
    const auto value = gpio_[reg_offset];
    gpio_[reg_offset] = (value & ~(0x7 << bit)) | ((function & 0x7) << bit);
  }

  volatile uint32_t& operator[](int index) { return gpio_[index]; }
  const volatile uint32_t& operator[](int index) const { return gpio_[index]; }

 private:
  base::SystemMmap mmap_;
  volatile uint32_t* const gpio_;
};
}

Rpi3RawUart::Rpi3RawUart(const Options& options) {
  fd_ = ::open("/dev/mem", O_RDWR | O_SYNC);
  mjlib::base::system_error::throw_if(
      fd_ < 0, "rpi3_uart: could not open /dev/mem");

  uart_mmap_ = base::SystemMmap(fd_, 4096, RASPI_23_PERI_BASE + UART0_BASE);
  uart_ = reinterpret_cast<volatile Pl011Uart*>(uart_mmap_.ptr());
  Gpio gpio(fd_);

  uart_->ctrl = 0x0000;  // disable everything

  // Configure our pin mode functions so that the UART drives them.
  gpio.SetGpioMode(14, Gpio::ALT_0);
  gpio.SetGpioMode(15, Gpio::ALT_0);

  // Now configure the baud rate.
  const int integral_divisor = kMaxBaudRate / options.baud_rate;
  const int fractional_divisor = static_cast<int>(std::fmod(kMaxBaudRate, options.baud_rate) * 64 + 0.5);

  const double generated_divisor = integral_divisor + fractional_divisor / 64.0;
  const double actual_baud = kMaxBaudRate / generated_divisor;
  const double error = (actual_baud - options.baud_rate) / options.baud_rate;
  if (std::abs(error) > options.max_baud_rate_error) {
    throw mjlib::base::system_error::einval(
        fmt::format("Could not generate baud {}, error {:.4f} > {:.4f}",
                    options.baud_rate, std::abs(error),
                    options.max_baud_rate_error));
  }

  uart_->int_baud = integral_divisor;
  uart_->frac_baud = fractional_divisor;

  // Setting the line control must take place immediately after the
  // baud rate registers to actually get them all to the device.
  uart_->line_ctrl = LCR_FIFO_EN | LCR_8_BITS;

  // Read to ignore any errors.
  (void) uart_->data;
  uart_->rx_err = 0;

  // Now, enable the UART.
  uart_->ctrl = CR_UART_ENABLE | CR_TX_ENABLE | CR_RX_ENABLE;
}

Rpi3RawUart::~Rpi3RawUart() {
}

}
}
