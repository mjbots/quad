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

#if 0

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/mman.h>

#include <iostream>

#include <fmt/format.h>

#include "mjlib/base/system_error.h"

#define RASPI_23_PERI_BASE   0x3F000000
#define GPIO_BASE            0x00200000
#define UART0_BASE           0x00201000

#define GPIO_INPUT          0
#define GPIO_OUTPUT         1
#define GPIO_ALT_0          4
#define GPIO_ALT_1          5
#define GPIO_ALT_2          6
#define GPIO_ALT_3          7
#define GPIO_ALT_4          3
#define GPIO_ALT_5          2



// PL011 UART register (16C650 type)
// =================================
#define UART_DATA         (UartAddr+0x00)
#define UART_RX_ERR       (UartAddr+0x04)
#define UART_FLAG         (UartAddr+0x18)
#define UART_ILPR         (UartAddr+0x20)
#define UART_INT_BAUD     (UartAddr+0x24)
#define UART_FRAC_BAUD    (UartAddr+0x28)
#define UART_LINE_CTRL    (UartAddr+0x2C)
#define UART_CTRL         (UartAddr+0x30)
#define UART_FIFO_LEVEL   (UartAddr+0x34)
#define UART_INT_MASK     (UartAddr+0x38)
#define UART_RAW_INT      (UartAddr+0x3C)
#define UART_INT_STAT     (UartAddr+0x40)
#define UART_INT_CLR      (UartAddr+0x44)
#define UART_DMA_CTRL     (UartAddr+0x48)
#define UART_TEST_CTRL    (UartAddr+0x80)
#define UART_TEST_IN      (UartAddr+0x84)
#define UART_IEST_OUT     (UartAddr+0x88)
#define UART_TEST_DATA    (UartAddr+0x8C)
#define UART_MEM_SIZE     0xC0

// UART_FLAG register
// ==================
#define UART_RX_FIFO_EMPTY (1 << 4)
#define UART_TX_FIFO_FULL  (1 << 5)

// UART Line Control Register
// ==========================
#define UART_LCR_BREAK          (1 << 0)
#define UART_LCR_PARITY_EN      (1 << 1)
#define UART_LCR_EVEN_PARITY    (1 << 2)
#define UART_LCR_2_STOP         (1 << 3)
#define UART_LCR_FIFO_EN        (1 << 4)
#define UART_LCR_8_BITS         (3 << 5)
#define UART_LCR_STICK_PARITY   (1 << 7)

// UART Control Register
// ======================
#define UARTCR_UART_ENABLE      (1 << 0)
#define UARTCR_LOOPBACK         (1 << 7)
#define UARTCR_TX_ENABLE        (1 << 8)
#define UARTCR_RX_ENABLE        (1 << 9)
#define UARTCR_RTS              (1 << 11)

// UART Interrupt masks
// ====================
#define INT_CTS                 (1 << 1)
#define INT_RX                  (1 << 4)
#define INT_TX                  (1 << 5)
#define INT_RX_TIMEOUT          (1 << 6)
#define INT_FRAMING_ERR         (1 << 7)
#define INT_PARITY_ERR          (1 << 8)
#define INT_BREAK_ERR           (1 << 9)
#define INT_OVER_ERR            (1 << 10)

class Gpio {
 public:
  Gpio(int dev_mem_fd) {
    int gpio_addr = RASPI_23_PERI_BASE + GPIO_BASE;
    void* gpio_ptr =
        ::mmap(0, 4096, PROT_READ|PROT_WRITE, MAP_SHARED, dev_mem_fd, gpio_addr);
    mjlib::base::system_error::throw_if(gpio_ptr == MAP_FAILED);

    gpio_ = reinterpret_cast<volatile uint32_t*>(gpio_ptr);
  }

  void set_gpio_mode(unsigned int gpio, unsigned int function) {
    unsigned int reg_offset = (gpio / 10);
    unsigned int bit = (gpio % 10) * 3;
    auto value = gpio_[reg_offset];
    gpio_[reg_offset] = (value & ~(0x7 << bit)) | ((function & 0x7) << bit);
    ::usleep(1000);
  }

  volatile uint32_t* gpio_;
};

struct Pl011Uart {
  uint32_t data;
  uint32_t rx_err;

  uint32_t ign1[4];

  uint32_t flag;
  uint32_t ign2;
  uint32_t ilpr;
  uint32_t int_baud;
  uint32_t frac_baud;
  uint32_t line_ctrl;
  uint32_t ctrl;
  uint32_t fifo_level;
};


int main(int argc, char** argv) {
  int fd = ::open("/dev/mem", O_RDWR|O_SYNC);
  mjlib::base::system_error::throw_if(fd < 0);
  int UartAddr = RASPI_23_PERI_BASE + UART0_BASE;
  void* uart_ptr =
      ::mmap(0, 4096, PROT_READ|PROT_WRITE, MAP_SHARED, fd, UartAddr);
  auto* bytes = reinterpret_cast<uint32_t*>(uart_ptr);
  mjlib::base::system_error::throw_if(bytes == MAP_FAILED);

  Gpio gpio{fd};

  // For now, just dump it all out as hex to see if it makes sense.
  for (int i = 0; i < UART_MEM_SIZE / 4; i++) {
    if ((i % 4) == 0) {
      std::cout << "\n";
      std::cout << fmt::format("{:03x}:", i * 4);
    }
    std::cout << fmt::format(" {:08x}", bytes[i]);
  }
  std::cout << "\n";

  auto* uart = reinterpret_cast<volatile Pl011Uart*>(uart_ptr);

  // Try writing something out in a loop.
  uart->ctrl = 0x0000;  // disable everything

  gpio.set_gpio_mode(15, GPIO_ALT_0);      // GPIO15 connected to RxD
  gpio.set_gpio_mode(14, GPIO_ALT_0);      // GPIO14 connected to TxD

  // Set pull-down for the GPIO pin
  // ==============================
  // gpio_pullupdown(14, GPIO_PULL_UP);
  // gpio_pullupdown(15, GPIO_PULL_UP);


  // I think the input frequency is 3Mbit
  uart->int_baud = 1;
  uart->frac_baud = 0;
  uart->line_ctrl = 3 << 5;  // this must be set to latch the baud rate.  set 8 bit frame

  // Read to clear errors.
  auto ignored = uart->data;
  (void) ignored;
  uart->rx_err = 0;


  uart->ctrl = 0x0301;  // enable UART, rx and tx


  while (true) {
    uart->data = 0xaa;
    std::cout << fmt::format("flag={:04x}\n", uart->flag);
    ::usleep(100000);
  }

  return 0;
}
#endif

#include "mech/rpi3_raw_uart.h"

int main(int argc, char** argv) {
  mjmech::mech::Rpi3RawUart uart([]() {
      mjmech::mech::Rpi3RawUart::Options options;
      options.baud_rate = 1500000;
      return options;
    }());

  while (true) {
    uart.write("hello");
    ::usleep(1000);
  }

  return 0;
}
