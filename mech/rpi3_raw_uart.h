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

#pragma once

#include <time.h>

#include <cstdint>

#include "base/system_fd.h"
#include "base/system_mmap.h"

namespace mjmech {
namespace mech {

/// This class accesses the registers of the PL011 on a raspberry pi3
/// directly.  You must have previously disabled the kernel driver
/// using raspi-config and this needs to be run as either root, or
/// otherwise have access to /dev/mem.
class Rpi3RawUart {
 public:
  struct Options {
    int baud_rate = 3000000;

    double max_baud_rate_error = 0.02;

    Options() {}
  };

  Rpi3RawUart(const Options& = Options());
  ~Rpi3RawUart();

  Rpi3RawUart(const Rpi3RawUart&) = delete;
  Rpi3RawUart& operator=(const Rpi3RawUart&) = delete;

  /// Read a single character.
  ///
  /// Return:
  ///  -1 if the time as returned by clock_gettime(CLOCK_MONOTONIC) is
  ///     greater than @p timeout
  ///  -2 for some other serial error
  int getc(int64_t timeout) {
    struct timespec ts = {};
    while (true) {
      for (int i = 0; i < kInnerLoopRetryCount; i++) {
        if (uart_->flag & FLAG_RX_FIFO_EMPTY) { continue; }
        const auto result = uart_->data;
        if (result & 0xff00) {
          // A serial error.
          return -2;
        }
        return result & 0xff;
      }
      // Check our overall timeout.
      mjlib::base::system_error::throw_if(
          ::clock_gettime(CLOCK_MONOTONIC, &ts) < 0);;
      constexpr auto i64 = [](auto v) { return static_cast<int64_t>(v); };
      const int64_t now = i64(ts.tv_sec) * 1000000000ll + i64(ts.tv_nsec);
      if (now >= timeout) {
        return -1;
      }
    }
  }

  void putc(char c) {
    while (uart_->flag & FLAG_TX_FIFO_FULL);
    uart_->data = c & 0xff;
  }

  void write(std::string_view data) {
    for (auto c : data) { putc(c); }
  }

 private:
  // This is the memory layout of the PL011 memory mapped IO block.
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

  static constexpr uint32_t CR_UART_ENABLE = 1 << 0;
  static constexpr uint32_t CR_LOOPBACK = 1 << 7;
  static constexpr uint32_t CR_TX_ENABLE = 1 << 8;
  static constexpr uint32_t CR_RX_ENABLE = 1 << 9;

  static constexpr uint32_t LCR_BREAK = 1 << 0;
  static constexpr uint32_t LCR_PARITY_EN = 1 << 1;
  static constexpr uint32_t LCR_EVEN_PARITY = 1 << 2;
  static constexpr uint32_t LCR_2_STOP = 1 << 3;
  static constexpr uint32_t LCR_FIFO_EN = 1 << 4;
  static constexpr uint32_t LCR_8_BITS = 3 << 5;

  static constexpr uint32_t FLAG_RX_FIFO_EMPTY = 1 << 4;
  static constexpr uint32_t FLAG_TX_FIFO_FULL = 1 << 5;

  static constexpr int kInnerLoopRetryCount = 1000;

  base::SystemFd fd_;
  base::SystemMmap uart_mmap_;
  volatile Pl011Uart* uart_ = nullptr;
};

}
}
