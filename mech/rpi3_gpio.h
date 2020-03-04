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

#include "base/system_mmap.h"

namespace mjmech {
namespace mech {

class Rpi3Gpio {
 public:
  static constexpr uint32_t RASPI_23_PERI_BASE = 0x3F000000;
  static constexpr uint32_t GPIO_BASE          = 0x00200000;

  // static constexpr uint32_t INPUT = 0;
  static constexpr uint32_t OUTPUT = 1;
  static constexpr uint32_t ALT_0 = 4;
  // static constexpr uint32_t ALT_1 = 5;
  // static constexpr uint32_t ALT_2 = 6;
  // static constexpr uint32_t ALT_3 = 7;
  static constexpr uint32_t ALT_4 = 3;
  // static constexpr uint32_t ALT_5 = 2;

  Rpi3Gpio(int dev_mem_fd)
      : mmap_(dev_mem_fd, 4096, RASPI_23_PERI_BASE + GPIO_BASE),
        gpio_(reinterpret_cast<volatile uint32_t*>(mmap_.ptr())) {}

  void SetGpioMode(uint32_t gpio, uint32_t function) {
    uint32_t reg_offset = gpio / 10;
    uint32_t bit = (gpio % 10) * 3;
    const auto value = gpio_[reg_offset];
    gpio_[reg_offset] = (value & ~(0x7 << bit)) | ((function & 0x7) << bit);
  }

  void SetGpioOutput(uint32_t gpio, bool value) {
    if (value) {
      const uint32_t reg_offset = gpio / 32 + 7;
      gpio_[reg_offset] = 1 << (gpio % 32);
    } else {
      const uint32_t reg_offset = gpio / 32 + 10;
      gpio_[reg_offset] = 1 << (gpio % 32);
    }
  }

  volatile uint32_t& operator[](int index) { return gpio_[index]; }
  const volatile uint32_t& operator[](int index) const { return gpio_[index]; }

  class ActiveLow {
   public:
    ActiveLow(Rpi3Gpio* parent, uint32_t gpio) : parent_(parent), gpio_(gpio) {
      parent_->SetGpioOutput(gpio_, false);
    }

    ~ActiveLow() {
      parent_->SetGpioOutput(gpio_, true);
    }

   private:
    Rpi3Gpio* const parent_;
    const uint32_t gpio_;
  };

 private:
  base::SystemMmap mmap_;
  volatile uint32_t* const gpio_;
};

}
}
