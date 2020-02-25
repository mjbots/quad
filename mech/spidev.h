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

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>

namespace mjmech {
namespace mech {

class SpiDev {
 public:
  SpiDev(const std::string& filename, int speed_hz) {
    fd_ = ::open(filename.c_str(), O_RDWR);
    mjlib::base::system_error::throw_if(fd_ < 0, "opening: " + filename);

    xfer_[0].cs_change = 0;
    xfer_[0].delay_usecs = 0;
    xfer_[0].speed_hz = speed_hz;
    xfer_[0].bits_per_word = 8;
    xfer_[1].cs_change = 0;
    xfer_[1].delay_usecs = 0;
    xfer_[1].speed_hz = speed_hz;
    xfer_[1].bits_per_word = 8;
  }

  ~SpiDev() {
    if (fd_ >= 0) {
      ::close(fd_);
    }
  }

  void Write(int address, std::string_view data) {
    buf_[0] = (address & 0xff00) >> 8;
    buf_[1] = address & 0xff;
    BOOST_ASSERT(data.size() + 2 < sizeof(buf_));
    std::memcpy(&buf_[2], data.data(), data.size());
    xfer_[0].len = 2 + data.size();
    xfer_[0].tx_buf = reinterpret_cast<uint64_t>(buf_);

    const int status = ::ioctl(fd_, SPI_IOC_MESSAGE(1), xfer_);
    mjlib::base::system_error::throw_if(status < 0, "writing to SPI");
  }

  void Read(int address, mjlib::base::string_span data) {
    buf_[0] = (address & 0xff00) >> 8;
    buf_[1] = address & 0xff;
    xfer_[0].tx_buf = reinterpret_cast<uint64_t>(buf_);
    xfer_[0].len = 2;
    xfer_[1].rx_buf = reinterpret_cast<uint64_t>(data.data());
    xfer_[1].len = data.size();

    const int status = ::ioctl(fd_, SPI_IOC_MESSAGE(2), xfer_);
    mjlib::base::system_error::throw_if(status < 0, "reading from SPI");
  }

 private:
  int fd_ = -1;
  struct spi_ioc_transfer xfer_[2] = {};
  uint8_t buf_[1024] = {};
};

}
}
