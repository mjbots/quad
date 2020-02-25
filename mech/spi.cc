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

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>

#include <cstdint>
#include <iostream>

#include <fmt/format.h>

#include <boost/algorithm/string.hpp>
#include <boost/program_options.hpp>

#include "mjlib/base/string_span.h"
#include "mjlib/base/system_error.h"

namespace po = boost::program_options;

namespace {
class SPI {
 public:
  SPI(const std::string& filename, int speed_hz) {
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

  ~SPI() {
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

  int fd_ = -1;
  struct spi_ioc_transfer xfer_[2] = {};
  uint8_t buf_[1024] = {};
};

int ParseNybble(char c) {
  if (c >= '0' && c <= '9') { return c - '0'; }
  if (c >= 'a' && c <= 'f') { return (c - 'a') + 10; }
  if (c >= 'A' && c <= 'F') { return (c - 'A') + 10; }
  return 0;
}

char ParseHexByte(const std::string& hex_byte) {
  BOOST_ASSERT(hex_byte.size() == 2);
  return (ParseNybble(hex_byte[0]) << 4) | ParseNybble(hex_byte[1]);
}

std::string ReadHex(const std::string& line) {
  std::string result;
  for (size_t i = 0; i < line.size(); i += 2) {
    result.push_back(ParseHexByte(line.substr(i, 2)));
  }
  return result;
}

std::string FormatHex(const std::string_view data) {
  std::string result;
  for (auto c : data) {
    result += fmt::format("{:02x}", static_cast<int>(c));
  }
  return result;
}

void RunConsole(SPI* spi) {
  while (std::cin) {
    std::string line;
    std::getline(std::cin, line);
    boost::trim(line);
    if (line.empty()) { continue; }

    std::vector<std::string> fields;
    boost::split(fields, line, boost::is_any_of(" "));
    if (fields.size() < 3) {
      std::cerr << "not enough fields\n";
      continue;
    }

    const auto cmd = fields[0];
    if (cmd == "r") {
      const auto address = std::stoi(fields[1]);
      const auto size = std::stoi(fields[2]);
      std::vector<char> readbuf;
      readbuf.resize(size);
      spi->Read(address, mjlib::base::string_span(&readbuf[0], readbuf.size()));
      std::cout << FormatHex(std::string_view(&readbuf[0], readbuf.size())) << "\n";
    } else if (cmd == "w") {
      const auto address = std::stoi(fields[1]);
      const auto data = ReadHex(fields[2]);
      spi->Write(address, data);
    } else {
      std::cerr << "unknown command\n";
    }
  }
}
}

int main(int argc, char** argv) {
  std::string spi_device = "/dev/spidev0.0";
  int speed_hz = 20000000;
  int address = 1;
  std::string write_hex;
  size_t read_bytes = 0;
  bool interactive = false;

  po::options_description desc("Allowable options");

  desc.add_options()
      ("help,h", "display usage message")
      ("device,d", po::value(&spi_device), "SPI device")
      ("address,a", po::value(&address), "16 bit address")
      ("speed,s", po::value(&speed_hz), "speed in Hz")
      ("write,w", po::value(&write_hex), "data to write in hex")
      ("read,r", po::value(&read_bytes), "bytes to read")
      ("interactive,i", po::bool_switch(&interactive), "")
      ;

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);

  if (vm.count("help")) {
    std::cout << desc;
    return 0;
  }

  SPI spi(spi_device, speed_hz);

  if (interactive) {
    RunConsole(&spi);
  } else if (!write_hex.empty()) {
    spi.Write(address, ReadHex(write_hex));
  } else if (read_bytes != 0) {
    std::vector<char> readbuf;
    readbuf.resize(read_bytes);
    spi.Read(address, mjlib::base::string_span(&readbuf[0], readbuf.size()));
    std::cout << FormatHex(std::string_view(&readbuf[0], readbuf.size())) << "\n";
  } else {
    std::cerr << "No command specified\n";
  }

  return 0;
}
