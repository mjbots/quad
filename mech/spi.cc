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

#include <cstdint>
#include <iostream>

#include <fmt/format.h>

#include <boost/algorithm/string.hpp>
#include <boost/program_options.hpp>

#include "mjlib/base/string_span.h"
#include "mjlib/base/system_error.h"

#include "mech/rpi3_raw_aux_spi.h"
#include "mech/spidev.h"

namespace po = boost::program_options;

namespace {
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

class SpiBase {
 public:
  virtual ~SpiBase() {}
  virtual void Write(int, std::string_view) = 0;
  virtual void Read(int, mjlib::base::string_span) = 0;
};

class SpiDevWrapper : public SpiBase {
 public:
  SpiDevWrapper(const std::string& filename, int speed_hz)
      : spi_(filename, speed_hz) {}
  ~SpiDevWrapper() override {}

  void Write(int address, std::string_view data) override {
    spi_.Write(address, data);
  }

  void Read(int address, mjlib::base::string_span data) override {
    spi_.Read(address, data);
  }

  mjmech::mech::SpiDev spi_;
};

class RawAuxSpiWrapper : public SpiBase {
 public:
  RawAuxSpiWrapper(const mjmech::mech::Rpi3RawAuxSpi::Options& options, int cs)
      : spi_(options), cs_(cs) {}
  ~RawAuxSpiWrapper() override {}

  void Write(int address, std::string_view data) override {
    spi_.Write(cs_, address, data);
  }

  void Read(int address, mjlib::base::string_span data) override {
    spi_.Read(cs_, address, data);
  }

 private:
  mjmech::mech::Rpi3RawAuxSpi spi_;
  const int cs_;
};

template <typename Spi>
void RunInteractive(Spi* spi) {
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
  std::string mode = "spidev";
  int cs = 0;
  int speed_hz = 20000000;
  int address = 1;
  std::string write_hex;
  size_t read_bytes = 0;
  bool performance = false;
  bool interactive = false;

  po::options_description desc("Allowable options");

  desc.add_options()
      ("help,h", "display usage message")
      ("device,d", po::value(&spi_device), "SPI device")
      ("mode", po::value(&mode), "spidev/raw")
      ("cs", po::value(&cs), "CS line to use")
      ("address,a", po::value(&address), "16 bit address")
      ("speed,s", po::value(&speed_hz), "speed in Hz")
      ("write,w", po::value(&write_hex), "data to write in hex")
      ("read,r", po::value(&read_bytes), "bytes to read")
      ("performance,p", po::bool_switch(&performance), "")
      ("interactive,i", po::bool_switch(&interactive), "")
      ;

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);

  if (vm.count("help")) {
    std::cout << desc;
    return 0;
  }

  std::unique_ptr<SpiBase> spi;
  if (mode == "spidev") {
    spi = std::make_unique<SpiDevWrapper>(spi_device, speed_hz);
  } else if (mode == "raw") {
    spi = std::make_unique<RawAuxSpiWrapper>([&]() {
        mjmech::mech::Rpi3RawAuxSpi::Options options;
        options.speed_hz = speed_hz;
        return options;
      }(),
      cs);
  } else {
    std::cerr << "Unknown mode\n";
    return 1;
  }

  if (performance) {
    spi->Write(18, std::string_view("\x01\x00\x00\x80\x01\x03\x42\x01\x30", 9));
    for (int i = 0; i < 30; i++) {
      char buf[6] = {};
      spi->Read(16, buf);
      if (buf[0] != 0) {
        std::vector<char> readbuf;
        readbuf.resize(buf[0]);
        spi->Read(17, mjlib::base::string_span(&readbuf[0], readbuf.size()));
        std::cout << FormatHex({&readbuf[0], readbuf.size()}) << "\n";
        return 0;
      }
    }
    std::cout << "timeout\n";
  } else if (interactive) {
    RunInteractive(spi.get());
  } else if (!write_hex.empty()) {
    spi->Write(address, ReadHex(write_hex));
  } else if (read_bytes != 0) {
    std::vector<char> readbuf;
    readbuf.resize(read_bytes);
    spi->Read(address, mjlib::base::string_span(&readbuf[0], readbuf.size()));
    std::cout << FormatHex(std::string_view(&readbuf[0], readbuf.size())) << "\n";
  } else {
    std::cerr << "No command specified\n";
  }

  return 0;
}
