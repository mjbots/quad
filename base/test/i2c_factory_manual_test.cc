// Copyright 2016 Josh Pieper, jjp@pobox.com.  All rights reserved.
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

#include "base/i2c_factory.h"

#include <boost/asio/read_until.hpp>
#include <boost/asio/posix/stream_descriptor.hpp>
#include <boost/asio/streambuf.hpp>
#include <boost/format.hpp>
#include <boost/program_options.hpp>

#include "base/fail.h"
#include "base/linux_i2c_generator.h"
#include "base/program_options.h"
#include "base/tokenizer.h"

namespace po = boost::program_options;

using namespace mjmech::base;
using namespace std::placeholders;

namespace {
class I2CInterface {
 public:
  I2CInterface(boost::asio::io_service& service)
      : service_(service),
        in_(service, dup(0)),
        out_(service, dup(1)) {}

  void Start() {
    boost::asio::async_read_until(
        in_, streambuf_, "\n",
        std::bind(&I2CInterface::HandleLine, this, _1, _2));
  }

  void SetI2C(SharedI2C i2c) { i2c_ = i2c; }

 private:
  void HandleLine(ErrorCode ec, std::size_t size) {
    FailIf(ec);

    std::istream in(&streambuf_);
    std::string line;
    std::getline(in, line);

    Tokenizer tokenizer(line, " ");
    auto cmd = tokenizer.next();
    if (cmd == "read") {
      const int device = std::stoi(tokenizer.next(), 0, 0);
      const int address = std::stoi(tokenizer.next(), 0, 0);
      const int length = std::stoi(tokenizer.next(), 0, 0);
      i2c_->AsyncRead(device, address, boost::asio::buffer(buffer_, length),
                      std::bind(&I2CInterface::HandleRead, this, _1, _2));
    }

    std::cout << "got line: " << line << "\n";
    Start();
  }

  void HandleRead(ErrorCode ec, std::size_t length) {
    FailIf(ec);
    std::cout << "rx complete:";
    for (std::size_t i = 0; i < length; i++) {
      std::cout << boost::format(" %02X") % static_cast<int>(buffer_[i]);
    }
    std::cout << "\n";
  }

  boost::asio::io_service& service_;
  boost::asio::posix::stream_descriptor in_;
  boost::asio::posix::stream_descriptor out_;
  boost::asio::streambuf streambuf_;
  SharedI2C i2c_;
  char buffer_[256] = {};
};
}

int main(int argc, char** argv) {
  boost::asio::io_service service;

  po::options_description options;
  options.add_options()
      ("help,h", "display usage message")
      ;

  I2CFactory factory(service);
  factory.Register(std::unique_ptr<LinuxI2CGenerator>(
                       new LinuxI2CGenerator(service)));
  std::unique_ptr<I2CFactory::Parameters> parameters =
      factory.MakeParameters();

  MergeProgramOptions(parameters->options_description(), "i2c.", &options);

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, options), vm);
  po::notify(vm);

  if (vm.count("help")) {
    std::cerr << options;
    return 1;
  }

  I2CInterface interface(service);

  interface.Start();

  factory.AsyncCreate(*parameters, [&](ErrorCode ec, SharedI2C i2c) {
      FailIf(ec);
      interface.SetI2C(i2c);
      std::cout << "connected!\n";
    });

  service.run();

  return 0;
}
