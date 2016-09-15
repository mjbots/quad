// Copyright 2015 Josh Pieper, jjp@pobox.com.  All rights reserved.
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

#include "comm_factory.h"

#include <functional>
#include <iostream>

#include <boost/asio/read.hpp>
#include <boost/asio/spawn.hpp>
#include <boost/asio/streambuf.hpp>
#include <boost/asio/write.hpp>
#include <boost/exception/all.hpp>
#include <boost/program_options.hpp>

#include "common.h"
#include "error_wrap.h"
#include "program_options_archive.h"

using namespace std::placeholders;
using namespace mjmech::base;
namespace po = boost::program_options;

namespace {
const auto transfer = [](const boost::system::error_code& ec,
                         std::size_t bytes) -> std::size_t {
  if (ec || bytes) { return 0; }
  return 4096;
};

int work(int argc, char** argv) {
  typedef StreamFactory<StdioGenerator,
                        SerialPortGenerator,
                        TcpClientGenerator,
                        TcpServerGenerator> Factory;

  boost::asio::io_service service;

  Factory factory(service);
  Factory::Parameters parameters;
  po::options_description desc("Allowable options");

  desc.add_options()("help,h", "display usage message");
  ProgramOptionsArchive(&desc).Accept(&parameters);

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);

  if (vm.count("help")) {
    std::cerr << desc;
    return 1;
  }

  boost::asio::spawn(service, ErrorWrap([&](boost::asio::yield_context yield) {
      Factory::Parameters stdio_params;
      stdio_params.type = "stdio";
      SharedStream stdio = factory.AsyncCreate(stdio_params, yield);
      SharedStream stream = factory.AsyncCreate(parameters, yield);

      std::cout << "Connected!\n";

      boost::asio::spawn(yield, [stdio, stream](
                             boost::asio::yield_context yield) {
          boost::asio::streambuf streambuf;
          while (true) {
            boost::asio::async_read(*stdio, streambuf, transfer, yield);
            boost::asio::async_write(*stream, streambuf, yield);
          }
        });

      boost::asio::spawn(yield, [stdio, stream](
                             boost::asio::yield_context yield) {
          boost::asio::streambuf streambuf;
          while (true) {
            boost::asio::async_read(*stream, streambuf, transfer, yield);
            boost::asio::async_write(*stdio, streambuf, yield);
          }
        });
      }));

  service.run();
  return 0;
}
}

int main(int argc, char** argv) {
  try {
    return work(argc, argv);
  } catch (std::exception& e) {
    std::cerr << "error: " << e.what() << "\n";
    return 1;
  }
  return 0;
}
