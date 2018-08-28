// Copyright 2014-2015 Josh Pieper, jjp@pobox.com.  All rights reserved.
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

#include "base/linux_input.h"

#include <linux/input.h>

#include <iostream>

#include <boost/program_options.hpp>

#include "base/fail.h"

namespace {
using namespace mjmech::base;

class Reader {
 public:
  Reader(LinuxInput* input, bool absinfo)
      : input_(input), absinfo_(absinfo) {}

  void StartRead() {
    input_->AsyncRead(
        &event_, std::bind(&Reader::HandleRead, this,
                           std::placeholders::_1));
  }

  void HandleRead(ErrorCode ec) {
    FailIf(ec);

    std::cout << event_ << "\n";
    if (event_.ev_type == EV_ABS && absinfo_) {
      std::cout << input_->abs_info(event_.code) << "\n";
    }

    StartRead();
  }

  LinuxInput* const input_;
  const bool absinfo_;
  LinuxInput::Event event_;
};

int work(int argc, char** argv) {
  namespace po = boost::program_options;

  std::string device;
  bool wait = false;
  bool absinfo = false;
  po::options_description desc("Allowable options");
  desc.add_options()
      ("help,h", "display usage message")
      ("device,d", po::value(&device), "input device")
      ("wait,w", po::bool_switch(&wait), "wait for events")
      ("absinfo,a", po::bool_switch(&absinfo), "display absinfo")
      ;

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);

  if (vm.count("help")) {
    std::cerr << desc;
    return 1;
  }


  boost::asio::io_service service;
  LinuxInput linux_input(service);
  linux_input.Open(device);

  std::cout << linux_input << "\n";
  std::cout << linux_input.features(EV_REL) << "\n";
  std::cout << linux_input.features(EV_ABS) << "\n";
  std::cout << linux_input.features(EV_KEY) << "\n";

  auto abs_features = linux_input.features(EV_ABS);
  for (size_t i = 0; i < abs_features.capabilities.size(); i++) {
    if (!abs_features.capabilities.test(i)) { continue; }
    std::cout << linux_input.abs_info(i) << "\n";
  }

  if (wait) {
    Reader reader(&linux_input, absinfo);
    reader.StartRead();
    service.run();
  }
  return 0;
}
}

extern "C" {
int main(int argc, char** argv) {
  try {
    return work(argc, argv);
  } catch (std::exception& e) {
    std::cerr << "error: " << e.what() << "\n";
    return 1;
  }
  return 0;
}
}
