// Copyright 2014-2020 Josh Pieper, jjp@pobox.com.
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

#include <clipp/clipp.h>

#include <boost/asio/io_context.hpp>

#include "mjlib/base/clipp.h"
#include "mjlib/base/fail.h"

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

  void HandleRead(mjlib::base::error_code ec) {
    mjlib::base::FailIf(ec);

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
  std::string device;
  bool wait = false;
  bool absinfo = false;
  auto group = clipp::group(
      (clipp::option("d", "device") & clipp::value("", device)) % "input device",
      (clipp::option("w", "wait").set(wait)) % "wait for events",
      (clipp::option("a", "absinfo").set(absinfo)) % "display absinfo"
  );

  mjlib::base::ClippParse(argc, argv, group);

  boost::asio::io_context context;
  LinuxInput linux_input(context.get_executor());
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
    context.run();
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
