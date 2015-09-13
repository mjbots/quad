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

#include "telemetry_log.h"
#include "telemetry_log_registrar.h"
#include "telemetry_registry.h"
#include "visitor.h"

#include <boost/program_options.hpp>

namespace {
using namespace mjmech::base;

struct SampleStruct {
  boost::posix_time::ptime timestamp;
  double value1 = 1;
  double value2 = 2;
  double value3 = 3;
  std::array<uint8_t, 20> array = {};
  int32_t value4 = 4;
  int64_t value5 = 5;
  int64_t value6 = 6;
  uint64_t value7 = 7;
  uint64_t value8 = 8;
  std::array<double, 6> value9 = {};

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(timestamp));
    a->Visit(MJ_NVP(value1));
    a->Visit(MJ_NVP(value2));
    a->Visit(MJ_NVP(value3));
    a->Visit(MJ_NVP(array));
    a->Visit(MJ_NVP(value4));
    a->Visit(MJ_NVP(value5));
    a->Visit(MJ_NVP(value6));
    a->Visit(MJ_NVP(value7));
    a->Visit(MJ_NVP(value8));
    a->Visit(MJ_NVP(value9));
  }
};

int work(int argc, char** argv) {
  namespace po = boost::program_options;

  po::options_description desc("Allowable options");

  std::string output;
  bool realtime = false;
  int count = 1;
  int delay_us = 10000;
  int size = 100;
  bool structure = false;
  bool disable_compression = false;
  desc.add_options()
      ("output,o", po::value(&output), "output file")
      ("realtime,r", po::bool_switch(&realtime), "realtime mode")
      ("count,c", po::value(&count), "blocks to write")
      ("delay_us,d", po::value(&delay_us), "delay between blocks")
      ("size,s", po::value(&size), "size of blocks")
      ("struct", po::bool_switch(&structure), "serialize structure")
      ("disable-compression", po::bool_switch(&disable_compression),
       "disable compression")
      ("help,h", "display usage message")
      ;

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);

  if (vm.count("help")) {
    std::cerr << desc;
    return 0;
  }

  int flags = TelemetryLog::kDefaultFlags;
  if (disable_compression) {
    flags &= ~TelemetryLog::LogFlags::kDefaultCompression;
  }
  TelemetryLog log(static_cast<TelemetryLog::LogFlags>(flags));
  TelemetryRegistry<TelemetryLogRegistrar> registry(&log);
  auto callable = registry.Register<SampleStruct>("sample");

  log.SetRealtime(realtime);
  log.Open(output);

  std::string to_write(size, ' ');

  for (int i = 0; i < count; i++) {
    if (!structure) {
      auto buffer = log.GetBuffer();
      buffer->write(to_write.data(), to_write.size());
      log.WriteBlock(TelemetryFormat::BlockType::kBlockData,
                     std::move(buffer));
    } else {
      SampleStruct to_log;
      to_log.timestamp = boost::posix_time::microsec_clock::universal_time();
      callable(&to_log);
    }
    ::usleep(delay_us);
  }

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
