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

#include MODULE_HEADER_FILE

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/filesystem.hpp>
#include <boost/format.hpp>
#include <boost/program_options.hpp>

#include "base/fail.h"
#include "base/handler_util.h"
#include "base/logging.h"
#include "base/program_options_archive.h"
#include "base/telemetry_log.h"
#include "base/telemetry_log_registrar.h"
#include "base/telemetry_registry.h"
#include "base/telemetry_remote_debug_registrar.h"
#include "base/telemetry_remote_debug_server.h"

using namespace mjmech::base;
using namespace mjmech::mech;

namespace {
struct Context {
  boost::asio::io_service service;
  TelemetryLog telemetry_log;
  TelemetryRemoteDebugServer remote_debug{service};
  TelemetryRegistry<TelemetryLogRegistrar,
                    TelemetryRemoteDebugRegistrar> telemetry_registry{
    &telemetry_log, &remote_debug};
};

int safe_main(int argc, char**argv) {
  namespace po = boost::program_options;

  Context context;
  MODULE_CLASS_NAME module(context);

  std::string config_file;
  std::string log_file;
  bool debug = false;

  po::options_description desc("Allowable options");
  desc.add_options()
      ("help,h", "display usage message")
      ("config,c", po::value(&config_file), "read options from file")
      ("log,l", po::value(&log_file), "write to log file")
      ("debug,d", po::bool_switch(&debug),
       "disable real-time signals and other debugging hindrances")
      ;

  AddLoggingOptions(&desc);
  ProgramOptionsArchive(&desc, "remote_debug.").Accept(
      context.remote_debug.parameters());
  ProgramOptionsArchive(&desc).Accept(module.parameters());

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);

  InitLogging();

  if (!config_file.empty()) {
    po::store(po::parse_config_file<char>(config_file.c_str(), desc), vm);
  }
  po::notify(vm);

  if (vm.count("help")) {
    std::cerr << desc;
    return 0;
  }

  if (!log_file.empty()) {
    context.telemetry_log.SetRealtime(!debug);

    // Make sure that the log file has a date and timestamp somewhere
    // in the name.
    namespace fs = boost::filesystem;
    fs::path log_file_path(log_file);
    std::string extension = log_file_path.extension().native();

    const auto now = boost::posix_time::microsec_clock::universal_time();
    std::string datestamp =
        (boost::format("%s-%02d%02d%02d") %
         to_iso_string(now.date()) %
         now.time_of_day().hours() %
         now.time_of_day().minutes() %
         now.time_of_day().seconds()).str();

    const std::string stem = log_file_path.stem().native();

    fs::path stamped_path = log_file_path.parent_path() /
        (boost::format("%s-%s%s") %
         stem % datestamp % extension).str();

    context.telemetry_log.Open(stamped_path.native());
  }

  std::shared_ptr<ErrorHandlerJoiner> joiner =
      std::make_shared<ErrorHandlerJoiner>(
          [=](ErrorCode ec) {
            FailIf(ec);
            if (debug) { std::cout << "Started!\n"; }
          });

  context.remote_debug.AsyncStart(joiner->Wrap("starting remote_debug"));
  module.AsyncStart(joiner->Wrap("starting main module"));

  context.service.run();
  return 0;
}
}

int main(int argc, char**argv) {
  try {
    return safe_main(argc, argv);
  } catch (std::exception& e) {
    std::cerr << "Error: " << e.what() << "\n";
    return 1;
  } catch (...) {
    std::cerr << "Unknown error.\n";
    return 2;
  }
}
