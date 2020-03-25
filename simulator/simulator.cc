// Copyright 2015-2020 Josh Pieper, jjp@pobox.com.  All rights reserved.
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

#include <boost/program_options.hpp>

#include <dart/gui/LoadGlut.hpp>

#include "base/logging.h"
#include "base/program_options.h"

#include "simulator/simulator_window.h"

using namespace mjmech;
using namespace mjmech::simulator;

int main(int argc, char** argv) {
  namespace po = boost::program_options;

  std::string config_file;

  po::options_description desc("Allowable options");
  desc.add_options()
      ("help,h", "display usage message")
      ("config,c", po::value(&config_file), "read options from file")
      ;
  base::AddLoggingOptions(&desc);

  base::Context context;
  SimulatorWindow window(context);
  base::MergeProgramOptions(window.options(), "", &desc);

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);

  if (vm.count("help")) {
    std::cerr << desc;
    return 0;
  }

  base::InitLogging();

  if (!config_file.empty()) {
    po::store(po::parse_config_file<char>(config_file.c_str(), desc), vm);
  }
  po::notify(vm);

  glutInit(&argc, argv);
  window.InitWindow(640, 480, "Mech Simulator");
  window.AsyncStart(mjlib::base::FailIf);
  glutMainLoop();
}
