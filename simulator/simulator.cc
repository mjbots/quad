// Copyright 2015-2020 Josh Pieper, jjp@pobox.com.
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

#include <fstream>

#include <dart/gui/LoadGlut.hpp>

#include "mjlib/base/clipp.h"

#include "base/logging.h"

#include "simulator/simulator_window.h"

using namespace mjmech;
using namespace mjmech::simulator;

int main(int argc, char** argv) {
  std::string config_file;
  std::string log_file;

  auto group = (
      (clipp::option("c", "config") & clipp::value("", config_file)) %
      "read options from file",
      (clipp::option("l", "log") & clipp::value("", log_file)) %
      "write to log file"
  );

  group.push_back(base::MakeLoggingOptions());

  base::Context context;
  SimulatorWindow window(context);

  group.push_back(window.program_options());

  mjlib::base::ClippParse(argc, argv, group);

  base::InitLogging();

  if (!config_file.empty()) {
    std::ifstream inf(config_file);
    mjlib::base::system_error::throw_if(
        !inf.is_open(), "opening "  + config_file);
    mjlib::base::ClippParseIni(inf, group);
  }

  glutInit(&argc, argv);
  window.InitWindow(640, 480, "Mech Simulator");
  window.AsyncStart(mjlib::base::FailIf);
  glutMainLoop();
}
