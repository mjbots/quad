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

#pragma once

#include <dart/gui/SimWindow.hpp>

#include <clipp/clipp.h>

#include "base/context.h"

namespace mjmech {
namespace simulator {

class SimulatorWindow {
 public:
  SimulatorWindow(base::Context&);
  ~SimulatorWindow();

  clipp::group program_options();

  void AsyncStart(mjlib::io::ErrorCallback);
  void InitWindow(int x, int y, const char* name);

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}
}
