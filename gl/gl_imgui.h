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

/// @file
///
/// This file is named gl_imgui.h just so that it will not shadow the
/// official "imgui.h".

#pragma once

#include <imgui.h>

namespace mjmech {
namespace gl {

class ImGuiWindow {
 public:
  ImGuiWindow(const char* name)
      : open_{ImGui::Begin(name)} {
  }

  ~ImGuiWindow() {
    ImGui::End();
  }

  bool open() const {
    return open_;
  }

  operator bool() const {
    return open_;
  }

 private:
  const bool open_;
};

}
}
