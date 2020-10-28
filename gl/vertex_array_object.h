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

#pragma once

#include "mjlib/imgui/gl.h"

namespace mjmech {
namespace gl {

class VertexArrayObject {
 public:
  VertexArrayObject() {
    glGenVertexArrays(1, &vao_);
  }

  ~VertexArrayObject() {
    glDeleteVertexArrays(1, &vao_);
  }

  VertexArrayObject(const VertexArrayObject&) = delete;
  VertexArrayObject& operator=(const VertexArrayObject&) = delete;

  void bind() {
    glBindVertexArray(vao_);
  }

  void unbind() {
    glBindVertexArray(0);
  }

 private:
  GLuint vao_ = 0;
};

}
}
