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

#include "gl/gl.h"

namespace mjmech {
namespace gl {

class VertexBufferObject {
 public:
  VertexBufferObject() {
    glGenBuffers(1, &vbo_);
  }

  ~VertexBufferObject() {
    glDeleteBuffers(1, &vbo_);
  }

  VertexBufferObject(const VertexBufferObject&) = delete;
  VertexBufferObject& operator=(const VertexBufferObject&) = delete;

  void bind(GLenum target) {
    glBindBuffer(target, vbo_);
  }

  template <typename Vector>
  void set_vector(GLenum target, const Vector& vector, GLenum usage) {
    bind(target);
    glBufferData(target, vector.size() * sizeof(vector[0]), &vector[0], usage);
  }

  template <typename Array>
  void set_data_array(GLenum target, const Array& array, GLenum usage) {
    bind(target);
    glBufferData(target, sizeof(array), &array[0], usage);
  }

 private:
  GLuint vbo_ = 0;
};

}
}
