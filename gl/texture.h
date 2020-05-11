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

class Texture {
 public:
  Texture() {
    glGenTextures(1, &tex_);
  }

  ~Texture() {
    glDeleteTextures(1, &tex_);
  }

  Texture(const Texture&) = delete;
  Texture& operator=(const Texture&) = delete;

  void bind(GLenum target, GLenum texture = GL_TEXTURE0) {
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(target, tex_);
  }

  GLuint id() const { return tex_; }

 private:
  GLuint tex_ = 0;
};

}
}
