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

#include <Eigen/Core>

#include "gl/texture.h"

namespace mjmech {
namespace gl {

/// A GL texture which can have RGB data sent into it.
class FlatRgbTexture {
 public:
  FlatRgbTexture(Eigen::Vector2i size, GLenum format = GL_RGB)
      : size_(size),
        format_(format) {
    texture_.bind(GL_TEXTURE_2D);

    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

    glTexImage2D(GL_TEXTURE_2D, 0, format_,
                 size.x(), size.y(),
                 0, GL_RGB, GL_UNSIGNED_BYTE, NULL);
  }

  void Store(const void* data) {
    bind();
    glTexSubImage2D(
        GL_TEXTURE_2D, 0, 0, 0,
        size_.x(), size_.y(),
        format_, GL_UNSIGNED_BYTE,
        data);
  }

  void bind() {
    texture_.bind(GL_TEXTURE_2D);
  }

  GLuint id() const { return texture_.id(); }
  Texture& texture() { return texture_; }

 private:
  Texture texture_;
  const Eigen::Vector2i size_;
  const GLenum format_;
};

}
}
