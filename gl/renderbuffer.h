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

#include <GL/gl3w.h>

namespace mjmech {
namespace gl {

class Renderbuffer {
 public:
  Renderbuffer() {
    glGenRenderbuffers(1, &rbo_);
  }

  ~Renderbuffer() {
    glDeleteRenderbuffers(1, &rbo_);
  }

  void bind() const {
    glBindRenderbuffer(GL_RENDERBUFFER, rbo_);
  }

  GLuint id() const { return rbo_; }

 private:
  GLuint rbo_ = 0;
};

}
}
