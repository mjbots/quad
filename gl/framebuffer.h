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

#include <cstdlib>

#include <fmt/format.h>

#include "gl/renderbuffer.h"
#include "gl/texture.h"

namespace mjmech {
namespace gl {

class Framebuffer {
 public:
  Framebuffer() {
    glGenFramebuffers(1, &framebuffer_);
  }

  ~Framebuffer() {
    glDeleteFramebuffers(1, &framebuffer_);
  }

  class Bind {
   public:
    Bind(Framebuffer& parent) {
      glBindFramebuffer(GL_FRAMEBUFFER, parent.id());
    }

    ~Bind() {
      glBindFramebuffer(GL_FRAMEBUFFER, 0);
    }
  };

  void attach(const Texture& texture, const Renderbuffer& rbo) {
    Bind binder(*this);

    glFramebufferTexture(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, texture.id(), 0);

    rbo.bind();

    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH24_STENCIL8,
                          texture.size().x(), texture.size().y());

    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT,
                              GL_RENDERBUFFER, rbo.id());

    GLenum draw_buffers[] = { GL_COLOR_ATTACHMENT0 };
    glDrawBuffers(1, draw_buffers);
  }

  GLuint id() const { return framebuffer_; }

 private:
  GLuint framebuffer_ = -1;
};

}
}
