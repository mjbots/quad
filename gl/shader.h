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

#include <string_view>
#include <vector>

#include "gl/gl.h"

#include "mjlib/base/fail.h"

namespace mjmech {
namespace gl {

/// Represents a single compiled shader.
class Shader {
 public:
  Shader(std::string_view source, GLenum type)
      : shader_(glCreateShader(type)) {
    const char* data  = source.data();
    GLint length = source.size();
    glShaderSource(shader_, 1, static_cast<GLchar const**>(&data), &length);
    glCompileShader(shader_);
    GLint status = -1;
    glGetShaderiv(shader_, GL_COMPILE_STATUS, &status);
    if (status != GL_TRUE) {
      GLint log_length = 0;
      glGetShaderiv(shader_, GL_INFO_LOG_LENGTH, &length);
      std::vector<char> log;
      log.resize(log_length);
      glGetShaderInfoLog(shader_, log_length, &log_length, log.data());

      mjlib::base::Fail("Error compiling shader: " +
                        std::string(log.data(), log_length));
    }
    TRACE_GL_ERROR();
  }

  ~Shader() {
    glDeleteShader(shader_);
  }

  Shader(const Shader&) = delete;
  Shader& operator=(const Shader&) = delete;

  GLuint get() const { return shader_; }

 private:
  const GLuint shader_;
};

}
}
