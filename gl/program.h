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

#include <Eigen/Core>

#include "mjlib/base/fail.h"
#include "mjlib/base/system_error.h"

#include "base/named_type.h"
#include "gl/gl.h"
#include "gl/shader.h"

namespace mjmech {
namespace gl {

using Attribute = base::NamedType<GLuint, struct AttributeStruct>;
using Uniform = base::NamedType<GLuint, struct UniformStruct>;

class Program {
 public:
  Program(const Shader& vertex_shader, const Shader& fragment_shader)
      : program_(glCreateProgram()) {
    glAttachShader(program_, vertex_shader.get());
    glAttachShader(program_, fragment_shader.get());
    glLinkProgram(program_);
    GLint status = 0;
    glGetProgramiv(program_, GL_LINK_STATUS, &status);
    if (status != GL_TRUE) {
      GLint log_length = 0;
      glGetShaderiv(program_, GL_INFO_LOG_LENGTH, &log_length);
      std::vector<char> log;
      log.resize(log_length);
      glGetShaderInfoLog(program_, log_length, &log_length, log.data());

      mjlib::base::Fail("Error linking program: " +
                        std::string(log.data(), log_length));
    }
    TRACE_GL_ERROR();
  }

  ~Program() {
    glDeleteProgram(program_);
  }

  Program(const Program&) = delete;
  Program& operator=(const Program&) = delete;

  Uniform uniform(std::string_view name) {
    try {
      return Uniform(
          VerifyNonNegative(glGetUniformLocation(program_, name.data())));
    } catch (mjlib::base::system_error& se) {
      se.code().Append("Name: " + std::string(name));
      throw;
    }
  }

  Attribute attribute(std::string_view name) {
    try {
      return Attribute(
          VerifyNonNegative(glGetAttribLocation(program_, name.data())));
    } catch (mjlib::base::system_error& se) {
      se.code().Append("Name: " + std::string(name));
      throw;
    }
  }

  void SetUniform(Uniform uniform, int value) {
    glUniform1i(uniform.get(), value);
  }

  void SetUniform(Uniform uniform, float value) {
    glUniform1f(uniform.get(), value);
  }

  template <int RowsAtCompileTime, int ColsAtCompileTime, int Options>
  void SetUniform(
      Uniform uniform,
      const Eigen::Matrix<float, RowsAtCompileTime, ColsAtCompileTime,
      Options>& matrix) {
    // We don't support row-major order.
    static_assert(Options == 0);

    if constexpr (RowsAtCompileTime == 4 && ColsAtCompileTime == 4) {
      glUniformMatrix4fv(uniform.get(), 1, GL_FALSE, matrix.data());
    } else if constexpr (RowsAtCompileTime == 3 && ColsAtCompileTime == 1) {
      glUniform3fv(uniform.get(), 1, matrix.data());
    } else {
      mjlib::base::Fail("Unsupported uniform matrix type");
    }
  }

  void use() {
    glUseProgram(program_);
  }

  GLuint get() const { return program_; }

  // The corresponding VAO and VBO must be activated and populated
  // before calling this.
  void VertexAttribPointer(
      Attribute attribute, GLint size, GLenum type,
      GLboolean normalized, GLsizei stride, int offset) {
    glEnableVertexAttribArray(attribute.get());
    glVertexAttribPointer(attribute.get(), size, type, normalized, stride,
                          reinterpret_cast<void*>(offset));
  }

 private:
  GLint VerifyNonNegative(GLint value) {
    if (value < 0) {
      TRACE_GL_ERROR();
      throw mjlib::base::system_error::einval(
          "Invalid uniform or attribute name");
    }
    return value;
  }

  const GLuint program_;
};

}
}
