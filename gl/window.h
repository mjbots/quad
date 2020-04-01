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

#include <Eigen/Core>

#include "mjlib/base/fail.h"

#include "gl/gl.h"

/// GL must be included *before* we include glfw
#include <GLFW/glfw3.h>

namespace mjmech {
namespace gl {

class Window {
 public:
  Window(int width, int height, std::string_view name) {
    glfwSetErrorCallback(glfw_error_callback);

    if (!glfwInit()) {
      mjlib::base::Fail("glfw init failed");
    }

    // GL 3.0 + GLSL 130
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, kGlVersionMajor);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, kGlVersionMinor);

    window_ = glfwCreateWindow(width, height, name.data(), nullptr, nullptr);

    if (window_ == nullptr) {
      mjlib::base::Fail("Could not create glfw window");
    }

    glfwMakeContextCurrent(window_);
    glfwSwapInterval(1);

    const bool err = gl3wInit() != 0;
    if (err) {
      mjlib::base::Fail("Could not initialize gl3w");
    }

    TRACE_GL_ERROR();
  }

  ~Window() {
    glfwDestroyWindow(window_);
    glfwTerminate();
  }

  GLFWwindow* window() const { return window_; }

  void PollEvents() {
    TRACE_GL_ERROR();
    glfwPollEvents();
    TRACE_GL_ERROR();
  }

  void SwapBuffers() {
    TRACE_GL_ERROR();
    glfwSwapBuffers(window_);
    TRACE_GL_ERROR();
  }

  bool should_close() {
    return glfwWindowShouldClose(window_);
  }

  Eigen::Vector2i framebuffer_size() {
    int result_w = 0;
    int result_h = 0;
    glfwGetFramebufferSize(window_, &result_w, &result_h);
    return { result_w, result_h };
  }

 private:
  static void glfw_error_callback(int error, const char* description) {
    mjlib::base::Fail(fmt::format("glfw error {}: {}\n", error, description));
  }

  GLFWwindow* window_ = nullptr;
};

}
}
