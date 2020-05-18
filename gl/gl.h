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

namespace mjmech {
namespace gl {

constexpr int kGlVersionMajor = 4;
constexpr int kGlVersionMinor = 0;
constexpr const char* kGlslVersion = "#version 400";

inline void TraceGlError(const char* file, int line) {
  const auto v = glGetError();
  if (v == GL_NO_ERROR) { return; }

  fmt::print(stderr, "{}:{} glError()={}\n", file, line,v);
  std::abort();
}

#ifdef MJMECH_ENABLE_TRACE_GL_ERROR
#define TRACE_GL_ERROR() ::mjmech::gl::TraceGlError(__FILE__, __LINE__)
#else
#define TRACE_GL_ERROR()
#endif

}
}
