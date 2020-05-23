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

#include <opencv2/imgcodecs.hpp>

#include "tools/cpp/runfiles/runfiles.h"

#include "base/runfiles.h"

#include "gl/flat_rgb_texture.h"
#include "gl/gl.h"

namespace mjmech {
namespace gl {

/// A GL texture read from a source file.
class ImageTexture {
 public:
  ImageTexture(std::string_view filepath)
      : runfiles_(),
        image_{cv::imread(runfiles_.Rlocation(filepath), cv::IMREAD_UNCHANGED)},
        texture_(EigenSize(image_.size()),
                 image_.channels() == 3 ? GL_RGB : GL_RGBA) {
    texture_.Store(image_.data);
    TRACE_GL_ERROR();
  }

  void bind() {
    texture_.bind();
  }

  GLuint id() const { return texture_.id(); }
  Texture& texture() { return texture_.texture(); }

 private:
  template <typename T>
  static Eigen::Vector2i EigenSize(const T& value) {
    return {value.width, value.height};
  }

  base::Runfiles runfiles_;
  cv::Mat image_;
  FlatRgbTexture texture_;
};

}
}
