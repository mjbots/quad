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

#include "gl/camera.h"

namespace mjmech {
namespace gl {

class OrthographicCamera : public Camera {
 public:
  struct Options {
    double left = -1;
    double right = 1;
    double top = -1;
    double bottom = 1;
    double near = 0.1;
    double far = 2000.0;

    Options() {}
  };

  OrthographicCamera(const Options& options = {})
      : left_(options.left),
        right_(options.right),
        top_(options.top),
        bottom_(options.bottom),
        near_(options.near),
        far_(options.far) {}

  ~OrthographicCamera() override {}

  Type type() const override { return kOrthographic; }

  Eigen::Matrix4f matrix(double zoom = 1.0) override {
    double dx = (right_ - left_) / (2 * zoom);
    double dy = (top_ - bottom_) / (2 * zoom);
    double cx = (right_ + left_) / 2.0;
    double cy = (top_ + bottom_) / 2.0;

    double left = cx - dx;
    double right = cx + dx;
    double top = cy + dy;
    double bottom = cy - dy;
    return MakeOrtho(left, right, top, bottom, near_, far_);
  }

  static Eigen::Matrix4f MakeOrtho(
      float left, float right, float top, float bottom, float near, float far) {
    Eigen::Matrix4f r;

    r(0, 0) = 2.0f / (right - left);
    r(1, 1) = 2.0f / (top - bottom);
    r(2, 2) = -1.0f / (far - near);
    r(3, 0) = -(right + left) / (right - left);
    r(3, 1) = -(top + bottom) / (top - bottom);
    r(3, 2) = -near / (far - near);
    r(3, 3) = 1.0f;
    return r;
  }

 private:
  double left_;
  double right_;
  double top_;
  double bottom_;
  double near_;
  double far_;
};

}
}
