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

#include <cmath>

#include <Eigen/Core>

#include "base/common.h"

#include "gl/camera.h"

namespace mjmech {
namespace gl {

class PerspectiveCamera : public Camera {
 public:
  struct Options {
    double fov = 50;
    double aspect = 1;
    double near = 0.1;
    double far = 2000;

    Options() {}
  };
  PerspectiveCamera(const Options& options = {})
      : fov_(options.fov),
        near_(options.near),
        far_(options.far),
        aspect_(options.aspect) {
  }

  ~PerspectiveCamera() override {}

  Type type() const override { return kPerspective; }

  Eigen::Matrix4f matrix(double zoom = 1.0) override {
    double top = near_ * std::tan(base::Radians(0.5 * fov_)) / zoom;
    double height = 2 * top;
    double width = aspect_ * height;
    double left = -0.5 * width;

    return MakePerspective(left, left + width, top,
                           top - height, near_, far_);
  }

  static Eigen::Matrix4f MakePerspective(
      float left, float right, float top, float bottom, float near, float far) {
    Eigen::Matrix4f r;

    float x = 2 * near / (right - left);
    float y = 2 * near / (top - bottom);

    float a = (right + left) / (right - left);
    float b = (top + bottom) / (top - bottom);
    float c = -(far + near) / (far - near);
    float d = -2 * far * near / (far - near);

    r(0, 0) = x;  r(0, 1) = 0;      r(0, 2) = a;   r(0, 3) = 0;
    r(1, 0) = 0;  r(1, 1) = y;      r(1, 2) = b;   r(1, 3) = 0;
    r(2, 0) = 0;  r(2, 1) = 0;      r(2, 2) = c;   r(2, 3) = d;
    r(3, 0) = 0;  r(3, 1) = 0;      r(3, 2) = -1;  r(3, 3) = 0;

    return r;
  }

 private:
  float fov_;
  float near_;
  float far_;
  float aspect_;
};

}
}
