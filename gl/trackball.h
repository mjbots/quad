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
#include <Eigen/Geometry>

namespace mjmech {
namespace gl {

/// Implement trackball mouse controls.  Roughly ported from
/// https://github.com/mrdoob/three.js/blob/dev/examples/jsm/controls/TrackballControls.js
class Trackball {
 public:
  struct Options {
    float rotate_speed = 1.0f;
    float zoom_speed = 1.2f;
    float pan_speed = 0.3f;
    float dynamic_damping_factor = 0.2f;
    float min_distance = 0.0f;
    float max_distance = std::numeric_limits<float>::infinity();

    Options() {}
  };

  Trackball(const Eigen::Vector3f& initial_position,
            const Eigen::Vector3f& target,
            const Eigen::Vector3f& up,
            const Options& options = Options())
      : options_(options) {
    state_.position = initial_position;
    state_.target = target;
    state_.up = up;
    Update();
  }

  Eigen::Matrix4f matrix() const {
    return view_;
  }

  bool active() const {
    return mode_ != Mode::kNone;
  }

  void MouseDown(Eigen::Vector2f position, int button) {
    if (!enabled) { return; }

    if (mode_ == Mode::kNone) {
      if (button == 0) {
        mode_ = Mode::kRotate;
      } else if (button == 1) {
        mode_ = Mode::kZoom;
      } else if (button == 2) {
        mode_ = Mode::kPan;
      } else {
        mode_ = Mode::kNone;
      }
    }

    if (mode_ == Mode::kRotate && !no_rotate) {
      move_curr_ = GetMouseOnCircle(position);
      move_prev_ = move_curr_;
    } else if (mode_ == Mode::kZoom && !no_zoom) {
      zoom_start_ = GetMouseOnScreen(position);
      zoom_end_ = zoom_start_;
    } else if (mode_ == Mode::kPan && !no_pan) {
      pan_start_ = GetMouseOnScreen(position);
      pan_end_ = pan_start_;
    }
  }

  void MouseMove(Eigen::Vector2f position) {
    if (!enabled) { return; }

    if (mode_ == Mode::kRotate && !no_rotate) {
      move_prev_ = move_curr_;
      move_curr_ = GetMouseOnCircle(position);
    } else if (mode_ == Mode::kZoom && !no_zoom) {
      zoom_end_ = GetMouseOnScreen(position);
    } else if (mode_ == Mode::kPan && !no_pan) {
      pan_end_ = GetMouseOnScreen(position);
    }

    Update();
  }

  void MouseUp(Eigen::Vector2f position) {
    if (!enabled) { return; }

    mode_ = Mode::kNone;
  }

  void MouseWheel(Eigen::Vector2f position, double wheel_angle) {
    if (!enabled) { return; }
    if (!no_zoom) { return; }

    // TODO.
  }

  // These are variables the user can tweak live.  It doesn't seem
  // worth it to make them private with setter and getters.
  bool enabled = true;
  bool no_rotate = false;
  bool no_zoom = false;
  bool no_pan = false;
  bool no_roll = false;
  bool static_moving = false;

 private:
  void Update() {
    eye_ = state_.position - state_.target;

    if (!no_rotate) { RotateCamera(); }
    if (!no_zoom) { ZoomCamera(); }
    if (!no_pan) { PanCamera(); }

    state_.position = state_.target + eye_;

    CheckDistances();
    LookAt(state_.target);

    last_position_ = eye_;
  }

  void RotateCamera() {
    const Eigen::Vector2f move_delta = move_curr_ - move_prev_;
    double angle = move_delta.norm();

    if (angle != 0.0) {
      eye_ = state_.position - state_.target;
      const Eigen::Vector3f eye_direction = eye_.normalized();
      const Eigen::Vector3f normalized_up = state_.up.normalized();
      const Eigen::Vector3f object_up_direction =
          normalized_up * (move_curr_.y() - move_prev_.y());
      const Eigen::Vector3f object_sideways_direction =
          normalized_up.cross(eye_direction).normalized() *
          (move_curr_.x() - move_prev_.x());
      const Eigen::Vector3f move_direction =
          object_up_direction + object_sideways_direction;
      Eigen::Vector3f axis = move_direction.cross(eye_).normalized();

      angle *= options_.rotate_speed;
      Eigen::Quaternionf quaternion{Eigen::AngleAxisf(angle, axis)};
      eye_ = quaternion * eye_;
      state_.up = quaternion * state_.up;

      last_axis_ = axis;
      last_angle_ = angle;
    } else if (!static_moving && last_angle_ != 0.0) {
      last_angle_ *= std::sqrt(1.0 - options_.dynamic_damping_factor);
      eye_ = state_.position - state_.target;
      Eigen::Quaternionf quaternion{Eigen::AngleAxisf(last_angle_, last_axis_)};
      eye_ = quaternion * eye_;
      state_.up = quaternion * state_.up;
    }

    move_prev_ = move_curr_;
  }

  void ZoomCamera() {
    const float factor =
        1.0f + (zoom_end_.y() - zoom_start_.y()) * options_.zoom_speed;

    if (factor != 1.0f && factor > 0.0f) {
      eye_ = eye_ * factor;
      if (static_moving) {
        zoom_start_ = zoom_end_;
      } else {
        zoom_start_.y() += (zoom_end_.y() - zoom_start_.y()) *
                           options_.dynamic_damping_factor;
      }
    }
  }

  void PanCamera() {
    Eigen::Vector2f mouse_change = pan_end_ - pan_start_;

    if (mouse_change.squaredNorm() == 0.0f) { return; }

    mouse_change *= eye_.norm() * options_.pan_speed;
    Eigen::Vector3f pan = (
        eye_.cross(state_.up).normalized() * mouse_change.x() +
        state_.up.normalized() * mouse_change.y());

    state_.position += pan;
    state_.target += pan;

    if (static_moving) {
      pan_start_ = pan_end_;
    } else {
      pan_start_ += (pan_end_ - pan_start_) * options_.dynamic_damping_factor;
    }
  }

  void CheckDistances() {
    if (no_zoom && no_pan) { return; }
  }

  void LookAt(const Eigen::Vector3f& target) {
    view_ = LookAt(state_.position, target, state_.up);
  }

  Eigen::Vector2f GetMouseOnScreen(Eigen::Vector2f position) {
    return position;
  }

  Eigen::Vector2f GetMouseOnCircle(Eigen::Vector2f position) {
    return Eigen::Vector2f(
        2.0f * (position.x() - 0.5f),
        2.0f * (position.y() - 0.5f));
  }

  static Eigen::Matrix4f LookAt(const Eigen::Vector3f& eye,
                                const Eigen::Vector3f& center,
                                const Eigen::Vector3f& up) {

    // From glm: matrix_transform.inl
    const Eigen::Vector3f f = (center - eye).normalized();
    const Eigen::Vector3f s = f.cross(up).normalized();
    const Eigen::Vector3f u = s.cross(f);

    Eigen::Matrix4f result = Eigen::Matrix4f::Identity();
    result(0, 0) = s.x();
    result(1, 0) = s.y();
    result(2, 0) = s.z();
    result(0, 1) = u.x();
    result(1, 1) = u.y();
    result(2, 1) = u.z();
    result(0, 2) =-f.x();
    result(1, 2) =-f.y();
    result(2, 2) =-f.z();
    result(3, 0) =-s.dot(eye);
    result(3, 1) =-u.dot(eye);
    result(3, 2) = f.dot(eye);

    return result.transpose();
  }

  Options options_;

  struct State {
    Eigen::Vector3f target;
    Eigen::Vector3f position;
    Eigen::Vector3f up;
    double zoom = 1.0;
  };
  State state_;

  enum class Mode {
    kNone,
    kRotate,
    kZoom,
    kPan,
  };

  Mode mode_ = Mode::kNone;

  Eigen::Vector3f last_position_;

  Eigen::Vector3f eye_;
  Eigen::Vector2f move_prev_;
  Eigen::Vector2f move_curr_;
  Eigen::Vector3f last_axis_;
  double last_angle_ = 0;
  Eigen::Vector2f zoom_start_;
  Eigen::Vector2f zoom_end_;
  Eigen::Vector2f pan_start_;
  Eigen::Vector2f pan_end_;


  Eigen::Matrix4f view_{Eigen::Matrix4f::Identity()};
};

}

}
