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

extern "C" {
#include <libavutil/frame.h>
}

#include <utility>

#include <Eigen/Core>

#include "ffmpeg/error.h"
#include "ffmpeg/ref_base.h"

namespace mjmech {
namespace ffmpeg {

class Frame {
 public:
  ///  Allocate an empty frame with no backing data.  This is required
  ///  for certain APIs which need an empty object to dump into.
  Frame() {
    frame_ = av_frame_alloc();
  }

  ~Frame() {
    av_frame_free(&frame_);
  }

  Frame(const Frame&) = delete;
  Frame& operator=(const Frame&) = delete;

  class Ref : public RefBase<AVFrame, Ref> {
   public:
    using RefBase<AVFrame, Ref>::RefBase;

    void reset() {
      if (value_) {
        av_frame_unref(value_);
      }
      value_ = nullptr;
    }
  };

  /// Allocate a Frame of given size and alignment.
  ///
  /// @param align == 0 means select based on platform
  Ref Allocate(AVPixelFormat format, Eigen::Vector2i size, int align = 1) {
    frame_->format = format;
    frame_->width = size.x();
    frame_->height = size.y();
    ErrorCheck(av_frame_get_buffer(frame_, align));
    return Ref::MakeInternal(frame_);
  }

  Eigen::Vector2i size() const {
    return {frame_->width, frame_->height};
  }

  AVPixelFormat format() const {
    return static_cast<AVPixelFormat>(frame_->format);
  }

  AVFrame* get() { return frame_; }

 private:
  AVFrame* frame_ = nullptr;
};

}
}
