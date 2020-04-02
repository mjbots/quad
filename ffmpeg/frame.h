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

#include "ffmpeg/ref_base.h"

namespace mjmech {
namespace ffmpeg {

class Frame {
 public:
  Frame() {
    frame_ = av_frame_alloc();
  }

  ~Frame() {
    av_frame_free(&frame_);
  }

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

  AVFrame* get() { return frame_; }

 private:
  AVFrame* frame_ = nullptr;
};

}
}
