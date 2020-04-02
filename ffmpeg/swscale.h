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
#include "libswscale/swscale.h"
}

#include <Eigen/Core>

#include "mjlib/base/fail.h"

#include "ffmpeg/frame.h"

namespace mjmech {
namespace ffmpeg {

class Swscale {
 public:
  enum Algorithm {
    kFastBilinear,
    kBilinear,
    kBicubic,
  };
  Swscale(Eigen::Vector2i src_size, enum AVPixelFormat src_format,
          Eigen::Vector2i dst_size, enum AVPixelFormat dst_format,
          Algorithm algorithm)
      : src_size_(src_size) {
    const int av_flags = [&]() {
      switch (algorithm) {
        case kFastBilinear: return SWS_FAST_BILINEAR;
        case kBilinear: return SWS_BILINEAR;
        case kBicubic: return SWS_BICUBIC;
      }
      mjlib::base::AssertNotReached();
    }();
    context_ = sws_getContext(
        src_size.x(), src_size.y(), src_format,
        dst_size.x(), dst_size.y(), dst_format,
        av_flags,
        nullptr, nullptr, nullptr);
  }

  ~Swscale() {
    sws_freeContext(context_);
  }

  void Scale(const Frame::Ref& src, const Frame::Ref& dst) {
    const auto av_src_frame = &*src;
    const auto av_dst_frame = &*dst;

    sws_scale(context_, av_src_frame->data, av_src_frame->linesize,
              0, src_size_.y(),
              av_dst_frame->data, av_dst_frame->linesize);
  }

 private:
  const Eigen::Vector2i src_size_;
  SwsContext* context_ = nullptr;
};

}
}
