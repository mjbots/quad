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
#include <libavcodec/avcodec.h>
}

#include <optional>

#include "ffmpeg/error.h"
#include "ffmpeg/frame.h"
#include "ffmpeg/packet.h"
#include "ffmpeg/stream.h"

namespace mjmech {
namespace ffmpeg {

class Codec {
 public:
  // TODO(jpieper): Should I enforce this to be constructed after some
  // frames have been read?  Determine if it makes a difference as to
  // what is populated.
  Codec(const Stream& stream) {
    const auto av_codec = stream.av_codec();
    const auto av_stream = stream.av_stream();
    const auto p = stream.codec_parameters();

    context_ = avcodec_alloc_context3(av_codec);

    context_->width = p->width;
    context_->height = p->height;
    context_->framerate = av_stream->avg_frame_rate;
    context_->pix_fmt = static_cast<AVPixelFormat>(p->format);

    context_->color_range = p->color_range;
    context_->color_primaries = p->color_primaries;
    context_->color_trc = p->color_trc;
    context_->colorspace = p->color_space;
    context_->chroma_sample_location = p->chroma_location;

    ErrorCheck(avcodec_open2(context_, av_codec, nullptr));
  }

  ~Codec() {
    avcodec_free_context(&context_);
  }

  void SendPacket(const Packet::Ref& packet) {
    ErrorCheck(avcodec_send_packet(context_, &*packet));
  }

  std::optional<Frame::Ref> GetFrame(Frame* frame) {
    const int ret = avcodec_receive_frame(context_, frame->get());
    if (ret == AVERROR(EAGAIN)) { return {}; }
    ErrorCheck(ret);
    return Frame::Ref::MakeInternal(frame->get());
  }

 private:
  AVCodecContext* context_;
};

}
}
