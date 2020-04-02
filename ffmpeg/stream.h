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
#include <libavformat/avformat.h>
}

namespace mjmech {
namespace ffmpeg {

class Stream {
 public:
  static Stream MakeInternal(AVStream* stream, AVCodec* codec) {
    return Stream(stream, codec);
  }

  const AVCodecParameters* codec_parameters() const { return stream_->codecpar; }
  const AVStream* av_stream() const { return stream_; }
  const AVCodec* av_codec() const { return codec_; }

 private:
  Stream(AVStream* stream,
         AVCodec* codec)
      : stream_(stream),
        codec_(codec) {}

  AVStream* stream_;
  AVCodec* codec_;
};

}
}
