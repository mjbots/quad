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

#include "mjlib/base/fail.h"

#include "ffmpeg/dictionary.h"
#include "ffmpeg/error.h"
#include "ffmpeg/ffmpeg.h"
#include "ffmpeg/input_format.h"
#include "ffmpeg/packet.h"
#include "ffmpeg/stream.h"

namespace mjmech {
namespace ffmpeg {

class File {
 public:
  struct Flags {
    bool nonblock = false;
    std::optional<InputFormat> input_format;

    Flags& set_nonblock(bool v) {
      nonblock = v;
      return *this;
    }

    Flags& set_input_format(InputFormat v) {
      input_format = std::move(v);
      return *this;
    }

    Flags() {}
  };

  File(std::string_view url,
       const Dictionary& options = Dictionary(),
       const Flags& flags = Flags()) {
    Ffmpeg::Register();

    context_ = avformat_alloc_context();
    if (flags.nonblock) {
      context_->flags = AVFMT_FLAG_NONBLOCK;
    }

    AVDictionary* av_options = nullptr;
    for (const auto& pair : options) {
      av_dict_set(&av_options, pair.first.c_str(), pair.second.c_str(), 0);
    }

    try {
      ErrorCheck(avformat_open_input(
                     &context_, url.data(),
                     !flags.input_format ? nullptr : flags.input_format->get(),
                     &av_options));
    } catch (mjlib::base::system_error& se) {
      se.code().Append(fmt::format("Opening: '{}'", url));
      throw;
    }
    // Check for leftover options?
  }

  ~File() {
    avformat_free_context(context_);
  }

  File(const File&) = delete;
  File& operator=(const File&) = delete;

  enum StreamType {
    kVideo,
    kAudio,
  };

  Stream FindBestStream(StreamType type) {
    const auto av_type = [&]() {
      switch (type) {
        case kVideo: return AVMEDIA_TYPE_VIDEO;
        case kAudio: return AVMEDIA_TYPE_AUDIO;
      }
      mjlib::base::AssertNotReached();
    }();
    AVCodec* codec = nullptr;
    const int stream_num =
        ErrorCheck(av_find_best_stream(context_, av_type, -1, -1, &codec, 0));
    return Stream::MakeInternal(context_->streams[stream_num], codec);
  }

  std::optional<Packet::Ref> Read(Packet* packet) {
    const int ret = av_read_frame(context_, packet->get());
    if (ret == AVERROR(EAGAIN)) { return {}; }
    ErrorCheck(ret);
    return Packet::Ref::MakeInternal(packet->get());
  }

  struct SeekOptions {
    // non-keyframes are treated as keyframes
    bool any = false;
  };
  void Seek(Stream stream, int64_t min_ts, int64_t ts, int64_t max_ts,
            const SeekOptions& options) {
    const int ret = avformat_seek_file(
        context_, stream.av_stream()->index, min_ts, ts, max_ts,
        options.any ? AVSEEK_FLAG_ANY : 0);
    ErrorCheck(ret);
  }

 private:
  AVFormatContext* context_;
};

}
}
