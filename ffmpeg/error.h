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
#include <libavutil/error.h>
}

#include <fmt/format.h>

#include "mjlib/base/system_error.h"

namespace mjmech {
namespace ffmpeg {

inline int ErrorCheck(int value) {
  if (value < 0) {
    throw mjlib::base::system_error::einval(
        fmt::format("ffmpeg error: {}", av_err2str(value)));
  }
  return value;
}

}
}
