# -*- python -*-

# Copyright 2020 Josh Pieper, jjp@pobox.com.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "ffmpeg",
    hdrs = [
        "codec.h",
        "dictionary.h",
        "error.h",
        "file.h",
        "ffmpeg.h",
        "frame.h",
        "input_format.h",
        "packet.h",
        "ref_base.h",
        "stream.h",
        "swscale.h"
    ],
    deps = [
        "@com_github_mjbots_mjlib//mjlib/base:fail",
        "@com_github_mjbots_mjlib//mjlib/base:system_error",
        "@eigen",
        "@ffmpeg",
        "@fmt",
    ],
)

cc_test(
    name = "ffmpeg_test",
    srcs = ["test/" + x for x in [
        "codec_test.cc",
        "file_test.cc",
        "frame_test.cc",
        "packet_test.cc",
        "stream_test.cc",
        "swscale_test.cc",
        "test_main.cc",
    ]],
    deps = [
        ":ffmpeg",
        "@bazel_tools//tools/cpp/runfiles",
        "@boost//:test",
        "@org_llvm_libcxx//:libcxx",
    ],
    data = [
        "test/data/sample_log.mp4",
    ],
)
