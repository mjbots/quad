# -*- python -*-

# Copyright 2019 Josh Pieper, jjp@pobox.com.
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
    name = "raspicam",
    hdrs = ['src/' + x for x in [
        'raspicam.h',
        'raspicam_cv.h',
        'raspicam_still.h',
        'raspicam_still_cv.h',
        'raspicamtypes.h',
        'scaler.h',
    ]],
    srcs = ['src/' + x for x in [
        'raspicam.cpp',
        'raspicam_cv.cpp',
        'raspicam_still.cpp',
        'raspicam_still_cv.cpp',
        'private/exceptions.h',
        'private/private_impl.cpp',
        'private/private_impl.h',
        'private/private_types.h',
        'private/threadcondition.h',
        'private/threadcondition.cpp',
        'private_still/private_still_impl.cpp',
        'private_still/private_still_impl.h',
    ]] + glob(['dependencies/**/*.h']),
    deps = [
        "@opencv//:videoio",
        "@raspberrypi-firmware",
    ],
    includes = ["src", "dependencies"],
    copts = [
        "-Wno-dynamic-exception-spec",
        "-Wno-unused-private-field",
    ],
)
