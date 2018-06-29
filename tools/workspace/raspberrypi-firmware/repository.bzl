# -*- python -*-

# Copyright 2018 Josh Pieper, jjp@pobox.com.
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

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")


def raspberrypi_firmware_repository(name):
    http_archive(
        name = name,
        url = "https://github.com/raspberrypi/firmware/archive/1.20180417.tar.gz",
        sha256 = "aa3b7dfc9760c4be47f23a6210b6b989cf95b92e695dd4682facccd6c712c3a5",
        strip_prefix = "firmware-1.20180417",
        build_file = Label("//tools/workspace/raspberrypi-firmware:package.BUILD"),
    )
