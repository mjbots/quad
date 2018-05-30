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


def rpi_bazel_repository(name):
    http_archive(
        name = name,
        url = "https://github.com/mjbots/rpi_bazel/archive/a1331f856dac5ff2098e5bee0cd89d6fa52dd816.zip",
        sha256 = "efb40d5c854d23033ab066412aac2ebea8d1c8bf0bd7ebcdba959cd4e7ad196f",
        strip_prefix = "rpi_bazel-a1331f856dac5ff2098e5bee0cd89d6fa52dd816",
    )
