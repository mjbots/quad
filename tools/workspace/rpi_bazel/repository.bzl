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
    commit = "d81f4be4cfa8a95f459e8af923194fcc74b6a16b"
    http_archive(
        name = name,
        url = "https://github.com/mjbots/rpi_bazel/archive/{}.zip".format(commit),
        sha256 = "12eedf68b4069cffcb4e1ff540c1140b36f8cb51189ece8d7e727b8deb0e1220",
        strip_prefix = "rpi_bazel-{}".format(commit),
    )
