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


def bazel_deps_repository(name):
    commit = "5532da15eb54efe61b41e6d22ffa5178d9e6b90a"
    http_archive(
        name = name,
        url = "https://github.com/mjbots/bazel_deps/archive/{}.zip".format(commit),
        sha256 = "7fd83e6418f6420e1f266a5563a3a89ea257578fbaa8ef9706bd4fae58a7d1c8",
        strip_prefix = "bazel_deps-{}".format(commit),
    )
