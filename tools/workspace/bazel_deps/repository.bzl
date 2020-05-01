# -*- python -*-

# Copyright 2018-2019 Josh Pieper, jjp@pobox.com.
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

load("//tools/workspace:github_archive.bzl", "github_archive")

def bazel_deps_repository(name):
    github_archive(
        name = name,
        repo = "mjbots/bazel_deps",
        commit = "9feb52c0d5c124c65af9c0b5fbf25e6fc7cd3205",
        sha256 = "5940e750e9b0c7065d213c55274734d953f54db148250481249c98bceb2d39d0",
    )
