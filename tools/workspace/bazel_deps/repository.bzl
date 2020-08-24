# -*- python -*-

# Copyright 2018-2020 Josh Pieper, jjp@pobox.com.
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
        commit = "52800d3708663f08aa6532b3b487ed0273bb894a",
        sha256 = "962d69c13bce8156c62a0d40b9b4d3931526e09a24248bf8e7321dc52df58f10",
    )
