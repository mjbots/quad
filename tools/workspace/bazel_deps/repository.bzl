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
    commit = "ec53ce9d36df6e51f559605f6428a58cad159809"
    http_archive(
        name = name,
        url = "https://github.com/mjbots/bazel_deps/archive/{}.zip".format(commit),
        sha256 = "d6c5cdc7790581e1cdcaeaa62082af25cafaa1c9da06fcf2fb745527d1662f67",
        strip_prefix = "bazel_deps-{}".format(commit),
    )
