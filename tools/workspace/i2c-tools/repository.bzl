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


def i2c_tools_repository(name):
    http_archive(
        name = name,
        urls = [
            "https://mirrors.edge.kernel.org/pub/software/utils/i2c-tools/i2c-tools-4.0.tar.xz",
        ],
        sha256 = "d900ca1c11c51ea20caa50b096f948008b8a7ad832311b23353e21baa7af28d6",
        strip_prefix = "i2c-tools-4.0",
        build_file = Label("//tools/workspace/i2c-tools:package.BUILD"),
    )
