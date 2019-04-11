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

def rpi_bazel_repository(name):
    github_archive(
        name = name,
        repo = "mjbots/rpi_bazel",
        commit = "565c7544e9ef08eb378aec214ccb9f22cae58117",
        sha256 = "f07db9d89a1e9e5bc8f7207d9b2876f31661d42db4060741f12ad7aaaf5c2113",
    )
