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

def mjlib_repository(name):
    github_archive(
        name = name,
        repo = "mjbots/mjlib",
        commit = "c5f066ac443135cb374e4fd072adbb5de9e7e0b9",
        sha256 = "bedd367e111253e81f023b0e20d51ae0390c2dbe04a9a35688f16404af000093",
    )
