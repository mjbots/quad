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
        commit = "34a25d4d855abe1b093fc37ed75472b6c2cbc76e",
        sha256 = "3edfc3c972cc0fbd6be63e13902b04056e5d5286c8458189823d1175a7a97c9f",
    )
