# -*- python -*-

# Copyright 2020 Josh Pieper, jjp@pobox.com.
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

def implot_repository(name):
    github_archive(
        name = name,
        repo = "epezent/implot",
        commit = "997bf15584b708a2e5df200587691c9d191806e3",
        sha256 = "1fa1d25468a517acd230601d8b057bcabb48a9c2ca016f0b983a21c2e0349d0d",
        build_file = Label("//tools/workspace/implot:package.BUILD"),
    )
