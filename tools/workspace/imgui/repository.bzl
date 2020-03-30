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

def imgui_repository(name):
    github_archive(
        name = name,
        repo = "ocornut/imgui",
        commit = "68c5d030cdb7c5f928bd961ac7081cdac7009b17",
        sha256 = "b9e35143b92548a53657be2a3933bde5bad8365359fb5459efb4aa11aaf52848",
        build_file = Label("//tools/workspace/imgui:package.BUILD"),
    )
