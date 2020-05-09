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
        commit = "538d28e29ed177f1bde0ad0f1c6293f001298647",
        sha256 = "1189d1ad2fad351f06866da8b2aa94a12d16b95493ea2d84bf7584290cc0f99a",
        build_file = Label("//tools/workspace/imgui:package.BUILD"),
    )
