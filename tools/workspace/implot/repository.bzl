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
        commit = "023f96d067653835355329cf00a81e369d2f213a",
        sha256 = "f6866dd949d7f9d8fa396407390dea99f34208645c0b5b08bc2e96d3f157e415",
        build_file = Label("//tools/workspace/implot:package.BUILD"),
    )
