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
        commit = "9894df49348a8de412fd3b52ff7bfb322dbeede2",
        sha256 = "f5f655ef6d6c167c978373dcd8a53f7188d99ce768a20156fc1e042167a7c296",
        build_file = Label("//tools/workspace/implot:package.BUILD"),
        patches = [Label("//tools/workspace/implot:empty-legend.diff")],
        patch_args = ["-p1"],
    )
