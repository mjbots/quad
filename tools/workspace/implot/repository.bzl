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
        commit = "cb5937c4a39ab6b1a1a7afd9dbddf1c093a826e8",
        sha256 = "9585816791b6bbd293b6a1dc58ef96c9bdc7e6bf7c818b09533ab9694b83ceea",
        build_file = Label("//tools/workspace/implot:package.BUILD"),
    )
