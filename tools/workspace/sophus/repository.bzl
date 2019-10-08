# -*- python -*-

# Copyright 2019 Josh Pieper, jjp@pobox.com.
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

def sophus_repository(name):
    github_archive(
        name = name,
        repo = "strasdat/Sophus",
        commit = "ef9551ff429899b5adae66eabd5a23f165953199",
        sha256 = "b5a260f5db7ace1718e9bd44c21fb1a8588e1fb05ae0da29e04bb0eca1906143",
        build_file = Label("//tools/workspace/sophus:package.BUILD"),
    )
