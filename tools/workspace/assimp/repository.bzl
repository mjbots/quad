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

def assimp_repository(name):
    github_archive(
        name = name,
        repo = "assimp/assimp",
        commit = "3a6648734c78f4504f6ab90926f3ecc04d2cc40f",
        sha256 = "e9501051c1ad8f7623d6b8c10268b72b830d8d0cbdec88d8b23c3494f743f65c",
        patches = [Label("//tools/workspace/assimp:assimp.patch")],
        patch_args = ["-p1"],
        build_file = Label("//tools/workspace/assimp:package.BUILD"),
    )
