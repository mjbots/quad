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

def raspicam_repository(name):
    github_archive(
        name = name,
        repo = "cedricve/raspicam",
        commit = "651c56418a5a594fc12f1414eb14f2b899729cb1",
        sha256 = "35348ef9556aa3ebe7a5f6571fb586e9c5c1fbd1290b22245ffcee1fb2e1582c",
        build_file = Label("//tools/workspace/raspicam:package.BUILD"),
        patches = [
            Label("//tools/workspace/raspicam:raspicam.diff"),
        ],
        patch_args = ["-p1"],
    )
