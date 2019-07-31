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
        commit = "c7945b5234d1debc8650403d89ec2f8e3747dc51",
        sha256 = "73b1f3a589cdbbf8d944b064b29fc87ddf269109a6bb5f2926c3439033958ae7",
    )
