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
        commit = "bac41e86410fce7e7fba99f651b551a17e23f9c5",
        sha256 = "2929a6ad634b7aa779873a1411144b2de4e029c397d6b4c6106e927872322833",
    )
