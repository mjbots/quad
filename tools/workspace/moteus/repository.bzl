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

def moteus_repository(name):
    github_archive(
        name = name,
        repo = "mjbots/moteus",
        commit = "5c3a8f46712f7ba42956270ab6eeb11236f45bd1",
        sha256 = "cc9c28a7508b0be166819b6a6ec48f45e29934df3d57ee074aebefc40e2112b0",
    )
