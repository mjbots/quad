# -*- python -*-

# Copyright 2018-2020 Josh Pieper, jjp@pobox.com.
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

def pi3hat_repository(name):
    github_archive(
        name = name,
        repo = "mjbots/pi3hat",
        commit = "4e1711502bfccda19a23c0469eaee18437b30ab7",
        sha256 = "a077f8e17bed3a44cb5d2e7c06ca86f7a39049d704ef91571f2472a78cd3fd8a",
    )
