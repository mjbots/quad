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
        commit = "2bcae9d7746410063a0e5c5529af472f317106ef",
        sha256 = "b83f222fcffb28cd36693203254da3c7a4d649d5328242c85e0337fad5e1af7c",
    )
