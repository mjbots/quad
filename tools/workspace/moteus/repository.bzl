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
        commit = "69a0e36d46dd85d3ad30a8b03c039d4c264093cf",
        sha256 = "5bf308ce594924924d14f7751e4342553207c79b04dc5f489968ac8857fd10ad",
    )
