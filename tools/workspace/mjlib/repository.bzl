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

def mjlib_repository(name):
    github_archive(
        name = name,
        repo = "mjbots/mjlib",
        commit = "5c9d358b5068aa6e10baaff73082cb846e7e64c2",
        sha256 = "8be53b09f0f396530ec16a03a4ec6912d2ee9ac6b8e4316be45e12b8a338037d",
    )
