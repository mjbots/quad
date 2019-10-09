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

def utfcpp_repository(name):
    github_archive(
        name = name,
        repo = "ledger/utfcpp",
        commit = "a64e2a8380ca1a3097eebd10cf6b15299b499e5c",
        sha256 = "1e3daa51e25bb12a221a3ced9da85760967b0c1f487f7ab70efbef189a9e5c9f",
        build_file = Label("//tools/workspace/utfcpp:package.BUILD"),
    )
