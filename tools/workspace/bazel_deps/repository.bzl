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

def bazel_deps_repository(name):
    github_archive(
        name = name,
        repo = "mjbots/bazel_deps",
        commit = "f9e4680ed31e8e14670c66aa087eefa0e27ff815",
        sha256 = "57d26a316fc75448772b216cfd09af41c8fba333351f8fabf88d19a9165312cb",
    )
