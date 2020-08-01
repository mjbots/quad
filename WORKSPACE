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

workspace(name = "com_github_mjbots_mech")

BAZEL_VERSION = "3.4.1"
BAZEL_VERSION_SHA = "1a64c807716e10c872f1618852d95f4893d81667fe6e691ef696489103c9b460"

load("//tools/workspace:default.bzl", "add_default_repositories")

add_default_repositories()

load("@rpi_bazel//tools/workspace:default.bzl",
     rpi_bazel_add = "add_default_repositories")
rpi_bazel_add()

load("@com_github_mjbots_bazel_deps//tools/workspace:default.bzl",
     bazel_deps_add = "add_default_repositories")
bazel_deps_add()

load("@com_github_mjbots_mjlib//tools/workspace:default.bzl",
     mjlib_add = "add_default_repositories")
mjlib_add()

load("@moteus//tools/workspace:default.bzl",
     moteus_add = "add_default_repositories")
moteus_add()
