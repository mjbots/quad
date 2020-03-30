# -*- python -*-

# Copyright 2020 Josh Pieper, jjp@pobox.com.
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

def glfw_repository(name):
    github_archive(
        name = name,
        repo = "glfw/glfw",
        commit = "44b5d06583cd21ac237eb8f6263db03faf1726c2",
        sha256 = "d22c409c39513a99b2c264974a436953820919fa9ba29204ad61d8f92465833a",
        build_file = Label("//tools/workspace/glfw:package.BUILD"),
    )
