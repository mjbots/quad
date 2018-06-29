# -*- python -*-

# Copyright 2018 Josh Pieper, jjp@pobox.com.
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

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")


def gst_rpicamsrc_repository(name):
    commit = "a181cd8d4b284d09b5f0e23d9ddb4f0a94e1dc8c"
    http_archive(
        name = name,
        url = "https://github.com/thaytan/gst-rpicamsrc/archive/{}.zip".format(commit),
        sha256 = "8ebbd6736b8f396e9cef362d4e0a927f15fc97a133cadf7f8acb6cf06ad310a7",
        strip_prefix = "gst-rpicamsrc-{}".format(commit),
        build_file = Label("//tools/workspace/gst-rpicamsrc:package.BUILD"),
    )
