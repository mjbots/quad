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
        commit = "ab632c82bd501b9fcb6f8200df0551989292b7a1",
        sha256 = "45bf7f021214eb1b9390e3d06e8e0afa3d243f5b739fae776dd22c855c178a09",
    )
