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

load("//tools/workspace/bazel_deps:repository.bzl", "bazel_deps_repository")
load("//tools/workspace/gst-rpicamsrc:repository.bzl", "gst_rpicamsrc_repository")
load("//tools/workspace/i2c-tools:repository.bzl", "i2c_tools_repository")
load("//tools/workspace/moteus:repository.bzl", "moteus_repository")
load("//tools/workspace/raspberrypi-firmware:repository.bzl", "raspberrypi_firmware_repository")
load("//tools/workspace/rpi_bazel:repository.bzl", "rpi_bazel_repository")
load("//tools/workspace/sophus:repository.bzl", "sophus_repository")

def add_default_repositories(excludes = []):
    if "bazel_deps" not in excludes:
        bazel_deps_repository(name = "com_github_mjbots_bazel_deps")
    if "gst-rpicamsrc" not in excludes:
        gst_rpicamsrc_repository(name = "gst-rpicamsrc")
    if "i2c-tools" not in excludes:
        i2c_tools_repository(name = "i2c-tools")
    if "moteus" not in excludes:
        moteus_repository(name = "moteus")
    if "raspberrypi-firmware" not in excludes:
        raspberrypi_firmware_repository(name = "raspberrypi-firmware")
    if "rpi_bazel" not in excludes:
        rpi_bazel_repository(name = "rpi_bazel")
    if "sophus" not in excludes:
        sophus_repository(name = "sophus")
