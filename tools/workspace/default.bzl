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

load("//tools/workspace/bazel_deps:repository.bzl", "bazel_deps_repository")
load("//tools/workspace/gl3w:repository.bzl", "gl3w_repository")
load("//tools/workspace/glfw:repository.bzl", "glfw_repository")
load("//tools/workspace/gst-rpicamsrc:repository.bzl", "gst_rpicamsrc_repository")
load("//tools/workspace/i2c-tools:repository.bzl", "i2c_tools_repository")
load("//tools/workspace/imgui:repository.bzl", "imgui_repository")
load("//tools/workspace/implot:repository.bzl", "implot_repository")
load("//tools/workspace/mjlib:repository.bzl", "mjlib_repository")
load("//tools/workspace/moteus:repository.bzl", "moteus_repository")
load("//tools/workspace/pi3hat:repository.bzl", "pi3hat_repository")
load("//tools/workspace/raspicam:repository.bzl", "raspicam_repository")
load("//tools/workspace/raspberrypi-firmware:repository.bzl", "raspberrypi_firmware_repository")
load("//tools/workspace/rpi_bazel:repository.bzl", "rpi_bazel_repository")
load("//tools/workspace/rules_pkg:repository.bzl", "rules_pkg_repository")
load("//tools/workspace/sophus:repository.bzl", "sophus_repository")

def add_default_repositories(excludes = []):
    if not native.existing_rule("com_github_mjbots_bazel_deps"):
        bazel_deps_repository(name = "com_github_mjbots_bazel_deps")
    if not native.existing_rule("gl3w"):
        gl3w_repository(name = "gl3w")
    if not native.existing_rule("glfw"):
        glfw_repository(name = "glfw")
    if not native.existing_rule("gst-rpicamsrc"):
        gst_rpicamsrc_repository(name = "gst-rpicamsrc")
    if not native.existing_rule("i2c-tools"):
        i2c_tools_repository(name = "i2c-tools")
    if not native.existing_rule("imgui"):
        imgui_repository(name = "imgui")
    if not native.existing_rule("implot"):
        implot_repository(name = "implot")
    if not native.existing_rule("com_github_mjbots_mjlib"):
        mjlib_repository(name = "com_github_mjbots_mjlib")
    if not native.existing_rule("moteus"):
        moteus_repository(name = "moteus")
    if not native.existing_rule("pi3hat"):
        pi3hat_repository(name = "pi3hat")
    if not native.existing_rule("raspicam"):
        raspicam_repository(name = "raspicam")
    if not native.existing_rule("raspberrypi-firmware"):
        raspberrypi_firmware_repository(name = "raspberrypi-firmware")
    if not native.existing_rule("rpi_bazel"):
        rpi_bazel_repository(name = "rpi_bazel")
    if not native.existing_rule("rules_pkg"):
        rules_pkg_repository(name = "rules_pkg")
    if not native.existing_rule("sophus"):
        sophus_repository(name = "sophus")
