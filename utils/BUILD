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

package(default_visibility = ["//visibility:public"])

genrule(
    name = "tplot_uic",
    srcs = [
        "tplot_main_window.ui",
    ],
    outs = [
        "ui_tplot_main_window.py",
    ],
    output_to_bindir = 1,
    cmd = "pyside-uic $< > $@",
)

py_binary(
    name = "tplot",
    srcs = ["tplot.py"],
    data = [":tplot_uic"],
    deps = [
        "@com_github_mjbots_mjlib//mjlib/telemetry:py_file_reader",
    ],
)

py_binary(
    name = "video_aligner",
    srcs = ["video_aligner.py"],
    deps = [
        "@com_github_mjbots_mjlib//mjlib/telemetry:py_file_reader",
    ],
)

cc_binary(
    name = "tplot2",
    srcs = [
        "imgui_tree_archive.h",
        "quadruped_tplot2.h",
        "quadruped_tplot2.cc",
        "tplot2.cc",
        "tree_view.h",
    ],
    deps = [
        "//ffmpeg",
        "//gl",
        "//mech",
        "@com_github_mjbots_mjlib//mjlib/base:buffer_stream",
        "@com_github_mjbots_mjlib//mjlib/base:clipp",
        "@com_github_mjbots_mjlib//mjlib/base:tokenizer",
        "@com_github_mjbots_mjlib//mjlib/telemetry:file_reader",
        "@com_github_mjbots_mjlib//mjlib/telemetry:mapped_binary_reader",
        "@implot",
        "@org_llvm_libcxx//:libcxx",
    ],
)

exports_files([
    "rezero.sh",
    "start-robot.sh",
    "performance_governor.sh",
    "update_limits.sh",
    "update_pid.sh",
    "update_voltage.sh",
    "turret-start.sh",
    "turret_screen.conf",
    "turret.cmd",
])
