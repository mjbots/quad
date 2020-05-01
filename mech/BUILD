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

load("//base:module_main.bzl", "module_main")
load("@rules_pkg//:pkg.bzl", "pkg_tar")

filegroup(
    name = "web_control_assets",
    srcs = [
        "web_control_assets/index.html",
        "web_control_assets/styles.css",
        "web_control_assets/js/app.js",
        "turret_assets/index.html",
        "turret_assets/styles.css",
        "turret_assets/js/app.js",
    ],
)

cc_library(
    name = "mech",
    srcs = [
        "mammal_ik.cc",
        "mime_type.cc",
        "moteus_servo.cc",
        "multiplex_client.cc",
        "nrfusb_client.cc",
        "quadruped.cc",
        "quadruped_control.cc",
        "quadruped_debug.cc",
        "rf_control.cc",
        "rpi3_hat_aux_stm32.cc",
        "rpi3_hat_raw_spi.cc",
        "rpi3_raw_aux_spi.cc",
        "rpi3_raw_spi.cc",
        "rpi3_raw_uart.cc",
        "rpi3_threaded_client.cc",
        "turret.cc",
        "turret_control.cc",
        "turret_rf_control.cc",
        "trajectory.cc",
        "web_server.cc",
    ],
    hdrs = glob(["*.h"]),
    deps = [
        "//base",
        "@boost//:filesystem",
        "@dart",
        "@com_github_mjbots_mjlib//mjlib/base:pid",
        "@com_github_mjbots_mjlib//mjlib/io:exclusive_command",
        "@com_github_mjbots_mjlib//mjlib/io:selector",
        "@com_github_mjbots_mjlib//mjlib/multiplex:asio_client",
        "@com_github_mjbots_mjlib//mjlib/multiplex:frame",
        "@com_github_mjbots_mjlib//mjlib/multiplex:register",
        "@opencv//:core",
        "@opencv//:aruco",
        "@opencv//:imgcodecs",
        "@sophus",
    ] + select({
        "//conditions:default" : [],
        "//:raspberrypi" : [],
    }),
    copts = [
        "-Wno-gnu-designator",
    ] + select({
        "//conditions:default" : [],
        "//:raspberrypi" : ["-DCOM_GITHUB_MJBOTS_RASPBERRYPI"],
    }),
    data = [
        ":web_control_assets",
    ],
    features = [
        "dbg",
    ],
)

cc_test(
    name = "test",
    srcs = ["test/" + x for x in [
        "mammal_ik_test.cc",
        "trajectory_test.cc",
        "test_main.cc",
    ]],
    deps = [
        ":mech",
        "@boost//:test",
    ],
)

cc_binary(
    name = "direct_servo_latency_test",
    srcs = ["direct_servo_latency_test.cc"],
    deps = [":mech"],
)

cc_binary(
    name = "rpi_serial_test",
    srcs = ["rpi_serial_test.cc"],
    deps = [":mech"],
)

cc_binary(
    name = "qdd100_test",
    srcs = ["qdd100_test.cc"],
    deps = [
        ":mech",
        "//base",
        "@com_github_mjbots_mjlib//mjlib/base:clipp_archive",
        "@com_github_mjbots_mjlib//mjlib/base:clipp",
        "@com_github_mjbots_mjlib//mjlib/multiplex:stream_asio_client_builder",
        "@org_llvm_libcxx//:libcxx",
    ],
)

cc_binary(
    name = "aruco_test",
    srcs = ["aruco_test.cc"],
    deps = ["//base", "@boost", "@boost//:timer", "@boost//:system", "@fmt",
            "@opencv//:aruco", "@opencv//:imgcodecs"],
)

cc_binary(
    name = "aruco_draw",
    srcs = ["aruco_draw.cc"],
    deps = ["//base", "@boost", "@boost//:timer",
            "@opencv//:aruco", "@opencv//:imgcodecs"],
)

module_main(
    name = "quadruped",
    cname = "mjmech::mech::Quadruped",
    prefix = "mech",
    deps = [":mech"],
)

module_main(
    name = "turret",
    cname = "mjmech::mech::Turret",
    prefix = "mech",
    deps = [":mech"],
)

cc_binary(
    name = "multiplex_tool",
    srcs = ["multiplex_tool_main.cc"],
    deps = [
        ":mech",
        "@com_github_mjbots_mjlib//mjlib/multiplex:libmultiplex_tool",
        "@org_llvm_libcxx//:libcxx",
    ],
)

cc_binary(
    name = "moteus_tool",
    srcs = ["moteus_tool_main.cc"],
    deps = [
        ":mech",
        "@moteus//moteus/tool:moteus_tool_lib",
        "@org_llvm_libcxx//:libcxx",
    ],
)

cc_binary(
    name = "spi",
    srcs = ["spi.cc"],
    deps = [
        ":mech",
        "@fmt",
        "@com_github_mjbots_mjlib//mjlib/base:string_span",
        "@com_github_mjbots_mjlib//mjlib/base:system_error",
        "@org_llvm_libcxx//:libcxx",
    ],
)

cc_binary(
    name = "bmi088_test",
    srcs = ["bmi088_test.cc"],
    deps = [
        ":mech",
        "@fmt",
        "@com_github_mjbots_mjlib//mjlib/base:string_span",
        "@com_github_mjbots_mjlib//mjlib/base:system_error",
        "@org_llvm_libcxx//:libcxx",
    ],
)

cc_binary(
    name = "slot_rf_test",
    srcs = ["slot_rf_test.cc"],
    deps = [
        ":mech",
        "@fmt",
        "@com_github_mjbots_mjlib//mjlib/base:string_span",
        "@com_github_mjbots_mjlib//mjlib/base:system_error",
        "@org_llvm_libcxx//:libcxx",
    ],
)

pkg_tar(
    name = "deploy",
    extension = "tar",
    include_runfiles = True,
    package_dir = "mech",
    srcs = [
        ":bmi088_test",
        ":moteus_tool",
        ":multiplex_tool",
        ":quadruped",
        ":slot_rf_test",
        ":spi",
        ":turret",
    ],
    files = {
        "//configs:quadruped.ini": "configs/quadruped.ini",
        "//configs:quada0.cfg": "configs/quada0.cfg",
        "//configs:quada1.cfg": "configs/quada1.cfg",
        "//configs:turret.ini": "configs/turret.ini",
        "//utils:start-robot.sh": "start-robot.sh",
        "//utils:performance_governor.sh": "performance_governor.sh",
        "//utils:rezero.sh": "rezero.sh",
        "//utils:update_limits.sh": "update_limits.sh",
        "//utils:update_pid.sh": "update_pid.sh",
        "//utils:update_voltage.sh": "update_voltage.sh",
        "web_control_assets/index.html" : "web_control_assets/index.html",
        "web_control_assets/styles.css" : "web_control_assets/styles.css",
        "web_control_assets/js/app.js" : "web_control_assets/js/app.js",
        "turret_assets/index.html" : "turret_assets/index.html",
        "turret_assets/styles.css" : "turret_assets/styles.css",
        "turret_assets/js/app.js" : "turret_assets/js/app.js",
    },
)

cc_binary(
    name = "imgui_test",
    srcs = ["imgui_test.cc"],
    deps = [
        "@boost",
        "@eigen",
        "@ffmpeg",
        "@fmt",
        "@imgui",
        "@org_llvm_libcxx//:libcxx",
    ],
    copts = [
        "-Wno-deprecated-declarations",
    ],
)

cc_binary(
    name = "rf_command",
    srcs = ["rf_command.cc"],
    deps = [
        ":mech",
        "//ffmpeg",
        "//gl",
        "@com_github_mjbots_mjlib//mjlib/base:clipp",
        "@org_llvm_libcxx//:libcxx",
    ],
)
