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
        "web_control_assets/mjbots.png",
        "web_control_assets/index.html",
        "web_control_assets/styles.css",
        "web_control_assets/js/app.js",
    ],
)

cc_library(
    name = "mech",
    srcs = [
        "camera_driver.cc",
        "mammal_ik.cc",
        "mime_type.cc",
        "nrfusb_client.cc",
        "pi3hat_wrapper.cc",
        "quadruped.cc",
        "quadruped_control.cc",
        "quadruped_trot.cc",
        "rf_control.cc",
        "system_info.cc",
        "swing_trajectory.cc",
        "trajectory.cc",
        "trajectory_line_intersect.cc",
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
        "@opencv//:videoio",
        "@sophus",
    ] + select({
        "//conditions:default" : [],
        "//:raspberrypi" : [
            "@raspicam",
            "@pi3hat//lib/cpp/mjbots/pi3hat:libpi3hat",
        ],
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
        "expo_map_test.cc",
        "mammal_ik_test.cc",
        "swing_trajectory_test.cc",
        "trajectory_line_intersect_test.cc",
        "trajectory_test.cc",
        "test_main.cc",
        "vertical_line_frame_test.cc",
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

cc_binary(
    name = "multiplex_tool",
    srcs = ["multiplex_tool_main.cc"],
    deps = [
        ":mech",
        "@com_github_mjbots_mjlib//mjlib/multiplex:libmultiplex_tool",
        "@org_llvm_libcxx//:libcxx",
    ],
)

pkg_tar(
    name = "quad_deploy",
    extension = "tar",
    include_runfiles = True,
    package_dir = "mech",
    srcs = [
        ":multiplex_tool",
        ":quadruped",
    ],
    files = {
        "//configs:quadruped.ini": "configs/quadruped.ini",
        "//configs:quada0.cfg": "configs/quada0.cfg",
        "//configs:quada1.cfg": "configs/quada1.cfg",
        "//utils:config_servos.py": "config_servos.py",
        "//utils:performance_governor.sh": "performance_governor.sh",
        "//utils:quad.cmd" : "quad.cmd",
        "//utils:quad_screen.conf" : "quad_screen.conf",
        "//utils:quad-start.sh" : "quad-start.sh",
        "//utils:start-robot.sh": "start-robot.sh",
        "//utils:zero_leg.py": "zero_leg.py",
        "web_control_assets/mjbots.png" : "web_control_assets/mjbots.png",
        "web_control_assets/index.html" : "web_control_assets/index.html",
        "web_control_assets/styles.css" : "web_control_assets/styles.css",
        "web_control_assets/js/app.js" : "web_control_assets/js/app.js",
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
