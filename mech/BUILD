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

package(default_visibility = ["//visibility:public"])

load("//base:module_main.bzl", "module_main")
load("@bazel_tools//tools/build_defs/pkg:pkg.bzl", "pkg_tar")

cc_library(
    name = "mech",
    srcs = [
        "ahrs.cc",
        "camera_driver.cc",
        "gait_driver.cc",
        "gst_helpers.cc",
        "gst_main_loop.cc",
        "herkulex.cc",
        "jump_test.cc",
        "mcast_video_link.cc",
        "mech_warfare.cc",
        "mech_warfare_command.cc",
        "mjmech_imu_driver.cc",
        "moteus_servo.cc",
        "multiplex_client.cc",
        "rtsp_server.cc",
        "servo_monitor.cc",
        "target_tracker.cc",
        "turret.cc",
        "video_display.cc",
    ],
    hdrs = glob(["*.h"]),
    deps = [
        "//base",
        "@gstreamer",
        "@gst-plugins-base//:gstapp",
        "@gst-plugins-base//:plugins",
        "@gst-plugins-bad//:plugins",
        "@gst-plugins-ugly//:plugins",
        "@gst-libav//:plugins",
        "@gst-rtsp-server//:rtsp-server",
        "@moteus//mjlib/multiplex:asio_client",
        "@opencv//:core",
        "@opencv//:aruco",
        "@opencv//:imgcodecs",
    ] + select({
        "//conditions:default" : [],
        "//:raspberrypi" : ["@gst-rpicamsrc"],
    }),
    copts = [
        "-Wno-gnu-designator",
    ] + select({
        "//conditions:default" : [],
        "//:raspberrypi" : ["-DCOM_GITHUB_MJBOTS_RASPBERRYPI"],
    }),
)

cc_test(
    name = "mech_test",
    srcs = ["test/" + x for x in [
        "herkulex_test.cc",
        "leg_ik_test.cc",
        "ripple_test.cc",
        "test_main.cc",
    ]],
    deps = [
        ":mech",
        "@boost//:test",
    ],
)

cc_binary(
    name = "herkulex_tool",
    srcs = ["herkulex_tool.cc"],
    deps = [":mech"],
)

cc_binary(
    name = "servo_latency_test",
    srcs = ["servo_latency_test.cc"],
    deps = [":mech"],
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
    name = "mjmech_imu_driver",
    cname = "mjmech::mech::MjmechImuDriver",
    prefix = "mech",
    deps = [":mech"],
)

module_main(
    name = "video_sender_app",
    cname = "mjmech::mech::VideoSenderApp",
    prefix = "mech",
    deps = [":mech"],
)

module_main(
    name = "video_controller_app",
    cname = "mjmech::mech::VideoControllerApp",
    prefix = "mech",
    deps = [":mech"],
)

module_main(
    name = "mech_warfare",
    cname = "mjmech::mech::MechWarfare",
    prefix = "mech",
    deps = [":mech"],
)

module_main(
    name = "mw_command",
    cname = "mjmech::mech::MWCommand",
    prefix = "mech",
    deps = [":mech"],
)

module_main(
    name = "jump_test",
    cname = "mjmech::mech::JumpTest",
    prefix = "mech",
    deps = [":mech"],
)

cc_binary(
    name = "multiplex_tool",
    srcs = ["multiplex_tool_main.cc"],
    deps = [
        "@moteus//mjlib/multiplex:libmultiplex_tool",
        "@org_llvm_libcxx//:libcxx",
    ],
)

pkg_tar(
    name = "deploy",
    extension = "tar",
    include_runfiles = True,
    package_dir = "mech",
    srcs = [
        ":video_sender_app",
        ":mech_warfare",
        ":jump_test",
        ":herkulex_tool",
        ":multiplex_tool",
        ":servo_latency_test",
    ],
    files = {
        "//configs:real.cfg": "configs/real.cfg",
        "//configs:mw.ini": "configs/mw.ini",
        "//configs:mw_lizard.ini": "configs/mw_lizard.ini",
        "//configs:lizard.cfg": "configs/lizard.cfg",
        "//configs:moteus.ini": "configs/moteus.ini",
        "//configs:moteus_ik.cfg": "configs/moteus_ik.cfg",
        "//configs:jump_test.ini": "configs/jump_test.ini",
        "//utils:start-robot.sh": "start-robot.sh",
        "//utils:performance_governor.sh": "performance_governor.sh",
        "//utils:rezero.sh": "rezero.sh",
        "//utils:update_pid.sh": "update_pid.sh",
    },
)
