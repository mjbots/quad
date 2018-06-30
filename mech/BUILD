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

package(default_visibility = ["//visibility:public"])

load("//base:module_main.bzl", "module_main")

cc_library(
    name = "mech",
    srcs = [
        "ahrs.cc",
        "camera_driver.cc",
        "gait_driver.cc",
        "gst_helpers.cc",
        "gst_main_loop.cc",
        "herkulex.cc",
        "mcast_video_link.cc",
        "mech_warfare.cc",
        "mjmech_imu_driver.cc",
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
#        "leg_ik_test.cc",
#        "ripple_test.cc",
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
