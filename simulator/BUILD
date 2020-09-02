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

package(default_visibility = ["//visibility:public"])

load("//base:module_main.bzl", "module_main")

cc_binary(
    name = "simulator",
    srcs = [
        "make_robot.cc",
        "simulator_window.cc",
        "simulator.cc",
    ] + glob(["*.h"]),
    deps = [
        "//mech",
        "@com_github_mjbots_mjlib//mjlib/base:limit",
        "@com_github_mjbots_mjlib//mjlib/base:pid",
        "@com_github_mjbots_mjlib//mjlib/micro:pool_ptr",
        "@com_github_mjbots_mjlib//mjlib/multiplex:micro_server",
        "@dart//:gui",
        "@org_llvm_libcxx//:libcxx",
    ],
    linkstatic = False,
)
