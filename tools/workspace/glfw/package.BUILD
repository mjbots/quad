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

cc_library(
    name = "glfw",
    hdrs = [
        "include/GLFW/glfw3.h",
        "include/GLFW/glfw3native.h",
    ],
    srcs = ["src/" + x for x in [
        "internal.h",
        "mappings.h",
        "context.c",
        "init.c",
        "input.c",
        "monitor.c",
        "vulkan.c",
        "window.c",
        "x11_platform.h",
        "xkb_unicode.h",
        "posix_time.h",
        "posix_thread.h",
        "glx_context.h",
        "egl_context.h",
        "osmesa_context.h",
        "x11_init.c",
        "x11_monitor.c",
        "x11_window.c",
        "xkb_unicode.c",
        "posix_time.c",
        "posix_thread.c",
        "glx_context.c",
        "egl_context.c",
        "osmesa_context.c",
        "linux_joystick.h",
        "linux_joystick.c",
    ]],
    copts = [
        "-D_GLFW_X11",
        "-DHAVE_XKBCOMMON_COMPOSE_H",
    ],
    includes = ["include"],
    deps = ["@libx11"],
)
