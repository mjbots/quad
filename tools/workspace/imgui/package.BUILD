# -*- python -*-

# Copyright 2019 Josh Pieper, jjp@pobox.com.
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
    name = "imgui",
    hdrs = [
        "imgui.h",
    ],
    srcs = [
        "imgui.cpp",
        "imgui_demo.cpp",
        "imgui_draw.cpp",
        "imgui_internal.h",
        "imgui_widgets.cpp",
        "imstb_rectpack.h",
        "imstb_textedit.h",
        "imstb_truetype.h",
        "examples/imgui_impl_glfw.h",
        "examples/imgui_impl_glfw.cpp",
        "examples/imgui_impl_opengl3.h",
        "examples/imgui_impl_opengl3.cpp",
    ],
    includes = ["."],
    deps = ["@glfw", "@gl3w"],
    defines = [
        "IMGUI_DISABLE_INCLUDE_IMCONFIG_H",
        'ImDrawIdx="unsigned int"',
    ],
)
