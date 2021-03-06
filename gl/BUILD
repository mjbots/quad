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
    name = "gl",
    hdrs = [
        "camera.h",
        "flat_rgb_texture.h",
        "framebuffer.h",
        "gl_imgui.h",
        "image_texture.h",
        "orthographic_camera.h",
        "perspective_camera.h",
        "program.h",
        "renderbuffer.h",
        "shader.h",
        "simple_line_render_list.h",
        "simple_texture_render_list.h",
        "texture.h",
        "trackball.h",
        "vertex_array_object.h",
        "vertex_buffer_object.h",
    ],
    srcs = [
        "simple_line_render_list.cc",
        "simple_texture_render_list.cc",
    ],
    deps = [
        "//base",
        "@com_github_mjbots_mjlib//mjlib/imgui:gl",
        "@eigen",
        "@fmt",
        "@gl3w",
        "@glfw",
        "@imgui",
        "@opencv//:imgcodecs",
    ],
    defines = [
        "MJMECH_ENABLE_TRACE_GL_ERROR",
    ],
)

cc_test(
    name = "test",
    srcs = ["test/" + x for x in [
        "flat_rgb_texture_test.cc",
        "framebuffer_test.cc",
        "perspective_camera_test.cc",
        "program_test.cc",
        "shader_test.cc",
        "test_main.cc",
        "texture_test.cc",
        "vertex_array_object_test.cc",
        "vertex_buffer_object_test.cc",
    ]],
    deps = [
        ":gl",
        "@boost//:test",
    ],
)
