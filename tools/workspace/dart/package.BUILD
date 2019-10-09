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

load("@com_github_mjbots_mech//tools/workspace:generate_file.bzl", "generate_file")

generate_file(
    name = "dart/config.hpp",
    content = """
#ifndef DART_CONFIG_HPP_
#define DART_CONFIG_HPP_

#include <Eigen/Core>

#define DART_MAJOR_VERSION 6
#define DART_MINOR_VERSION 9
#define DART_PATCH_VERSION 2

#define DART_VERSION_AT_LEAST(x,y,z) \
  (DART_MAJOR_VERSION > x || (DART_MAJOR_VERSION >= x && \
  (DART_MINOR_VERSION > y || (DART_MINOR_VERSION >= y && \
  DART_PATCH_VERSION >= z))))

#define DART_MAJOR_MINOR_VERSION_AT_LEAST(x,y) \
  (DART_MAJOR_VERSION > x || (DART_MAJOR_VERSION >= x && \
  (DART_MINOR_VERSION > y || (DART_MINOR_VERSION >= y))))

#define DART_VERSION_AT_MOST(x,y,z) \
  (DART_MAJOR_VERSION < x || (DART_MAJOR_VERSION <= x && \
  (DART_MINOR_VERSION < y || (DART_MINOR_VERSION <= y && \
  DART_PATCH_VERSION <= z))))

#define DART_MAJOR_MINOR_VERSION_AT_MOST(x,y) \
  (DART_MAJOR_VERSION < x || (DART_MAJOR_VERSION <= x && \
  (DART_MINOR_VERSION < y || (DART_MINOR_VERSION <= y))))

#define EIGEN_VERSION_AT_MOST(x,y,z) \
  (EIGEN_WORLD_VERSION < x || (EIGEN_WORLD_VERSION <= x && \
  (EIGEN_MAJOR_VERSION < y || (EIGEN_MAJOR_VERSION <= y && \
  EIGEN_MINOR_VERSION <= z))))

// Detect the compiler
#if defined(__clang__)
  #define DART_COMPILER_CLANG
#elif defined(__GNUC__) || defined(__GNUG__)
  #define DART_COMPILER_GCC
#elif defined(_MSC_VER)
  #define DART_COMPILER_MSVC
#endif

#endif
    """)

cc_library(
    name = "dart",
    hdrs = glob(["dart/**/*.hpp"]),
    srcs = [
        "dart/config.hpp",
        "dart/optimizer/Function.cpp",
        "dart/optimizer/Problem.cpp",
        "dart/optimizer/GradientDescentSolver.cpp",
        "dart/optimizer/Solver.cpp",
    ] + glob([
        "dart/common/**/*.cpp",
        "dart/dynamics/**/*.cpp",
        "dart/math/**/*.cpp",
    ], exclude = [
        # These need ikfast
        "dart/dynamics/IkFast.cpp",
        "dart/dynamics/SharedLibraryIkFast.cpp",
    ]),
    deps = ["@assimp", "@boost", "@eigen"],
    includes = ["."],
)
