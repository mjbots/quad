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

load("@com_github_mjbots_mech//tools/workspace:template_file.bzl",
     "template_file")
load("@com_github_mjbots_mech//tools/workspace:generate_file.bzl",
     "generate_file")

template_file(
    name = "include/assimp/config.h",
    src = "include/assimp/config.h.in",
    substitutions = {
        "#cmakedefine": "#define",
    },
)

generate_file(
    name = "revision.h",
    content = """
#ifndef ASSIMP_REVISION_H_INC
#define ASSIMP_REVISION_H_INC

#define GitVersion 0xdeadbeef
#define GitBranch "master"

#define VER_MAJOR 5
#define VER_MINOR 0
#define VER_PATCH 0
#define VER_BUILD 0

#define STR_HELP(x) #x
#define STR(x) STR_HELP(x)

#define VER_FILEVERSION             VER_MAJOR,VER_MINOR,VER_PATCH,VER_BUILD
#if (GitVersion == 0)
#define VER_FILEVERSION_STR         STR(VER_MAJOR) "." STR(VER_MINOR) "." STR(VER_PATCH) "." STR(VER_BUILD)
#else
#define VER_FILEVERSION_STR         STR(VER_MAJOR) "." STR(VER_MINOR) "." STR(VER_PATCH) "." STR(VER_BUILD) " (Commit 3a664873)"
#endif

#ifdef  NDEBUG
#define VER_ORIGINAL_FILENAME_STR   "assimp.dll"
#else
#define VER_ORIGINAL_FILENAME_STR   "assimp.dll"
#endif //  NDEBUG

#endif // ASSIMP_REVISION_H_INC
    """)

cc_library(
    name = "assimp",
    hdrs = [
        "include/assimp/config.h",
    ] + glob([
        "include/**/*.hpp",
        "include/**/*.h",
        "include/**/*.inl",
    ]),
    srcs = [
        "revision.h",
    ] + glob([
        "contrib/unzip/*.h",
        "contrib/unzip/*.c",
        "code/CApi/*.cpp",
        "code/CApi/*.h",
        "code/Common/*.cpp",
        "code/Common/*.h",
        "code/PostProcessing/*.cpp",
        "code/PostProcessing/*.h",
        "code/Material/*.cpp",
        "code/Material/*.h",
        "contrib/utf8cpp/**/*.h",
    ]),
    includes = ["include", "code", "contrib/unzip"],
    copts = [
        "-DASSIMP_BUILD_NO_X_IMPORTER",
        "-DASSIMP_BUILD_NO_AMF_IMPORTER",
        "-DASSIMP_BUILD_NO_3DS_IMPORTER",
        "-DASSIMP_BUILD_NO_MD3_IMPORTER",
        "-DASSIMP_BUILD_NO_MDL_IMPORTER",
        "-DASSIMP_BUILD_NO_MD2_IMPORTER",
        "-DASSIMP_BUILD_NO_PLY_IMPORTER",
        "-DASSIMP_BUILD_NO_ASE_IMPORTER",
        "-DASSIMP_BUILD_NO_OBJ_IMPORTER",
        "-DASSIMP_BUILD_NO_HMP_IMPORTER",
        "-DASSIMP_BUILD_NO_SMD_IMPORTER",
        "-DASSIMP_BUILD_NO_MDC_IMPORTER",
        "-DASSIMP_BUILD_NO_MD5_IMPORTER",
        "-DASSIMP_BUILD_NO_STL_IMPORTER",
        "-DASSIMP_BUILD_NO_LWO_IMPORTER",
        "-DASSIMP_BUILD_NO_DXF_IMPORTER",
        "-DASSIMP_BUILD_NO_NFF_IMPORTER",
        "-DASSIMP_BUILD_NO_RAW_IMPORTER",
        "-DASSIMP_BUILD_NO_SIB_IMPORTER",
        "-DASSIMP_BUILD_NO_OFF_IMPORTER",
        "-DASSIMP_BUILD_NO_AC_IMPORTER",
        "-DASSIMP_BUILD_NO_BVH_IMPORTER",
        "-DASSIMP_BUILD_NO_IRRMESH_IMPORTER",
        "-DASSIMP_BUILD_NO_IRR_IMPORTER",
        "-DASSIMP_BUILD_NO_Q3D_IMPORTER",
        "-DASSIMP_BUILD_NO_B3D_IMPORTER",
        "-DASSIMP_BUILD_NO_COLLADA_IMPORTER",
        "-DASSIMP_BUILD_NO_TERRAGEN_IMPORTER",
        "-DASSIMP_BUILD_NO_CSM_IMPORTER",
        "-DASSIMP_BUILD_NO_3D_IMPORTER",
        "-DASSIMP_BUILD_NO_LWS_IMPORTER",
        "-DASSIMP_BUILD_NO_OGRE_IMPORTER",
        "-DASSIMP_BUILD_NO_OPENGEX_IMPORTER",
        "-DASSIMP_BUILD_NO_MS3D_IMPORTER",
        "-DASSIMP_BUILD_NO_COB_IMPORTER",
        "-DASSIMP_BUILD_NO_BLEND_IMPORTER",
        "-DASSIMP_BUILD_NO_Q3BSP_IMPORTER",
        "-DASSIMP_BUILD_NO_NDO_IMPORTER",
        "-DASSIMP_BUILD_NO_IFC_IMPORTER",
        "-DASSIMP_BUILD_NO_XGL_IMPORTER",
        "-DASSIMP_BUILD_NO_FBX_IMPORTER",
        "-DASSIMP_BUILD_NO_ASSBIN_IMPORTER",
        "-DASSIMP_BUILD_NO_GLTF_IMPORTER",
        "-DASSIMP_BUILD_NO_C4D_IMPORTER",
        "-DASSIMP_BUILD_NO_3MF_IMPORTER",
        "-DASSIMP_BUILD_NO_X3D_IMPORTER",
        "-DASSIMP_BUILD_NO_MMD_IMPORTER",
        "-DASSIMP_BUILD_NO_STEP_IMPORTER",

        "-DASSIMP_BUILD_NO_COLLADA_EXPORTER",
        "-DASSIMP_BUILD_NO_X_EXPORTER",
        "-DASSIMP_BUILD_NO_STEP_EXPORTER",
        "-DASSIMP_BUILD_NO_OBJ_EXPORTER",
        "-DASSIMP_BUILD_NO_STL_EXPORTER",
        "-DASSIMP_BUILD_NO_PLY_EXPORTER",
        "-DASSIMP_BUILD_NO_3DS_EXPORTER",
        "-DASSIMP_BUILD_NO_GLTF_EXPORTER",
        "-DASSIMP_BUILD_NO_ASSBIN_EXPORTER",
        "-DASSIMP_BUILD_NO_ASSXML_EXPORTER",
        "-DASSIMP_BUILD_NO_X3D_EXPORTER",
        "-DASSIMP_BUILD_NO_FBX_EXPORTER",
        "-DASSIMP_BUILD_NO_3MF_EXPORTER",
        "-DASSIMP_BUILD_NO_ASSJSON_EXPORTER",
    ],
    deps = ["@utfcpp"],
)
