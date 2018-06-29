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

load("@com_github_mjbots_bazel_deps//tools/workspace:generate_file.bzl", "generate_file")
load("@com_github_mjbots_bazel_deps//tools/workspace:template_file.bzl", "template_file")
load("@com_github_mjbots_bazel_deps//tools/workspace:autoconf_config.bzl", "autoconf_config")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "gst-rpicamsrc",
    srcs = ["src/" + x for x in [
        'gstrpicamsrc.c',
        'gstrpicamsrc.h',
        'gstrpicamsrcdeviceprovider.c',
        'gstrpicamsrcdeviceprovider.h',
        'RaspiCapture.c',
        'RaspiCapture.h',
        'RaspiCamControl.c',
        'RaspiCamControl.h',
        'RaspiPreview.c',
        'RaspiPreview.h',
        'RaspiCLI.c',
        'RaspiCLI.h',
        'gstrpicam-enum-types.h',
        'gstrpicam-enum-types.c',
        'gstrpicam_types.h',
    ]] + [
        "private/config.h",
    ],
    copts = [
        "-DHAVE_CONFIG_H",
        "-I$(GENDIR)/external/gst-rpicamsrc/private",
        "-I$(GENDIR)/external/gst-rpicamsrc/src",
        "-Iexternal/gst-rpicamsrc/src",
    ],
    deps = [
        "@gst-plugins-base//:gstvideo",
        "@gst-plugins-base//:gstpbutils",
        "@gstreamer//:libgstbase",
        "@gstreamer//:gstreamer",
        "@raspberrypi-firmware",
    ],
)

genrule(
    name = "enum_types_h",
    srcs = [
        'src/gstrpicam_types.h',
        'src/gstrpicam-enums-template.h',
    ],
    outs = ["src/gstrpicam-enum-types.h"],
    tools = ["@glib//:glib-mkenums"],
    cmd = (
        "$(location @glib//:glib-mkenums) " +
        "--template $(location src/gstrpicam-enums-template.h) " +
        "$(location src/gstrpicam_types.h) " +
        "> $@"),
)

genrule(
    name = "enum_types_c",
    srcs = [
        'src/gstrpicam_types.h',
        'src/tweaked-gstrpicam-enums-template.c',
    ],
    outs = ["src/gstrpicam-enum-types.c"],
    tools = ["@glib//:glib-mkenums"],
    cmd = (
        "$(location @glib//:glib-mkenums) " +
        "--template $(location src/tweaked-gstrpicam-enums-template.c) " +
        "$(location src/gstrpicam_types.h) " +
        "> $@"),
)

template_file(
    name = "src/tweaked-gstrpicam-enums-template.c",
    src = "src/gstrpicam-enums-template.c",
    substitutions = {
        "@filename@" : "@basename@",
    },
)

autoconf_config(
    name = "private/config.h",
    src = "private/config.h.in",
    package = "gst-rpicamsrc",
    version = "a181cd8d4b284d09b5f0e23d9ddb4f0a94e1dc8c",
    defines = [
        "HAVE_DLFCN_H",
        "HAVE_INTTYPES_H",
        "HAVE_LIBBCM_HOST",
        "HAVE_MEMORY_H",
        "HAVE_STDINT_H",
        "HAVE_STDLIB_H",
        "HAVE_STRINGS_H",
        "HAVE_STRING_H",
        "HAVE_SYS_STAT_H",
        "HAVE_SYS_TYPES_H",
        "HAVE_UNISTD_H",
        "STDC_HEADERS",
    ],
)

generate_file(
    name = "private/config.h.in",
    content = """
/* config.h.in.  Generated from configure.ac by autoheader.  */

/* Define to 1 if you have the <dlfcn.h> header file. */
#undef HAVE_DLFCN_H

/* Define to 1 if you have the <inttypes.h> header file. */
#undef HAVE_INTTYPES_H

/* Define to 1 if you have the `bcm_host' library (-lbcm_host). */
#undef HAVE_LIBBCM_HOST

/* Define to 1 if you have the <memory.h> header file. */
#undef HAVE_MEMORY_H

/* Define to 1 if you have the <stdint.h> header file. */
#undef HAVE_STDINT_H

/* Define to 1 if you have the <stdlib.h> header file. */
#undef HAVE_STDLIB_H

/* Define to 1 if you have the <strings.h> header file. */
#undef HAVE_STRINGS_H

/* Define to 1 if you have the <string.h> header file. */
#undef HAVE_STRING_H

/* Define to 1 if you have the <sys/stat.h> header file. */
#undef HAVE_SYS_STAT_H

/* Define to 1 if you have the <sys/types.h> header file. */
#undef HAVE_SYS_TYPES_H

/* Define to 1 if you have the <unistd.h> header file. */
#undef HAVE_UNISTD_H

/* Define to the sub-directory where libtool stores uninstalled libraries. */
#undef LT_OBJDIR

/* Name of package */
#undef PACKAGE

/* Define to the address where bug reports for this package should be sent. */
#undef PACKAGE_BUGREPORT

/* Define to the full name of this package. */
#undef PACKAGE_NAME

/* Define to the full name and version of this package. */
#undef PACKAGE_STRING

/* Define to the one symbol short name of this package. */
#undef PACKAGE_TARNAME

/* Define to the home page for this package. */
#undef PACKAGE_URL

/* Define to the version of this package. */
#undef PACKAGE_VERSION

/* Define to 1 if you have the ANSI C header files. */
#undef STDC_HEADERS

/* Version number of package */
#undef VERSION
    """)
