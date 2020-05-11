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

cc_library(
    name = "base",
    srcs = [
        "aspect_ratio.cc",
        "context.cc",
        "linux_input.cc",
        "logging.cc",
        "quaternion.cc",
        "system_fd.cc",
        "telemetry_remote_debug_server.cc",
        "timestamped_log.cc",
        "udp_data_link.cc",
        "udp_socket.cc",
    ],
    hdrs = glob([
        "*.h",
    ]),
    deps = [
        "@boost",
        "@boost//:system",
        "@eigen",
        "@fmt",
        "@log4cpp",
        "@com_github_mjbots_mjlib//mjlib/base:clipp",
        "@com_github_mjbots_mjlib//mjlib/base:clipp_archive",
        "@com_github_mjbots_mjlib//mjlib/base:eigen",
        "@com_github_mjbots_mjlib//mjlib/base:error_code",
        "@com_github_mjbots_mjlib//mjlib/base:fail",
        "@com_github_mjbots_mjlib//mjlib/base:fast_stream",
        "@com_github_mjbots_mjlib//mjlib/base:json5_read_archive",
        "@com_github_mjbots_mjlib//mjlib/base:json5_write_archive",
        "@com_github_mjbots_mjlib//mjlib/base:limit",
        "@com_github_mjbots_mjlib//mjlib/base:recording_stream",
        "@com_github_mjbots_mjlib//mjlib/base:system_error",
        "@com_github_mjbots_mjlib//mjlib/base:tokenizer",
        "@com_github_mjbots_mjlib//mjlib/base:visitor",
        "@com_github_mjbots_mjlib//mjlib/base:visit_archive",
        "@com_github_mjbots_mjlib//mjlib/io:async_sequence",
        "@com_github_mjbots_mjlib//mjlib/io:async_types",
        "@com_github_mjbots_mjlib//mjlib/io:debug_time",
        "@com_github_mjbots_mjlib//mjlib/io:realtime_executor",
        "@com_github_mjbots_mjlib//mjlib/io:repeating_timer",
        "@com_github_mjbots_mjlib//mjlib/io:stream_factory",
        "@com_github_mjbots_mjlib//mjlib/telemetry:file_writer",
        "@com_github_mjbots_mjlib//mjlib/telemetry:binary_read_archive",
        "@com_github_mjbots_mjlib//mjlib/telemetry:binary_write_archive",
        "@sophus",
        "@org_llvm_libcxx//:libcxx",
    ],
    defines = [
        "BOOST_ASIO_ENABLE_OLD_SERVICES",
    ],
    features = [
        "dbg",
    ],
)

cc_test(
    name = "test",
    srcs = ["test/" + x for x in [
        "aspect_ratio_test.cc",
        "named_type_test.cc",
        "quaternion_test.cc",
        "signal_result_test.cc",
        "sophus_test.cc",
        "telemetry_log_registrar_test.cc",
        "telemetry_registry_test.cc",
        "test_main.cc",
        "ukf_filter_test.cc",
    ]],
    deps = [
        ":base",
        "@boost//:test",
    ],
)

cc_binary(
    name = "linux_input_manual_test",
    srcs = ["test/linux_input_manual_test.cc"],
    deps = [":base"],
)

exports_files(["module_main.cc"])
