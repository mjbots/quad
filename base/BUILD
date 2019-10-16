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

cc_library(
    name = "base",
    srcs = [
        "context.cc",
        "linux_input.cc",
        "logging.cc",
        "program_options.cc",
        "quaternion.cc",
        "system_fd.cc",
        "telemetry_log.cc",
        "telemetry_remote_debug_server.cc",
        "udp_data_link.cc",
        "udp_socket.cc",
    ],
    hdrs = glob([
        "*.h",
        "gsl/*.h",
        "meta/*.hpp",
    ]),
    deps = [
        "@boost",
        "@boost//:program_options",
        "@boost//:system",
        "@eigen",
        "@fmt",
        "@log4cpp",
        "@mjlib//mjlib/base:eigen",
        "@mjlib//mjlib/base:error_code",
        "@mjlib//mjlib/base:fail",
        "@mjlib//mjlib/base:fast_stream",
        "@mjlib//mjlib/base:json5_read_archive",
        "@mjlib//mjlib/base:json5_write_archive",
        "@mjlib//mjlib/base:limit",
        "@mjlib//mjlib/base:program_options_archive",
        "@mjlib//mjlib/base:recording_stream",
        "@mjlib//mjlib/base:system_error",
        "@mjlib//mjlib/base:visitor",
        "@mjlib//mjlib/base:visit_archive",
        "@mjlib//mjlib/io:async_sequence",
        "@mjlib//mjlib/io:async_types",
        "@mjlib//mjlib/io:debug_time",
        "@mjlib//mjlib/io:realtime_executor",
        "@mjlib//mjlib/io:repeating_timer",
        "@mjlib//mjlib/io:stream_factory",
        "@mjlib//mjlib/telemetry:telemetry_archive",
        "@mjlib//mjlib/telemetry:telemetry_util",
        "@snappy",
        "@sophus",
        "@org_llvm_libcxx//:libcxx",
    ],
    defines = [
        "BOOST_ASIO_ENABLE_OLD_SERVICES",
    ],
)

cc_test(
    name = "test",
    srcs = ["test/" + x for x in [
        "program_options_test.cc",
        "quaternion_test.cc",
        "signal_result_test.cc",
        "telemetry_log_registrar_test.cc",
        "telemetry_log_test.cc",
        "telemetry_registry_test.cc",
        "test_main.cc",
        "tf_test.cc",
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
