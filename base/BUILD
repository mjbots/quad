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
        "debug_i2c_generator.cc",
        "i2c_factory.cc",
        "linux_i2c_generator.cc",
        "linux_input.cc",
        "logging.cc",
        "program_options.cc",
        "quaternion.cc",
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
        "@i2c-tools",
        "@log4cpp",
        "@moteus//mjlib/base:eigen",
        "@moteus//mjlib/base:error_code",
        "@moteus//mjlib/base:fail",
        "@moteus//mjlib/base:fast_stream",
        "@moteus//mjlib/base:json5_read_archive",
        "@moteus//mjlib/base:json5_write_archive",
        "@moteus//mjlib/base:limit",
        "@moteus//mjlib/base:program_options_archive",
        "@moteus//mjlib/base:recording_stream",
        "@moteus//mjlib/base:system_error",
        "@moteus//mjlib/base:visitor",
        "@moteus//mjlib/base:visit_archive",
        "@moteus//mjlib/io:async_types",
        "@moteus//mjlib/io:debug_time",
        "@moteus//mjlib/io:repeating_timer",
        "@moteus//mjlib/io:stream_factory",
        "@moteus//mjlib/telemetry:telemetry_archive",
        "@moteus//mjlib/telemetry:telemetry_util",
        "@snappy",
        "@org_llvm_libcxx//:libcxx",
    ],
    defines = [
        "BOOST_ASIO_ENABLE_OLD_SERVICES",
    ],
)

cc_test(
    name = "test",
    srcs = ["test/" + x for x in [
        "async_sequence_test.cc",
        "attitude_estimator_test.cc",
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
