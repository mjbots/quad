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

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "base",
    srcs = [
        "comm_factory.cc",
        "concrete_comm_factory.cc",
        "context.cc",
        "debug_i2c_generator.cc",
        "error_code.cc",
        "fail.cc",
        "i2c_factory.cc",
        "linux_i2c_generator.cc",
        "linux_input.cc",
        "logging.cc",
        "point3d.cc",
        "program_options.cc",
        "quaternion.cc",
        "telemetry_log.cc",
        "telemetry_remote_debug_server.cc",
        "udp_data_link.cc",
        "udp_socket.cc",
        "virtual_deadline_timer.cc",
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
        "@snappy",
    ],
    defines = [
        "BOOST_ASIO_ENABLE_OLD_SERVICES",
    ],
)

cc_test(
    name = "base_test",
    srcs = ["test/" + x for x in [
        "attitude_estimator_test.cc",
        "comm_test.cc",
        "debug_deadline_service_test.cc",
        "json_archive_test.cc",
        "now_test.cc",
        "program_options_test.cc",
        "quaternion_test.cc",
        "signal_result_test.cc",
        "telemetry_archive_test.cc",
        "telemetry_log_registrar_test.cc",
        "telemetry_log_test.cc",
        "telemetry_registry_test.cc",
        "test_main.cc",
        "tf_test.cc",
        "ukf_filter_test.cc",
        "virtual_deadline_timer_test.cc",
    ]],
    deps = [
        ":base",
        "@boost//:test",
    ],
)

exports_files(["module_main.cc"])
