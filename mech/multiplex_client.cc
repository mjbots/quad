// Copyright 2019 Josh Pieper, jjp@pobox.com.  All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "mech/multiplex_client.h"

#include <sched.h>

#include <functional>

#include <boost/algorithm/string.hpp>
#include <boost/asio/post.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <boost/filesystem.hpp>

#include <fmt/format.h>

#include "mjlib/base/fail.h"
#include "mjlib/base/program_options_archive.h"
#include "mjlib/base/system_error.h"

#include "mjlib/io/repeating_timer.h"

#include "base/logging.h"

namespace pl = std::placeholders;
namespace fs = boost::filesystem;

namespace {
void ThrowIf(bool value, std::string_view message = "") {
  if (value) {
    throw mjlib::base::system_error::syserrno(message.data());
  }
}

std::optional<std::string> ReadContents(const std::string& filename) {
  std::ifstream inf(filename);
  if (!inf.is_open()) { return {}; }

  std::ostringstream ostr;
  ostr << inf.rdbuf();
  return ostr.str();
}
}

namespace mjmech {
namespace mech {

class MultiplexClient::Impl {
 public:
  Impl(const boost::asio::executor& executor)
      : executor_(executor),
        timer_(executor) {
    mjlib::base::ProgramOptionsArchive(&options_).Accept(&parameters_);
  }

  void AsyncStart(mjlib::io::ErrorCallback handler) {
    Client::Options options;
    options.port = parameters_.serial_port;
    options.baud_rate = parameters_.serial_baud;
    options.query_timeout_s = 0.001;
    options.cpu_affinity = parameters_.cpu_affinity;
    client_ = std::make_unique<Client>(executor_, options);
    ProcessRequests();

    // Now we kick off a timer to fix up the real time priorities if
    // we can.
    timer_.start(boost::posix_time::milliseconds(100),
                 std::bind(&Impl::FixPriorities, this,
                           std::placeholders::_1));

    boost::asio::post(
        executor_,
        std::bind(handler, mjlib::base::error_code()));
  }

  void FixPriorities(const mjlib::base::error_code& ec) {
    if (ec == boost::asio::error::operation_aborted) { return; }
    mjlib::base::FailIf(ec);
    if (done_) { return; }

    // See if we can fix the priorities of the interrupt thread yet.
    // It sure would be nice if linux provided a non-racy way to do
    // this, but, it is free?
    auto maybe_irq =
        ReadContents("/sys/class/tty/" +
                     fs::path(parameters_.serial_port).filename().string() +
                     "/irq");
    if (!maybe_irq) { return; }

    const int irq = std::stoi(boost::algorithm::trim_copy(*maybe_irq));
    const auto desired_prefix = fmt::format("irq/{}-", irq);

    const auto maybe_irq_pid = [&]() -> std::optional<int> {
      // Now we have to look through all of /proc for any processes that
      // are named after irq/#.  Ick.
      for (auto item : fs::directory_iterator("/proc")) {
        auto maybe_contents = ReadContents((item.path() / "comm").string());
        if (!maybe_contents) { continue; }

        if (boost::starts_with(*maybe_contents, desired_prefix)) {
          return std::stoi(item.path().filename().string());
        }
      }
      return {};
    }();

    if (!maybe_irq_pid) { return; }

    const auto irq_pid = *maybe_irq_pid;

    if (parameters_.cpu_affinity >= 0) {
      log_.warn(fmt::format("Setting serial IRQ {} (pid={}) priorities",
                            irq, irq_pid));
      // Now we've got the PID.  We're going to try and set the affinity
      // as well as the real time priority.
      cpu_set_t cpuset = {};
      CPU_ZERO(&cpuset);
      CPU_SET(parameters_.cpu_affinity, &cpuset);
      ThrowIf(::sched_setaffinity(
                  irq_pid, sizeof(cpu_set_t), &cpuset) < 0);

      struct sched_param params = {};
      params.sched_priority = 99;
      ThrowIf(::sched_setscheduler(irq_pid, SCHED_RR, &params) < 0);
    }

    // We no longer have to keep trying.
    done_ = true;
    timer_.cancel();
  }

  void ProcessRequests() {
    BOOST_ASSERT(client_);

    for (auto callback : callbacks_) {
      boost::asio::post(
          executor_,
          [client=client_.get(), callback]() {
            callback({}, client);
          });
    }

    callbacks_.clear();
  }

  base::LogRef log_ = base::GetLogInstance("MultiplexClient");
  Parameters parameters_;

  boost::asio::executor executor_;
  mjlib::io::RepeatingTimer timer_;

  boost::program_options::options_description options_;

  std::unique_ptr<Client> client_;
  std::list<ClientCallback> callbacks_;
  bool done_ = false;
};

MultiplexClient::MultiplexClient(const boost::asio::executor& executor)
    : impl_(std::make_unique<Impl>(executor)) {}

MultiplexClient::~MultiplexClient() {}

MultiplexClient::Parameters* MultiplexClient::parameters() {
  return &impl_->parameters_;
}

boost::program_options::options_description* MultiplexClient::options() {
  return &impl_->options_;
}

void MultiplexClient::AsyncStart(mjlib::io::ErrorCallback callback) {
  impl_->AsyncStart(callback);
}

void MultiplexClient::RequestClient(ClientCallback callback) {
  if (impl_->client_) {
    boost::asio::post(
        impl_->executor_,
        std::bind(
            callback, mjlib::base::error_code(), impl_->client_.get()));
  } else {
    impl_->callbacks_.push_back(callback);
  }
}
}
}
