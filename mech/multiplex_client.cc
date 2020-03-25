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
#include "mech/rpi3_hat_raw_spi.h"

namespace pl = std::placeholders;
namespace fs = boost::filesystem;

namespace mjmech {
namespace mech {

namespace {
using ConcreteClient = Rpi3HatRawSpi;
}

class MultiplexClient::Impl {
 public:
  Impl(const boost::asio::executor& executor)
      : executor_(executor),
        timer_(executor) {
    mjlib::base::ProgramOptionsArchive(&options_).Accept(&parameters_);
  }

  void AsyncStart(mjlib::io::ErrorCallback handler) {
    ConcreteClient::Options options;
    options.query_timeout_s = 0.002;
    options.cpu_affinity = parameters_.cpu_affinity;
    options.spi_speed_hz = parameters_.spi_speed_hz;
    client_ = std::make_unique<ConcreteClient>(executor_, options);
    ProcessRequests();

    boost::asio::post(
        executor_,
        std::bind(std::move(handler), mjlib::base::error_code()));
  }

  void ProcessRequests() {
    BOOST_ASSERT(client_);

    for (auto& callback : callbacks_) {
      boost::asio::post(
          executor_,
          [client=client_.get(), callback=std::move(callback)]() mutable {
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
  impl_->AsyncStart(std::move(callback));
}

void MultiplexClient::RequestClient(ClientCallback callback) {
  if (impl_->client_) {
    boost::asio::post(
        impl_->executor_,
        std::bind(
            std::move(callback), mjlib::base::error_code(), impl_->client_.get()));
  } else {
    impl_->callbacks_.push_back(std::move(callback));
  }
}
}
}
