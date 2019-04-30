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

#include <functional>

#include "mjlib/base/program_options_archive.h"

namespace pl = std::placeholders;

namespace mjmech {
namespace mech {

class MultiplexClient::Impl {
 public:
  Impl(boost::asio::io_service& service,
       mjlib::io::StreamFactory& factory)
      : service_(service),
        factory_(factory) {
    mjlib::base::ProgramOptionsArchive(&options_).Accept(&stream_parameters_);
  }

  void AsyncStart(mjlib::io::ErrorCallback handler) {
    factory_.AsyncCreate(
        stream_parameters_,
        std::bind(&Impl::HandleCreate, this, pl::_1, pl::_2, handler));
  }

  void HandleCreate(const mjlib::base::error_code& ec,
                    mjlib::io::SharedStream stream,
                    mjlib::io::ErrorCallback handler) {
    if (!ec) {
      stream_ = stream;
      client_ = std::make_unique<Client>(stream_.get());
      ProcessRequests();
    }
    service_.post(std::bind(handler, ec));
  }

  void ProcessRequests() {
    BOOST_ASSERT(stream_);
    BOOST_ASSERT(client_);

    for (auto callback : callbacks_) {
      service_.post([client=client_.get(), callback]() {
          callback({}, client);
        });
    }

    callbacks_.clear();
  }

  Parameters parameters_;

  boost::asio::io_service& service_;
  mjlib::io::StreamFactory& factory_;

  mjlib::io::StreamFactory::Options stream_parameters_;

  boost::program_options::options_description options_;

  mjlib::io::SharedStream stream_;
  std::unique_ptr<Client> client_;
  std::list<ClientCallback> callbacks_;
};

MultiplexClient::MultiplexClient(boost::asio::io_service& service,
                                 mjlib::io::StreamFactory& factory)
    : impl_(std::make_unique<Impl>(service, factory)) {}

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
    impl_->service_.post(
        std::bind(
            callback, mjlib::base::error_code(), impl_->client_.get()));
  } else {
    impl_->callbacks_.push_back(callback);
  }
}
}
}
