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

#include "mech/web_server.h"

namespace mjmech {
namespace mech {

class WebServer::Impl {
 public:
  Impl(const boost::asio::executor& executor, const Options& options)
      : executor_(executor),
        options_(options) {}

  void AsyncStart(mjlib::io::ErrorCallback callback) {
    boost::asio::post(
        executor_,
        std::bind(callback, mjlib::base::error_code()));
  }

  boost::asio::executor executor_;
  const Options options_;
};

WebServer::WebServer(const boost::asio::executor& executor,
                     const Options& options)
    : impl_(std::make_unique<Impl>(executor, options)) {}

WebServer::~WebServer() {}

void WebServer::AsyncStart(mjlib::io::ErrorCallback callback) {
  impl_->AsyncStart(callback);
}

}
}
