// Copyright 2019 Josh Pieper, jjp@pobox.com.
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

#pragma once

#include <functional>
#include <map>
#include <memory>
#include <string>

#include <boost/asio/any_io_executor.hpp>

#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>

#include "mjlib/io/async_types.h"

namespace mjmech {
namespace mech {

/// An embedded web server that can serve static pages, as well as
/// websocket connections.  The server itself is run in a background
/// thread.
class WebServer {
 public:
  using WebsocketStream =
      boost::beast::websocket::stream<boost::beast::tcp_stream>;
  using WebsocketHandler = std::function<void (WebsocketStream)>;

  struct Options {
    std::string address = "0.0.0.0";
    int port = -1;

    struct Root {
      /// The URL prefix which is used for this root.
      std::string prefix;

      /// The location in the filesystem to serve these files.
      std::string root;
    };

    /// One or more document roots.  Resources that match multiple
    /// prefixes are tried in order.
    std::vector<Root> document_roots;

    /// The following URL prefixes will be treated as exclusively
    /// websocket endpoints.  No websocket "sub-protocols" are
    /// supported.  Once the websocket connection has been upgraded,
    /// the given handler will be invoked.  These websockets are given
    /// an executor that runs in a background thread, and may only
    /// execute in that thread.
    struct Websocket {
      std::string endpoint;

      WebsocketHandler handler;
    };

    std::vector<Websocket> websocket_handlers;
  };

  WebServer(const boost::asio::any_io_executor&, const Options&);
  ~WebServer();

  void AsyncStart(mjlib::io::ErrorCallback);

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};
}
}
