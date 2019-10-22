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

#include <algorithm>
#include <cstdlib>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <boost/asio/strand.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/http.hpp>
#include <boost/beast/websocket.hpp>
#include <boost/program_options.hpp>

#include <fmt/format.h>

#include "mjlib/base/error_code.h"
#include "mjlib/base/fail.h"

namespace {
namespace pl = std::placeholders;
namespace po = boost::program_options;
namespace beast = boost::beast;
namespace http = beast::http;
namespace websocket = beast::websocket;
using tcp = boost::asio::ip::tcp;

std::string_view GetMimeType(std::string_view path) {
  const auto ext = [&]() {
    const auto pos = path.rfind(".");
    if (pos == std::string::npos) { return std::string_view(); }
    return path.substr(pos);
  }();

  struct Mapping {
    std::string_view extension;
    std::string_view mime_type;
  };
  constexpr Mapping mappings[] = {
    { ".htm", "text/html" },
    { ".html", "text/html" },
    { ".css", "text/css" },
    { ".txt", "text/plain" },
    { ".js", "application/javascript" },
    { ".json", "application/json" },
    { ".xml", "application/xml" },
    { ".png", "image/png" },
    { ".jpeg", "image/jpeg" },
    { ".jpg", "image/jpg" },
    { ".gif", "image/gif" },
    { ".ico", "image/vnd.microsoft.icon" },
    { ".svg", "image/svg+xml" },
  };
  for (const auto& mapping : mappings) {
    if (ext == mapping.extension) {
      return mapping.mime_type;
    }
  }
  return "application/text";
}

class WebsocketSession
    : public std::enable_shared_from_this<WebsocketSession> {
 public:
  explicit WebsocketSession(beast::tcp_stream socket)
      : websocket_(std::move(socket)) {}

  template <typename Body>
  void Start(http::request<Body> request) {
    websocket_.set_option(
        websocket::stream_base::timeout::suggested(
            beast::role_type::server));
    websocket_.set_option(
        websocket::stream_base::decorator(
            [](websocket::response_type& response) {
              response.set(http::field::server,
                           std::string("test_v1 ws-js-async"));
            }));

    websocket_.accept(request);
    StartRead();
  }

  void StartRead() {
    websocket_.async_read(
        buffer_,
        std::bind(&WebsocketSession::HandleRead, shared_from_this(),
                  pl::_1));
  }

  void HandleRead(beast::error_code ec) {
    if (ec == boost::beast::error::timeout ||
        ec == boost::asio::error::broken_pipe ||
        ec == boost::asio::error::not_connected) {
      Close();
      return;
    }
    mjlib::base::FailIf(ec);

    // Print out what we got somehow.
    std::cout << "got message: " <<
        beast::make_printable(buffer_.data()) << "\n";
    buffer_.clear();

    message_ = fmt::format("{}", count_++);

    websocket_.async_write(
        boost::asio::buffer(message_),
        std::bind(&WebsocketSession::HandleWrite, shared_from_this(),
                  pl::_1));
  }

  void HandleWrite(beast::error_code ec) {
    mjlib::base::FailIf(ec);

    StartRead();
  }

  void Close() {
    websocket_.async_close(
        websocket::close_code::normal,
        std::bind(&WebsocketSession::HandleClose, shared_from_this(),
                  pl::_1));
  }

  void HandleClose(beast::error_code) {
  }

 private:
  websocket::stream<beast::tcp_stream> websocket_;
  beast::flat_buffer buffer_;

  std::string message_;
  uint32_t count_ = 0;
};

class Session : public std::enable_shared_from_this<Session> {
 public:
  Session(tcp::socket socket)
      : stream_(std::move(socket)) {}

  void Start() {
    StartRead();
  }

  void StartRead() {
    request_ = {};

    stream_.expires_after(std::chrono::seconds(30));

    http::async_read(
        stream_, buffer_, request_,
        std::bind(&Session::HandleRead, shared_from_this(), pl::_1));
  }

  void HandleRead(beast::error_code ec) {
    if (ec == http::error::end_of_stream) {
      return Close();
    }

    if (ec == boost::beast::error::timeout) {
      return Close();
    }

    mjlib::base::FailIf(ec);

    HandleRequest(std::move(request_));
  }

  template <typename Body>
  void HandleRequest(http::request<Body> request) {
    const auto make_response = [&](
        auto code,
        std::string_view content_type = "text/html") {
      http::response<http::string_body> response{code, request.version()};
      response.set(http::field::server, BOOST_BEAST_VERSION_STRING);
      response.set(http::field::content_type, content_type);
      response.keep_alive(request.keep_alive());
      return response;
    };

    const auto bad_request = [&](std::string_view message) {
      auto response = make_response(http::status::bad_request);
      response.body() = message;
      return response;
    };

    const auto not_found = [&](std::string_view filename) {
      auto response = make_response(http::status::not_found);
      response.body() = fmt::format("The resource '{}' was not found",
                                    filename);
      return response;
    };

    const auto server_error = [&](std::string_view why) {
      auto response = make_response(http::status::internal_server_error);
      response.body() = why;
      return response;
    };

    const auto send = [&](auto response) {
      response.prepare_payload();
      Send(std::move(response));
    };

    if (request.method() != http::verb::get &&
        request.method() != http::verb::head) {
      return send(bad_request("Unknown HTTP-method"));
    }

    if (request.target().empty() ||
        request.target()[0] != '/' ||
        request.target().find("..") != std::string_view::npos) {
      return send(bad_request("Illegal request target"));
    }

    if (request.target() == "/control") {
      // This is only valid for websocket requests.
      if (!websocket::is_upgrade(request)) {
        return send(bad_request("Only valid for websocket"));
      }

      std::cout << "got a websocket request\n";

      // Pass this off to our websocket handler.
      std::make_shared<WebsocketSession>(
          std::move(stream_))->Start(std::move(request));

      return;
    }

    // Fall back to reading a file.
    std::string path = std::string(request.target());
    if (request.target().back() == '/') {
      path += "index.html";
    }
    path = path.substr(1); // For now just get relative paths.

    beast::error_code ec;
    http::file_body::value_type body;
    body.open(path.c_str(), beast::file_mode::scan, ec);

    if (ec == beast::errc::no_such_file_or_directory) {
      std::cout << "path: " << path << "\n";
      return send(not_found(std::string(request.target())));
    }

    if (ec) {
      return send(server_error(ec.message()));
    }

    const auto size = body.size();

    if (request.method() == http::verb::head) {
      http::response<http::empty_body> response{
        http::status::ok, request.version()};
      response.set(http::field::server, BOOST_BEAST_VERSION_STRING);
      response.set(http::field::content_type, GetMimeType(path));
      response.content_length(size);
      response.keep_alive(request.keep_alive());
      return send(std::move(response));
    }

    http::response<http::file_body> response{
      std::piecewise_construct,
          std::make_tuple(std::move(body)),
          std::make_tuple(http::status::ok, request.version())};
    response.set(http::field::server, BOOST_BEAST_VERSION_STRING);
    response.set(http::field::content_type, GetMimeType(path));
    response.content_length(size);
    response.keep_alive(request.keep_alive());
    return send(std::move(response));
  }

  template <bool isRequest, typename Body>
  void Send(http::message<isRequest, Body> message) {
    auto sp = std::make_shared<http::message<isRequest, Body>>(
        std::move(message));
    http::async_write(
        stream_,
        *sp,
        std::bind(&Session::HandleWrite, shared_from_this(),
                  pl::_1, sp->need_eof(), sp));
  }

  void HandleWrite(beast::error_code ec, bool close, std::shared_ptr<void>) {
    mjlib::base::FailIf(ec);

    if (close) {
      Close();
      return;
    }

    StartRead();
  }

  void Close() {
    beast::error_code ec;
    stream_.socket().shutdown(tcp::socket::shutdown_send, ec);
    // Ignore any errors.
  }

 private:
  beast::tcp_stream stream_;
  http::request<http::string_body> request_;
  beast::flat_buffer buffer_;
};

class Listener : public std::enable_shared_from_this<Listener> {
 public:
  Listener(boost::asio::io_context& context,
           tcp::endpoint endpoint)
      : context_(context),
        acceptor_(context) {
    beast::error_code ec;

    acceptor_.open(endpoint.protocol(), ec);
    mjlib::base::FailIf(ec);

    acceptor_.set_option(boost::asio::socket_base::reuse_address(true), ec);
    mjlib::base::FailIf(ec);

    acceptor_.bind(endpoint, ec);
    mjlib::base::FailIf(ec);

    acceptor_.listen(boost::asio::socket_base::max_listen_connections, ec);
    mjlib::base::FailIf(ec);
  }

  void Start() {
    StartAccept();
  }

  void StartAccept() {
    acceptor_.async_accept(
        boost::asio::make_strand(context_),
        std::bind(&Listener::HandleAccept, shared_from_this(),
                  pl::_1, pl::_2));
  }

  void HandleAccept(beast::error_code ec, tcp::socket socket) {
    mjlib::base::FailIf(ec);

    std::make_shared<Session>(std::move(socket))->Start();

    StartAccept();
  }

 private:
  boost::asio::io_context& context_;
  tcp::acceptor acceptor_;
};
}

int main(int argc, char** argv) {
  po::options_description desc;

  std::string address = "0.0.0.0";
  int port = 4558;

  desc.add_options()
      ("help,h", "display usage")
      ("address,a", po::value(&address), "")
      ("port,p", po::value(&port), "")
      ;

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);

  if (vm.count("help")) {
    std::cout << desc;
    return 0;
  }

  boost::asio::io_context context;

  std::make_shared<Listener>(
      context, tcp::endpoint(
          boost::asio::ip::make_address(address),
          port))->Start();

  context.run();

  return 0;
}
