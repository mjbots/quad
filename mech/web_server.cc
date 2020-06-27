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

#include "mech/web_server.h"

#include <thread>

#include <boost/algorithm/string.hpp>
#include <boost/beast/http.hpp>
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>

#include <fmt/format.h>

#include "mjlib/base/fail.h"

#include "base/logging.h"

#include "mech/mime_type.h"

namespace pl = std::placeholders;
namespace fs = boost::filesystem;
namespace beast = boost::beast;
namespace http = beast::http;
namespace websocket = beast::websocket;
using tcp = boost::asio::ip::tcp;

namespace mjmech {
namespace mech {

namespace {
constexpr auto kRequestTimeout = std::chrono::seconds(30);

class ResponseFactory {
 public:
  ResponseFactory(http::request<http::string_body>* request)
      : request_(request) {}

  template <typename Body = http::string_body, typename Code>
  http::response<Body> MakeResponse(
      Code code,
      std::string_view content_type = "text/html") const {
    http::response<Body> response{code, request_->version()};
    SetupResponse(&response, content_type);
    return response;
  }

  template <typename Response>
  void SetupResponse(Response* response, std::string_view content_type) const {
    response->set(http::field::server, BOOST_BEAST_VERSION_STRING);
    response->set(http::field::content_type, content_type);
    response->keep_alive(request_->keep_alive());
  }

  http::response<http::string_body>
  BadRequest(std::string_view message) const {
    auto response = MakeResponse<>(http::status::bad_request);
    response.body() = message;
    return response;
  }

  http::response<http::string_body> NotFound(std::string_view filename) const {
    auto response = MakeResponse<>(http::status::not_found);
    response.body() = fmt::format("The resource '{}' was not found", filename);
    return response;
  }

  http::response<http::string_body> ServerError(std::string_view why) const {
    auto response = MakeResponse(http::status::internal_server_error);
    response.body() = why;
    return response;
  }

 private:
  const http::request<http::string_body>* const request_;
};

template <typename Body>
void StartWebsocket(WebServer::WebsocketHandler handler,
                    beast::tcp_stream socket,
                    http::request<Body> request) {
  using Stream = websocket::stream<beast::tcp_stream>;
  Stream websocket(std::move(socket));

  websocket.set_option(
      websocket::stream_base::timeout::suggested(
          beast::role_type::server));
  websocket.set_option(
      websocket::stream_base::decorator(
          [](websocket::response_type& response) {
            response.set(http::field::server,
                         std::string("mjmech WebServer"));
          }));
  websocket.accept(request);

  handler(std::move(websocket));
}
} // namespace

class WebServer::Impl {
 public:
  Impl(const boost::asio::executor& executor, const Options& options)
      : executor_(executor),
        options_(options) {}

  ~Impl() {
    boost::asio::post(
        child_context_.get_executor(),
        [this]() {
          child_context_.stop();
        });
    child_thread_.join();
  }

  void AsyncStart(mjlib::io::ErrorCallback callback) {
    child_thread_ = std::thread(std::bind(&Impl::ChildRun, this));

    boost::asio::post(
        executor_,
        std::bind(std::move(callback), mjlib::base::error_code()));
  }

  void ChildRun() {
    std::make_shared<Listener>(
        this, child_context_.get_executor(),
        tcp::endpoint(boost::asio::ip::make_address(options_.address),
                      options_.port))->Start();
    child_context_.run();
  }

  class Session : public std::enable_shared_from_this<Session> {
   public:
    Session(Impl* parent,
            tcp::socket socket)
        : parent_(parent),
          stream_(std::move(socket)) {}

    void Start() {
      StartRead();
    }

    void StartRead() {
      request_ = {};

      stream_.expires_after(kRequestTimeout);

      http::async_read(
          stream_,
          buffer_,
          request_,
          std::bind(&Session::HandleRead, shared_from_this(), pl::_1));
    }

    void HandleRead(beast::error_code ec) {
      if (ec == http::error::end_of_stream ||
          ec == boost::beast::error::timeout) {
        return Close();
      }

      if (ec) {
        log_.warn("Unknown error: " + ec.message());
        return Close();
      }

      HandleRequest(std::move(request_));
      request_ = {};
    }

    void HandleRequest(http::request<http::string_body> request) {
      ResponseFactory response_factory(&request);

      const auto send = [&](auto response) {
        response.prepare_payload();
        Send(std::move(response));
      };

      if (request.method() != http::verb::get &&
          request.method() != http::verb::head) {
        return send(response_factory.BadRequest("Unknown HTTP method"));
      }

      if (request.target().empty() ||
          request.target()[0] != '/' ||
          request.target().find("..") != std::string_view::npos) {
        return send(response_factory.BadRequest("Illegal request target"));
      }

      // Look to see if we match any websocket endpoints.
      for (const auto& handler : parent_->options_.websocket_handlers) {
        if (request.target() == handler.endpoint) {
          // Yes, we have one.

          // Websocket has its own timeout mechanism.
          stream_.expires_never();

          StartWebsocket(handler.handler, std::move(stream_), std::move(request));
          return;
        }
      }

      std::string path = std::string(request.target());
      if (request.target().back() == '/') {
        path += "index.html";
      }

      // Now look through our document roots to see if there is a file
      // we can serve.
      for (const auto& root : parent_->options_.document_roots) {
        if (!boost::starts_with(path, root.prefix)) { continue; }

        const std::string this_path =
            root.root + "/" + path.substr(root.prefix.size());
        if (!fs::exists(this_path)) { continue; }

        beast::error_code ec;
        http::file_body::value_type body;
        body.open(this_path.c_str(), beast::file_mode::scan, ec);

        if (ec) {
          return send(response_factory.ServerError(ec.message()));
        }

        const auto size = body.size();

        if (request.method() == http::verb::head) {
          auto response = response_factory.MakeResponse<http::empty_body>(
              http::status::ok, GetMimeType(path));
          response.content_length(size);
          return send(std::move(response));
        }

        // Because it is annoying to construct a
        // http::response<file_body>, we do that manually here.
        http::response<http::file_body> response{
          std::piecewise_construct,
              std::make_tuple(std::move(body)),
              std::make_tuple(http::status::ok, request.version())};
        response_factory.SetupResponse(&response, GetMimeType(path));
        response.content_length(size);
        return send(std::move(response));
      }

      return send(response_factory.NotFound(std::string(request.target())));
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
      if (ec) {
        log_.warn(fmt::format("Error writing: {}", ec.message()));
        Close();
        return;
      }

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
    Impl* const parent_;
    beast::tcp_stream stream_;
    http::request<http::string_body> request_;
    beast::flat_buffer buffer_;

    base::LogRef log_ = base::GetLogInstance("WebServer");
  };

  class Listener : public std::enable_shared_from_this<Listener> {
   public:
    Listener(Impl* parent,
             const boost::asio::executor& executor,
             tcp::endpoint endpoint)
        : parent_(parent),
          executor_(executor),
          acceptor_(executor) {
      beast::error_code ec;

      acceptor_.open(endpoint.protocol(), ec);
      mjlib::base::FailIf(ec);

      acceptor_.set_option(boost::asio::socket_base::reuse_address(true), ec);
      mjlib::base::FailIf(ec);

      acceptor_.bind(endpoint, ec);
      mjlib::base::FailIf(ec);

      acceptor_.listen(boost::asio::socket_base::max_listen_connections, ec);
      mjlib::base::FailIf(ec);

      log_.warn(fmt::format("listening on {}",
                            boost::lexical_cast<std::string>(endpoint)));
    }

    void Start() {
      StartAccept();
    }

    void StartAccept() {
      acceptor_.async_accept(
          std::bind(&Listener::HandleAccept, shared_from_this(),
                    pl::_1, pl::_2));
    }

    void HandleAccept(beast::error_code ec, tcp::socket socket) {
      mjlib::base::FailIf(ec);

      std::make_shared<Session>(parent_, std::move(socket))->Start();

      StartAccept();
    }

    Impl* const parent_;
    boost::asio::executor executor_;
    tcp::acceptor acceptor_;

    base::LogRef log_ = base::GetLogInstance("WebServer");
  };

  boost::asio::executor executor_;
  const Options options_;

  std::thread child_thread_;
  boost::asio::io_context child_context_;
};

WebServer::WebServer(const boost::asio::executor& executor,
                     const Options& options)
    : impl_(std::make_unique<Impl>(executor, options)) {}

WebServer::~WebServer() {}

void WebServer::AsyncStart(mjlib::io::ErrorCallback callback) {
  impl_->AsyncStart(std::move(callback));
}

}
}
