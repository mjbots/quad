// Copyright 2019-2020 Josh Pieper, jjp@pobox.com.
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

#include "mech/web_control.h"

#include <iostream>

#include <boost/beast/websocket.hpp>
#include <boost/filesystem.hpp>

#include "mjlib/base/clipp_archive.h"
#include "mjlib/base/fail.h"
#include "mjlib/base/json5_read_archive.h"
#include "mjlib/base/json5_write_archive.h"

#include "base/logging.h"
#include "mech/web_server.h"

namespace fs = boost::filesystem;
namespace pl = std::placeholders;
namespace beast = boost::beast;
namespace websocket = beast::websocket;

namespace mjmech {
namespace mech {

constexpr const char* kAssetPath = "web_control_assets";

namespace {
std::string FindAssetPath() {
  fs::path start = fs::canonical("/proc/self/exe").remove_filename();

  for (const char* to_try : { ".",
          "quadruped.runfiles/com_github_mjbots_mech/mech"}) {
    fs::path this_root = start / to_try;
    fs::path assets = this_root / kAssetPath;
    if (fs::exists(assets)) {
      return assets.native();
    }
  }

  // Hmmph, we couldn't find it.
  mjlib::base::Fail("Could not locate WebControl assets: " + start.native());
}

struct WebCommand {
  std::optional<QuadrupedCommand> command;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(command));
  }
};
}  // namespace

class WebControl::Impl {
 public:
  Impl(const boost::asio::executor& executor,
       QuadrupedControl* quadruped_control)
      : executor_(executor),
        quadruped_control_(quadruped_control) {
  }

  void AsyncStart(mjlib::io::ErrorCallback callback) {
    WebServer::Options server_options;

    server_options.port = parameters_.port;

    server_options.document_roots.push_back(
        {std::string("/"), FindAssetPath()});

    server_options.websocket_handlers.push_back(
        {"/control",
         std::bind(&Impl::HandleControlWebsocket, this, pl::_1)});

    std::cout << "Starting web server\n";
    web_server_ = std::make_unique<WebServer>(executor_, server_options);

    web_server_->AsyncStart(std::move(callback));
  }

  void HandleControlWebsocket(WebServer::WebsocketStream stream) {
    // This stream is running on a different executor, thus we need to
    // keep it segregated.
    std::make_shared<WebsocketServer>(this, std::move(stream))->Start();
  }

  class WebsocketServer : public std::enable_shared_from_this<WebsocketServer> {
   public:
    WebsocketServer(Impl* parent, WebServer::WebsocketStream stream)
        : parent_(parent),
          stream_(std::move(stream)),
          executor_(stream_.get_executor()) {
    }

    void Start() {
      StartRead();
    }

    void StartRead() {
      stream_.async_read(
          buffer_,
          std::bind(&WebsocketServer::HandleRead, shared_from_this(),
                    pl::_1));
    }

    bool MaybeClose(const mjlib::base::error_code& ec) {
      if (ec == beast::error::timeout ||
          ec == boost::asio::error::broken_pipe ||
          ec == boost::asio::error::not_connected ||
          ec == boost::asio::error::connection_reset ||
          ec == beast::websocket::error::closed) {
        Close();
        return true;
      }
      return false;
    }

    void HandleRead(mjlib::base::error_code ec) {
      if (MaybeClose(ec)) { return; }
      mjlib::base::FailIf(ec);

      std::string message(static_cast<const char*>(buffer_.data().data()),
                          buffer_.size());
      buffer_.clear();
      std::istringstream istr(message);

      try {
        const auto command =
            mjlib::base::Json5ReadArchive::Read<WebCommand>(istr);

        boost::asio::post(
            parent_->executor_,
            [self = shared_from_this(), command]() {
              if (command.command) {
                self->parent_->quadruped_control_->Command(*command.command);
              }
              const auto status = self->parent_->quadruped_control_->status();

              boost::asio::post(
                  self->executor_,
                  std::bind(&WebsocketServer::WriteReply, self, status));
            });
      } catch (mjlib::base::system_error& se) {
        if (se.code() == mjlib::base::error::kJsonParse) {
          // This we will just log, and then continue on.
          log_.warn(fmt::format("Error reading JSON: {}\n{}", se.what(),
                                message));
          StartRead();
          return;
        }
        throw;
      }
    }

    void WriteReply(const QuadrupedControl::Status& status) {
      message_ = mjlib::base::Json5WriteArchive::Write(status);

      stream_.async_write(
          boost::asio::buffer(message_),
          std::bind(&WebsocketServer::HandleWrite, shared_from_this(),
                    pl::_1));
    }

    void HandleWrite(mjlib::base::error_code ec) {
      if (MaybeClose(ec)) { return; }
      mjlib::base::FailIf(ec);

      StartRead();
    }

    void Close() {
      stream_.async_close(
          websocket::close_code::normal,
          std::bind(&WebsocketServer::HandleClose, shared_from_this()));
    }

    void HandleClose() {
    }

    Impl* const parent_;
    WebServer::WebsocketStream stream_;
    boost::asio::executor executor_;

    base::LogRef log_ = base::GetLogInstance("WebControl");

    beast::flat_buffer buffer_;
    std::string message_;
    uint32_t count_ = 0;
  };

  boost::asio::executor executor_;
  QuadrupedControl* const quadruped_control_;
  Parameters parameters_;

  std::unique_ptr<WebServer> web_server_;
};

WebControl::WebControl(const boost::asio::executor& executor,
                       QuadrupedControl* quadruped_control)
    : impl_(std::make_unique<Impl>(executor, quadruped_control)) {}

WebControl::~WebControl() {}

void WebControl::AsyncStart(mjlib::io::ErrorCallback callback) {
  impl_->AsyncStart(std::move(callback));
}

clipp::group WebControl::program_options() {
  return mjlib::base::ClippArchive().Accept(&impl_->parameters_).release();
}

}
}
