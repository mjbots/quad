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

#pragma once

#include <memory>

#include <clipp/clipp.h>

#include <boost/asio/executor.hpp>
#include <boost/beast/websocket.hpp>
#include <boost/filesystem.hpp>

#include "mjlib/base/clipp_archive.h"
#include "mjlib/base/fail.h"
#include "mjlib/base/json5_read_archive.h"
#include "mjlib/base/json5_write_archive.h"

#include "base/logging.h"

#include "mech/quadruped_control.h"
#include "mech/web_server.h"

namespace mjmech {
namespace mech {

/// Exposes an embedded web server with a command and control UI.
template <typename CommandClass, typename StatusClass>
class WebControl {
 public:
  struct Options {
    std::string asset_path;
  };

  using SetCommand = std::function<void (const CommandClass&)>;
  using GetStatus = std::function<StatusClass ()>;

  WebControl(const boost::asio::executor& executor,
             SetCommand set_command,
             GetStatus get_status,
             const Options& options)
      : executor_(executor),
        set_command_(set_command),
        get_status_(get_status),
        options_(options) {}

  ~WebControl() {}

  struct Parameters {
    int port = 4778;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(port));
    }
  };

  clipp::group program_options() {
    return mjlib::base::ClippArchive().Accept(&parameters_).release();
  }

  void AsyncStart(mjlib::io::ErrorCallback callback) {
    WebServer::Options server_options;

    server_options.port = parameters_.port;

    server_options.document_roots.push_back(
        {std::string("/"), FindAssetPath()});

    server_options.websocket_handlers.push_back(
        {"/control",
         std::bind(&WebControl::HandleControlWebsocket, this,
                   std::placeholders::_1)});

    std::cout << "Starting web server\n";
    web_server_ = std::make_unique<WebServer>(executor_, server_options);

    web_server_->AsyncStart(std::move(callback));
  }

 private:
  void HandleControlWebsocket(WebServer::WebsocketStream stream) {
    // This stream is running on a different executor, thus we need to
    // keep it segregated.
    std::make_shared<WebsocketServer>(this, std::move(stream))->Start();
  }

  class WebsocketServer : public std::enable_shared_from_this<WebsocketServer> {
   public:
    WebsocketServer(WebControl* parent, WebServer::WebsocketStream stream)
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
          std::bind(&WebsocketServer::HandleRead, this->shared_from_this(),
                    std::placeholders::_1));
    }

    bool MaybeClose(const mjlib::base::error_code& ec) {
      if (ec == boost::beast::error::timeout ||
          ec == boost::asio::error::broken_pipe ||
          ec == boost::asio::error::not_connected ||
          ec == boost::asio::error::connection_reset ||
          ec == boost::beast::websocket::error::closed) {
        log_.warn(fmt::format("Closing websocket connection: {}", ec.message()));
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
        using JsonRead = mjlib::base::Json5ReadArchive;
        const auto command =
            JsonRead::Read<WebCommand>(
                istr,
                JsonRead::Options().set_permissive_nan(true));

        boost::asio::post(
            parent_->executor_,
            [self = this->shared_from_this(), command]() {
              if (command.command) {
                self->parent_->set_command_(*command.command);
              }
              const auto status = self->parent_->get_status_();

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

    void WriteReply(const StatusClass& status) {
      using JsonWrite = mjlib::base::Json5WriteArchive;
      message_ = JsonWrite::Write(
          status, JsonWrite::Options().set_standard(true));

      stream_.async_write(
          boost::asio::buffer(message_),
          std::bind(&WebsocketServer::HandleWrite, this->shared_from_this(),
                    std::placeholders::_1));
    }

    void HandleWrite(mjlib::base::error_code ec) {
      if (MaybeClose(ec)) { return; }
      mjlib::base::FailIf(ec);

      StartRead();
    }

    void Close() {
      stream_.async_close(
          boost::beast::websocket::close_code::normal,
          std::bind(&WebsocketServer::HandleClose, this->shared_from_this()));
    }

    void HandleClose() {
    }

    WebControl* const parent_;
    WebServer::WebsocketStream stream_;
    boost::asio::executor executor_;

    base::LogRef log_ = base::GetLogInstance("WebControl");

    boost::beast::flat_buffer buffer_;
    std::string message_;
    uint32_t count_ = 0;
  };

  std::string FindAssetPath() {
    namespace fs = boost::filesystem;

    fs::path start = fs::canonical("/proc/self/exe").remove_filename();

    for (const char* to_try : {
            ".",
            "quadruped.runfiles/com_github_mjbots_mech/mech",
            "simulator.runfiles/com_github_mjbots_mech/mech",
            }) {
      fs::path this_path = start / to_try / options_.asset_path;
      if (fs::exists(this_path)) {
        return this_path.native();
      }
    }

    // Hmmph, we couldn't find it.
    mjlib::base::Fail("Could not locate WebControl assets: " + start.native());
  }

  struct WebCommand {
    std::optional<CommandClass> command;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(command));
    }
  };

  boost::asio::executor executor_;
  SetCommand set_command_;
  GetStatus get_status_;

  const Options options_;

  Parameters parameters_;

  std::unique_ptr<WebServer> web_server_;
};

}
}
