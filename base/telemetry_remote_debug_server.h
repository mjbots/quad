// Copyright 2015-2019 Josh Pieper, jjp@pobox.com.  All rights reserved.
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

#include <boost/asio/io_context.hpp>
#include <boost/asio/ip/udp.hpp>
#include <boost/signals2/signal.hpp>

#include "mjlib/base/json5_write_archive.h"
#include "mjlib/io/async_types.h"

namespace mjmech {
namespace base {

class TelemetryRemoteDebugServer : boost::noncopyable {
 public:
  typedef boost::asio::ip::udp udp;

  TelemetryRemoteDebugServer(const boost::asio::executor&);
  ~TelemetryRemoteDebugServer();

  struct Parameters {
    int port = 13380;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(port));
    }
  };

  Parameters* parameters();

  void AsyncStart(mjlib::io::ErrorCallback handler);

  template <typename T>
  void Register(const std::string& name,
                boost::signals2::signal<void (const T*)>* signal) {
    std::unique_ptr<Handler> handler(
        new ConcreteHandler<T>(this, name, signal));
    RegisterHandler(name, std::move(handler));
  }

 private:
  class Handler : boost::noncopyable {
   public:
    virtual ~Handler() {}

    /// Send now, or in the future, the contents of this registration
    /// to the given UDP endpoint.  If a request is still outstanding,
    /// this will be a noop.
    virtual void Respond(const udp::endpoint&) = 0;
  };

  template <typename T>
  struct Response {
    std::string type;

    struct Reply {
      template <typename Archive>
      void Serialize(Archive* a) {
        a->Visit(mjlib::base::MakeNameValuePair(parent->value, parent->name.c_str()));
      }

      Response* parent;
    } reply;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(type));
      a->Visit(MJ_NVP(reply));
    }

    Response(T* value, const std::string& name) {
      // NOTE jpieper: We don't use initializer lists and
      // appropriately const members in order to work around
      // https://gcc.gnu.org/bugzilla/show_bug.cgi?id=58187
      reply.parent = this;
      this->value = value;
      this->name = name;

      type = "reply";
    }

    T* value;
    std::string name;
  };

  /// This is the handler for classes which are default and
  /// copy-constructable.
  ///
  /// TODO jpieper: Make an implementation for classes which do not
  /// have these properties, which just waits for a signal to come in
  /// before responding.
  template <typename T>
  class ConcreteHandler : public Handler {
   public:
    ConcreteHandler(TelemetryRemoteDebugServer* parent,
                    const std::string& name,
                    boost::signals2::signal<void (const T*)>* signal)
        : parent_(parent),
          name_(name) {
      signal->connect(std::bind(&ConcreteHandler::HandleData, this,
                                std::placeholders::_1));
    }
    virtual ~ConcreteHandler() {}


    virtual void Respond(const udp::endpoint& endpoint) {
      Response<T> response(&data_, name_);
      parent_->SendResponse(mjlib::base::Json5WriteArchive::Write(response),
                            endpoint);
    }

    void HandleData(const T* data) { data_ = *data; }

    TelemetryRemoteDebugServer* const parent_;
    const std::string name_;
    T data_;
  };

  void RegisterHandler(const std::string&, std::unique_ptr<Handler>);

  void SendResponse(const std::string& data,
                    const udp::endpoint&);

  class Impl;
  std::unique_ptr<Impl> impl_;
};
}
}
