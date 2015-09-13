// Copyright 2015 Josh Pieper, jjp@pobox.com.  All rights reserved.
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

#include "telemetry_remote_debug_server.h"

#include <boost/property_tree/json_parser.hpp>

#include "fail.h"

namespace mjmech {
namespace base {
class TelemetryRemoteDebugServer::Impl : boost::noncopyable {
 public:
  Impl(boost::asio::io_service& service)
      : service_(service),
        socket_(service) {}

  void StartRead() {
    socket_.async_receive_from(
        boost::asio::buffer(receive_buffer_),
        receive_endpoint_,
        std::bind(&Impl::HandleRead, this,
                  std::placeholders::_1,
                  std::placeholders::_2));
  }

  void HandleRead(ErrorCode ec, std::size_t size) {
    FailIf(ec);

    std::string data(receive_buffer_, size);
    udp::endpoint from = receive_endpoint_;

    StartRead();

    HandleData(data, from);
  }

  void HandleData(const std::string& data, const udp::endpoint& from) {
    // Our messages should come in as JSON requests.
    boost::property_tree::ptree tree;
    try {
      std::istringstream inf(data);
      boost::property_tree::read_json(inf, tree);
    } catch (std::exception& e) {
      std::cerr << "error reading remote debug command: " << e.what() << "\n";
      return;
    }

    HandleMessage(tree, from);
  }

  void HandleMessage(const boost::property_tree::ptree& tree,
                     const udp::endpoint& from) {
    std::string command = tree.get<std::string>("command", "");
    if (command == "enumerate") {
      DoEnumerate(from);
    } else if (command == "get") {
      DoGet(tree, from);
    } else {
      std::cerr << "unknown remote debug command: '" << command << "'\n";
    }
  }

  struct EnumerateResponse {
    std::string type = "enumerate";
    std::vector<std::string> names;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(type));
      a->Visit(MJ_NVP(names));
    }
  };

  void DoEnumerate(const udp::endpoint& from) {
    EnumerateResponse response;
    for (const auto& pair: handlers_) { response.names.push_back(pair.first); }

    SendData(JsonWriteArchive::Write(&response), from);
  }

  void SendData(const std::string& data,
                const udp::endpoint& endpoint) {
    std::shared_ptr<std::string> shared_data(new std::string(data));
    socket_.async_send_to(boost::asio::buffer(*shared_data),
                          endpoint,
                          std::bind(&Impl::HandleWrite, this, shared_data,
                                    std::placeholders::_1));
  }

  void DoGet(const boost::property_tree::ptree& tree,
             const udp::endpoint& from) {
    for (const boost::property_tree::ptree::value_type& v:
             tree.get_child("names")) {
      std::string name = v.second.data();
      auto it = handlers_.find(name);
      if (it == handlers_.end()) {
        std::cerr << "request for unknown name: '" + name + "'\n";
        continue;
      }

      it->second->Respond(from);
    }
  }

  void HandleWrite(std::shared_ptr<std::string>,
                   ErrorCode ec) {
    FailIf(ec);
  }

  boost::asio::io_service& service_;
  Parameters parameters_;
  udp::socket socket_;
  char receive_buffer_[3000] = {};
  udp::endpoint receive_endpoint_;

  std::map<std::string, std::unique_ptr<Handler> > handlers_;
};

TelemetryRemoteDebugServer::TelemetryRemoteDebugServer(
    boost::asio::io_service& service)
    : impl_(new Impl(service)) {}

TelemetryRemoteDebugServer::~TelemetryRemoteDebugServer() {}

TelemetryRemoteDebugServer::Parameters*
TelemetryRemoteDebugServer::parameters() {
  return &impl_->parameters_;
}

void TelemetryRemoteDebugServer::AsyncStart(ErrorHandler handler) {
  impl_->socket_.open(udp::v4());
  udp::endpoint endpoint(udp::v4(), impl_->parameters_.port);
  impl_->socket_.bind(endpoint);
  impl_->StartRead();

  impl_->service_.post(std::bind(handler, ErrorCode()));
}

void TelemetryRemoteDebugServer::RegisterHandler(
    const std::string& name,
    std::unique_ptr<Handler> handler) {
  impl_->handlers_.insert(std::make_pair(name, std::move(handler)));
}

void TelemetryRemoteDebugServer::SendResponse(
    const std::string& data,
    const udp::endpoint& endpoint) {
  impl_->SendData(data, endpoint);
}
}
}
