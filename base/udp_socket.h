// Copyright 2015 Mikhail Afanasyev.  All rights reserved.
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

#include <string>

#include <boost/asio/ip/udp.hpp>
#include <boost/signals2/signal.hpp>
#include <boost/optional.hpp>

#include "logging.h"
#include "error_code.h"

namespace mjmech {
namespace base {

/*
  An UDP socket wrapper with support for multicast and other things.

  Entire multicast IP range is 224.0.0.0 to 239.255.255.255
  Of that, only 239.0.0.0/8 is 'local multicast address', so try
     to allocate addresses from this range.
*/
class UdpSocket {
 public:
  typedef boost::asio::ip::udp::endpoint endpoint;
  struct Parameters;

  // Creat an object. io_service is aliased, all other parameters
  // are copied. May raise.
  // @p listen_addr -- where to listen. If it contains multicast address,
  //    it subscribes to this address; else it overrides parameters.bind
  // @p server_mode -- what to do when neither listen_addr nor parameters.bind
  //    contain port number:
  //       true = use parameters.default_port.
  //       false = use zero.
  UdpSocket(boost::asio::io_service&, LogRef&,
            const std::string& listen_addr,
            bool server_mode,
            const Parameters&);
  ~UdpSocket();

  // Parse address into endpoint. Uses parameters.default_port if no
  // port is given.
  endpoint ParseSendEndpoint(const std::string& addr);
  // Send to a given address -- based on endpoint struct
  void SendTo(const std::string& data, const endpoint&);

  // asio's socket interface is designed to throttle down TCP connections
  // when the host is not keeping up. We have no such concerns, so we
  // expose more convinient API.
  typedef boost::signals2::signal<
    void(const std::string&, const endpoint&)> DataSignal;
  // Start reading loop.
  void StartRead();
  // Get the data signal. Reading loop must be started first.
  DataSignal* data_signal();

  // Return link's MTU.
  //int get_mtu();

  // Advanced parameters for UDP socket.
  struct Parameters {
    // Default port -- used when addr string does not contain one.
    int default_port = 0;

    // Local address to bind to.
    // May contain port which will then be used to blind client socket.
    // An empty string means 'listen on all interfaces', and 'send multicast
    // on default (= from routing tabl) interface'
    std::string bind;

    // Multicast TTL (ip(7), IPMULTICAST_TTL)
    int multicast_ttl = 1;

    // Network namespace to use. empty string for default.
    // Requires root to set.
    std::string netns;

    // Force MTU. When >0, disables kernel's fragmentation, sets DF flag,
    // and rejects packets bigger than this value.
    //int force_mtu = 0;

    // Disable fragmentation/routing of outgoing sockets
    bool dont_fragment = false;
    bool dont_route = false;

    // Warn and drop packets if more than that many are in flight.
    int max_tx_pending = 16;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(default_port));
      a->Visit(MJ_NVP(bind));
      a->Visit(MJ_NVP(multicast_ttl));
      a->Visit(MJ_NVP(netns));
      a->Visit(MJ_NVP(dont_fragment));
      a->Visit(MJ_NVP(dont_route));
      a->Visit(MJ_NVP(max_tx_pending));
    };
  };

  struct ParseResult {
    boost::optional<boost::asio::ip::address> address;
    boost::optional<int> port;
  };

  // Parse host:port string. May raise.
  static ParseResult ParseAddress(const std::string& address);
  static bool IsMulticastBroadcast(const boost::asio::ip::address&);

 private:
  typedef boost::asio::ip::udp udp;
  void StartNextRead();
  void HandleRead(ErrorCode, std::size_t size);
  void HandleWrite(std::shared_ptr<std::string>, ErrorCode);
  void PrepareSocket();
  void PrepareToSendTo(const endpoint&);

  boost::asio::io_service& service_;
  LogRef log_;
  Parameters parameters_;
  udp::socket socket_;
  char receive_buffer_[0x10000] = {};
  endpoint receive_endpoint_;

  bool reading_loop_running_ = false;
  bool v4_multicast_ready_ = false;
  bool v4_broadcast_ready_ = false;
  boost::asio::ip::address_v4 multicast_outbound_v4_;
  DataSignal data_signal_;
  int tx_pending_ = 0;
};

/*
  An UDP link wrapper.

Functionality:
  - options on client and server sides may be set identically
  - second socket for multicast
  - maintain/log client list
*/
class UdpDataLink {
 public:
  struct Parameters;

  UdpDataLink(bool server_mode,
              const Parameters& params);

  // Describes a 'link'. This structure may be set to identical
  // values (except for bind) on both server and clients, and it
  // should work,
  struct Parameters {
    // Server address -- where client sends, server receives
    // A string in IP, IP:PORT or :PORT format
    // - missing port means use default one
    // - address (on server):
    //     empty -> bind to all interfaces
    //     regular -> which IP to bind on
    //     multicast/broadcast -> listen on this address
    // - address (on client):
    //     empty -> learn from server's response (port ignored)
    //     other -> where to send. May be multicast/broadcast.
    std::string server;

    // Client address -- where server sends, client receives
    // A string in IP, IP:PORT or :PORT format
    // - missing port means use default one + 1 (unless address is empty)
    // - address (on client):
    //    empty -> bind a random port on all interfaces
    //    regular -> which IP to bind on
    //    multicast/broadcast -> listen on this address
    // - address (on server)
    //    empty -> send to address of last incoming packet (port ignored)
    //    other -> where to send. May be multicast/broadcast.
    std::string client;

    // If 'client' is set to this, it uses address of server + 1. This
    // is only useful if server is multicast.
    //static const char* kClientAddrNext = "+1";

    // If client address is specified, and there is a different one in the
    // packet source, do we want to use it?
    // (setting this to True on server means command responses will always go
    // over unicast)
    bool learn_address = false;

    // The client is considered 'gone' if it sent no packets for that many
    // seconds.
    double gone_timeout_s = 10.0;

    UdpSocket::Parameters socket_params;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(server));
      a->Visit(MJ_NVP(client));
      a->Visit(MJ_NVP(learn_address));
      socket_params.Serialize(a);
    }
  };

  UdpSocket::DataSignal* data_signal();

  void Send(const std::string& data);
  void SendTo(const std::string& data, const UdpSocket::endpoint&);
};

}
}
