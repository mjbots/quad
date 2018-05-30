// Copyright 2015-2018 Mikhail Afanasyev.  All rights reserved.
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

#include "udp_socket.h"

#include <boost/asio/ip/multicast.hpp>
#include <boost/lexical_cast.hpp>

#include "fail.h"

namespace mjmech {
namespace base {

/*
Cool UDP features:
 - IP_PKTINFO in ip(7) gives more addressing information
 - IP_RECVERR in ip(7) can give per-client errors reliably
 - SO_BINDTODEVICE in socket(7) is yet another way to bind to device (as root).
 - SO_TIMESTAMP gives exact packet timestamp
 - query MTU with IP_MTU
 - possibly IP_PMTUDISC_PROBE in ip(7) can override system MTU?
 */

namespace ip = boost::asio::ip;

UdpSocket::UdpSocket(boost::asio::io_service& service,
                     LogRef& log,
                     const std::string& listen_addr,
                     bool server_mode,
                     const Parameters& parameters)
    : UdpSocket(service, log, ParseAddress(listen_addr),
                server_mode, parameters) {
};


UdpSocket::UdpSocket(boost::asio::io_service& service,
                     LogRef& log,
                     const ParseResult& listen_p,
                     bool server_mode,
                     const Parameters& parameters)
    : log_(log),
      parameters_(parameters),
      socket_(service) {

  PrepareSocket();

  ParseResult bind_p = ParseAddress(parameters.bind);

  endpoint bind_to;

  if (!listen_p.address ||
      !IsMulticastBroadcast(*listen_p.address)) {
    // We are unicast
    if (listen_p.address && bind_p.address &&
        (*listen_p.address != *bind_p.address)) {
      log_.errorStream()
          << "Two distinct bind addresses specified: "
          << *listen_p.address << " and " << *bind_p.address
          << ", choosing first one";
    }
    bind_to = endpoint(
        listen_p.address.get_value_or(
            bind_p.address.get_value_or(ip::address_v4::any())),
        listen_p.port.get_value_or(
            bind_p.port.get_value_or(
                server_mode ? parameters_.default_port : 0)));
  } else if (listen_p.address->is_v4()) {
    // We are v4 multicast / broadcast
    bind_to = endpoint(
        bind_p.address.get_value_or(ip::address_v4::any()),
        listen_p.port.get_value_or(bind_p.port.get_value_or(
                                       parameters_.default_port)));

    ip::address_v4 addr = listen_p.address->to_v4();
    ip::address_v4 interface = bind_to.address().to_v4();

    ip::multicast::join_group option(addr, interface);
    log_.debugStream() << "Joining multicast group " << addr
                       << " on interface " << interface;
    socket_.set_option(option);
    if (bind_to.port() != 0) {
      socket_.set_option(boost::asio::socket_base::reuse_address(true));
    }
    rx_multicast_addr_ = endpoint(addr, bind_to.port());
  } else {
    base::Fail("address family not supported");
  }

  log_.debugStream() << "Binding to " << bind_to;
  socket_.bind(bind_to);
  if (bind_to.address().is_v4()) {
    multicast_outbound_v4_ = bind_to.address().to_v4();
  }
  endpoint bound_to = socket_.local_endpoint();
  if (bound_to != bind_to) {
    log_.debugStream() << "Actually bound to " << bound_to;

    if  (rx_multicast_addr_) {
      rx_multicast_addr_->port(bound_to.port());
    }
  }
}

UdpSocket::~UdpSocket() {}

void UdpSocket::PrepareSocket() {
  socket_.open(udp::v4());
  BOOST_ASSERT(parameters_.netns == "");
  if (parameters_.dont_fragment) {
    log_.debug("Setting do-not-fragment option");
    // Despite the strange parameter name, this sets DF bit in packets. See
    // ip(7).
    int fd = socket_.native_handle();
    int val = IP_PMTUDISC_DO;
    int result = ::setsockopt(fd, IPPROTO_IP, IP_MTU_DISCOVER,
                              &val, sizeof(val));
    if (result < 0) {
      throw base::SystemError(errno, boost::system::generic_category());
    }
  }

  if (parameters_.dont_route) {
    log_.debug("Setting do-not-route option");
    socket_.set_option(boost::asio::socket_base::do_not_route(true));
  }
}


UdpSocket::endpoint UdpSocket::ParseSendEndpoint(const std::string& addr) {
  ParseResult res = ParseAddress(addr);
  return endpoint(
        res.address.get_value_or(ip::address_v4::any()),
        res.port.get_value_or(parameters_.default_port));
}

void UdpSocket::PrepareToSendTo(const endpoint& endpoint) {
  const ip::address& address = endpoint.address();
  if (!IsMulticastBroadcast(address)) {
    // unicast. Can just send.
    return;
  }

  if (address.is_v4() && address.to_v4().is_multicast()) {
    if (v4_multicast_ready_) {
      return;
    }
    // Enable loopback -- otherwise we may not see some packets when there are
    // multiple processes on one host.
    log_.debugStream() << "Enabling V4 multicast TX to "
                       << endpoint << " with " << parameters_.multicast_ttl
                       << " hops via " << multicast_outbound_v4_;
    socket_.set_option(ip::multicast::enable_loopback(true));
    socket_.set_option(ip::multicast::hops(parameters_.multicast_ttl));
    if (multicast_outbound_v4_ != ip::address_v4()) {
      socket_.set_option(ip::multicast::outbound_interface(
                             multicast_outbound_v4_));
    }
    v4_multicast_ready_ = true;
    return;
  }

  if (address.is_v4() && address.to_v4() == ip::address_v4::broadcast()) {
    if (v4_broadcast_ready_) {
      return;
    }

    log_.debugStream() << "Enabling V4 broadcast TX to " << endpoint;
    socket_.set_option(boost::asio::socket_base::broadcast(true));
    v4_broadcast_ready_ = true;
    return;
  }

  base::Fail("Unknown multicast / broadcast type");
}

void UdpSocket::SendTo(const std::string& data, const endpoint& endpoint) {
  if (tx_pending_ > parameters_.max_tx_pending) {
    log_.warnStream() << "Cannot send " << data.size() << " bytes to "
                      << endpoint << ": too many pending packets("
                      << tx_pending_ << ")";
    return;
  }

  PrepareToSendTo(endpoint);
  tx_pending_++;
  std::shared_ptr<std::string> shared_data(new std::string(data));
  socket_.async_send_to(
      boost::asio::buffer(*shared_data),
      endpoint,
      std::bind(&UdpSocket::HandleWrite, this, shared_data,
                std::placeholders::_1));
}

void UdpSocket::HandleWrite(std::shared_ptr<std::string>,
                            ErrorCode ec) {
  FailIf(ec);
  tx_pending_--;
}


UdpSocket::DataSignal* UdpSocket::data_signal() {
  BOOST_ASSERT(reading_loop_running_);
  return &data_signal_;
}

void UdpSocket::StartRead() {
  BOOST_ASSERT(!reading_loop_running_);
  reading_loop_running_ = true;
  StartNextRead();
}

void UdpSocket::StartNextRead() {
  socket_.async_receive_from(
      boost::asio::buffer(receive_buffer_),
      receive_endpoint_,
      std::bind(&UdpSocket::HandleRead, this,
                std::placeholders::_1,
                std::placeholders::_2));
}

bool UdpSocket::is_receiving_multicast() const {
  return !!rx_multicast_addr_;
}

UdpSocket::endpoint UdpSocket::local_endpoint() const {
  if (rx_multicast_addr_) {
    return *rx_multicast_addr_;
  };
  return socket_.local_endpoint();
}

void UdpSocket::HandleRead(ErrorCode ec, std::size_t size) {
  FailIf(ec);

  std::string data(receive_buffer_, size);
  udp::endpoint from = receive_endpoint_;

  StartNextRead();

  data_signal_(data, from);
}


UdpSocket::ParseResult UdpSocket::ParseAddress(const std::string& address) {
  ParseResult result;
  std::string host = address;

  // TODO theamk: use resolver? but this may block for long periods of time
  const size_t colon = host.find_last_of(':');
  if (colon != std::string::npos) {
    std::string port_str =  host.substr(colon + 1);
    host = host.substr(0, colon);

    if (port_str != "") {
      try {
        result.port = boost::lexical_cast<int>(port_str);
        if (result.port < 0 || result.port > 0xffff) {
          throw boost::bad_lexical_cast();
        }
      } catch (boost::bad_lexical_cast) {
        throw std::invalid_argument(
            "Cannot parse port '" + port_str + "' from address '" +
            address + "'");
      }
    }
  }

  if (host != "") {
    try {
      result.address = ip::address::from_string(host);
    } catch (boost::system::system_error& e) {
      throw std::invalid_argument(
          "Cannot parse IP '" + host + "' from address '" +
          address + "': " + e.what());
    }
  }
  return result;
}


bool UdpSocket::IsMulticastBroadcast(const ip::address& addr) {
  if (addr.is_v4()) {
    ip::address_v4 addr4 = addr.to_v4();
    return addr4.is_multicast() || addr4 == ip::address_v4::broadcast();
  } else if (addr.is_v6()) {
    ip::address_v6 addr6 = addr.to_v6();
    return addr6.is_multicast();
  } else {
    base::Fail("invalid address type");
  }
}


}
}
