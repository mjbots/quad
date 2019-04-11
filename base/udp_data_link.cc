// Copyright 2015-2016 Mikhail Afanasyev.  All rights reserved.
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

#include "udp_data_link.h"

#include "mjlib/base/fail.h"

#include "base/now.h"

#include "common.h"
#include "stringify.h"

/*
Precise semantics:
- If one of 'source' or 'dest' contains an multicast address,
  while the other is an empty string, the empty field is produced
  by flipping last bit of both ip and port. Set the other value
  to something like ':' (empty host, empty port) to defeat this.

- Send() method sends to dest address
 - If it has no IP, we send to all active clients using unicast
  - But an empty dest will be replaced by flipped source address when
    source address is multicast.
 - If it contains unicast/multicast IP, we send it to this address
  - We try to send from the listening socket, so source ip/port will be
    determined by source parameter
  - We cannot do it when source is multicast, so in this case we
    will open another socket just for sending (its address will be
    determined by 'bind' parameter)

*/

namespace mjmech {
namespace base {

UdpDataLink::UdpDataLink(boost::asio::io_service& service,
                         LogRef& log,
                         const Parameters& params)
    : service_(service),
      periodic_timer_(service),
      params_(params),
      log_(log) {

  UdpSocket::ParseResult source_p = UdpSocket::ParseAddress(params_.source);
  UdpSocket::ParseResult dest_p = UdpSocket::ParseAddress(params_.dest);

  // Do automatic address fill-in.
  bool address_filled = false;
  if (params_.dest == "" && source_p.address &&
      UdpSocket::IsMulticastBroadcast(*source_p.address)) {
    // Address is simple
    dest_p.address = FlipBitInAddress(*source_p.address);
    if (dest_p.port) {
      source_p.port = *dest_p.port ^ 1;
    };
    address_filled = true;
  } else if (params_.source == "" && dest_p.address &&
             UdpSocket::IsMulticastBroadcast(*dest_p.address)) {
    source_p.address = FlipBitInAddress(*dest_p.address);
    if (source_p.port) {
      dest_p.port = *dest_p.port ^ 1;
    };
    address_filled = true;
  }
  // Using default port with auto-address is somewhat tricky. Make
  // port parity match address parity.
  if (address_filled && !dest_p.port) {
    BOOST_ASSERT(!source_p.port);
    int port_base = (params_.socket_params.default_port & ~1);
    source_p.port = port_base | GetLastBitOfAddress(*source_p.address);
    dest_p.port = port_base | GetLastBitOfAddress(*dest_p.address);
  }

  // We listen on default_port only if we have no destination address.
  bool server_mode = !dest_p.address;
  rx_socket_ = std::make_shared<UdpSocket>(
      service, log_, source_p, server_mode,
      params_.socket_params);

  auto logmsg = log_.noticeStream();

  if (rx_socket_->is_receiving_multicast()) {
    // We need a separate tx socket
    tx_socket_ = std::make_shared<UdpSocket>(
        service, base::GetSubLogger(log_, "tx"),
        "", false,  params_.socket_params);
    logmsg << "Receiving multicast " << rx_socket_->local_endpoint()
           << ", sending from " << tx_socket_->local_endpoint();
  } else {
    logmsg << "Receiving on " << rx_socket_->local_endpoint();
    tx_socket_ = rx_socket_;
  }

  rx_socket_->StartRead();
  rx_socket_->data_signal()->connect(
      std::bind(&UdpDataLink::HandleUdpPacket,
                this, std::placeholders::_1, std::placeholders::_2));

  if (dest_p.address) {
    tx_addr_ = UdpSocket::endpoint(
        *dest_p.address, dest_p.port.value_or(
            params_.socket_params.default_port));
    logmsg << ", sending to " << *tx_addr_;
  } else {
    logmsg << ", sending to recent clients";
  }

  HandlePeriodicTimer();
}

void UdpDataLink::Send(const std::string& data) {
  if (tx_addr_) {
    tx_socket_->SendTo(data, *tx_addr_);
  } else {
    for (const auto& val: peers_) {
      tx_socket_->SendTo(data, val.second.endpoint);
    }
  }
}

void UdpDataLink::SendTo(const std::string& data, const PeerInfo& peer) {
  tx_socket_->SendTo(data, peer.endpoint);
}


boost::asio::ip::address UdpDataLink::FlipBitInAddress(
    const boost::asio::ip::address& addr) {
  if (addr.is_v4()) {
    return boost::asio::ip::address_v4(
        addr.to_v4().to_ulong() ^ 1);
  }

  mjlib::base::AssertNotReached();
}

int UdpDataLink::GetLastBitOfAddress(const boost::asio::ip::address& addr) {
  if (addr.is_v4()) {
    return addr.to_v4().to_ulong() & 1;
  }

  mjlib::base::AssertNotReached();
}

void UdpDataLink::HandleUdpPacket(
    const std::string& data, const UdpSocket::endpoint& sender) {

  auto iter = peers_.find(sender);
  if (iter == peers_.end()) {
    last_peer_id_++;
    PeerInfo new_peer;
    new_peer.name = Stringify(sender);
    new_peer.endpoint = sender;
    new_peer.id = last_peer_id_;
    auto rv = peers_.insert(std::make_pair(sender, new_peer));
    iter = rv.first;
    BOOST_ASSERT(rv.second);
    peer_connected_signal_(iter->second);

    log_.noticeStream()
        << "Peer " << new_peer.id << " appeared, address " << new_peer.name;
  }
  iter->second.packet_count++;
  iter->second.last_rx_time = Now(service_);
  data_signal_(data, iter->second);
}

void UdpDataLink::HandlePeriodicTimer() {
  if (params_.peer_timeout_s <= 0) {
    return;
  }
  const boost::posix_time::ptime now = Now(service_);
  const boost::posix_time::time_duration timeout =
      ConvertSecondsToDuration(params_.peer_timeout_s);

  for (auto it = peers_.begin(); it != peers_.end(); ) {
    if (it->second.last_rx_time < (now - timeout)) {
      const PeerInfo& peer = it->second;
      log_.noticeStream()
          << "Peer " << peer.id << " gone after sending "
          << peer.packet_count <<  " packets, address was "
          << peer.name;
      peer_gone_signal_(peer);
      peers_.erase(it++);
    } else {
      ++it;
    }
  }

  // Allow timeout error of up to 20%.
  periodic_timer_.expires_from_now(timeout / 5.0);
  periodic_timer_.async_wait([=](const boost::system::error_code& ec) {
      mjlib::base::FailIf(ec);
      HandlePeriodicTimer();
    });
}


}
}
