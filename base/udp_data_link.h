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

#pragma once

#include <unordered_map>

#include "mjlib/io/deadline_timer.h"

#include "udp_socket.h"

namespace mjmech {
namespace base {

/*
A higher level wrapper around UDP link.
*/
class UdpDataLink : boost::noncopyable {
 public:
  struct Parameters;
  // @p log will be used for message logging
  // @p params -- parameters to use, will be copied
  UdpDataLink(const boost::asio::executor&, LogRef& log,
              const Parameters& params);
  ~UdpDataLink() {};

  struct Parameters {
    // Source address (IP, IP:PORT, or :PORT)
    // We listen on source address.
    //  If it has no IP, we do regular listen (on all interfaces)
    //   (but remember automatic fill of empty multicast addresses)
    //  If it contains unicast IP, we bind to interface with this address.
    //  If it contains multicast IP, we subscribe to this IP
    //    (and use 'bind' parameter to determine the interface)
    std::string source;

    // Destination address (IP, IP:PORT, or :PORT)
    //  If IP part is empty, we send to all active clients using unicast
    //   (but remember automatic fill of empty multicast addresses)
    //  If it contains unicast/multicast IP, we send it to this address.
    //   (We try to send from the listening socket, so source ip/port will be
    //    determined by source parameter, but when the source is multicast,
    //    we open a new socket just for sending)
    std::string dest;

    // Multicast note: if one of (source, dest) fields is an empty string,
    // while other field is a multicast address, then the empty string will be
    // replaced with a flipped-bit multicast address/port. This way, setting
    // only one of two fields to multicast address will work properly to
    // establish two-way link. To disable this (and use multicast only in a
    // single direction), set the other field to ":" (empty IP and port)

    // The peer is considered 'gone' if it sent no packets for that many
    // seconds.
    double peer_timeout_s = 10.0;

    // Link's MTU -- maximum size of packet
    // TODO mafanasyev: get from udp socket when this is zero.
    int link_mtu = 1400;

    UdpSocket::Parameters socket_params;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(source));
      a->Visit(MJ_NVP(dest));
      a->Visit(MJ_NVP(peer_timeout_s));
      socket_params.Serialize(a);
    }
  };

  struct PeerInfo {
    // name is endpoint stringification
    std::string name;
    UdpSocket::endpoint endpoint;
    // Sequential, positive peer index which is never reused.
    int id = 0;
    int packet_count = 0;
    boost::posix_time::ptime last_rx_time;
  };

  typedef boost::signals2::signal<
    void(const std::string&, const PeerInfo&)> DataSignal;
  DataSignal* data_signal() { return &data_signal_; }

  typedef boost::signals2::signal<void(const PeerInfo&)> PeerSignal;
  PeerSignal* peer_connected_signal() { return &peer_connected_signal_; }
  PeerSignal* peer_gone_signal() { return &peer_gone_signal_; };

  // Return maximum payload size.
  int get_max_data_size() const { return params_.link_mtu - kUdpV4HeaderSize; }

  // Send to all clients (see description of 'dest')
  void Send(const std::string& data);

  // This method always sends via unicast, to client address from the original
  // packet. There is currently no way for client to tell that it wants to
  // receive responses over multicast.
  void SendTo(const std::string& data, const PeerInfo&);

  // Flip last bit in the IP address
  static boost::asio::ip::address FlipBitInAddress(
      const boost::asio::ip::address&);
  // Return last bit of IP address (0 or 1)
  static int GetLastBitOfAddress(const boost::asio::ip::address&);

 private:
  void HandleUdpPacket(const std::string&, const UdpSocket::endpoint&);
  void HandlePeriodicTimer();

  const int kUdpV4HeaderSize = 28; // 20 byte IPv4 + 8 bytes UDP

  boost::asio::executor executor_;
  mjlib::io::DeadlineTimer periodic_timer_;
  const Parameters params_;
  LogRef log_;
  std::unordered_map<UdpSocket::endpoint, PeerInfo> peers_;
  DataSignal data_signal_;
  PeerSignal peer_connected_signal_;
  PeerSignal peer_gone_signal_;
  int last_peer_id_ = 0;

  std::optional<UdpSocket::endpoint> tx_addr_;

  std::shared_ptr<UdpSocket> rx_socket_;
  std::shared_ptr<UdpSocket> tx_socket_;
};

}
}
