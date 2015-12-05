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

#include "udp_socket.h"

#include <boost/asio/deadline_timer.hpp>
#include <boost/format.hpp>

#include "common.h"
#include "component_archives.h"
#include "error_code.h"
#include "fail.h"
#include "logging.h"
#include "visitor.h"

namespace mjmech {
namespace base {

class SocketTester: boost::noncopyable {
 public:
  struct Parameters;

  SocketTester(boost::asio::io_service& service,
               const char* logname,
               const Parameters& parameters)
      : service_(service),
        log_(base::GetLogInstance(logname)),
        parameters_(parameters),
        send_timer_(service_),
        stats_timer_(service_) {
  };

  struct Parameters {
    bool do_recv = false;
    std::string listen_addr;
    // Passed as-is to constructor
    bool server_mode = false;

    // print that many initial packets.
    int print_count = 10;
    // print stats every so often
    double stats_interval_s = 10.0;

    std::string send_addr;
    double send_interval_s = 1.0;
    int send_len = 1000;
    UdpSocket::Parameters opts;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(do_recv));
      a->Visit(MJ_NVP(listen_addr));
      a->Visit(MJ_NVP(server_mode));

      a->Visit(MJ_NVP(print_count));
      a->Visit(MJ_NVP(stats_interval_s));

      a->Visit(MJ_NVP(send_addr));
      a->Visit(MJ_NVP(send_interval_s));
      a->Visit(MJ_NVP(send_len));
      a->Visit(MJ_NVP(opts));
    }

    Parameters() {
      opts.default_port = 16429;
    }

  };

  void Start() {
    if (!parameters_.do_recv && parameters_.send_addr == "") {
      log_.info("not enabled");
      return;
    }
    socket_.reset(
        new UdpSocket(service_, log_,
                      parameters_.listen_addr,
                      parameters_.server_mode,
                      parameters_.opts));

    if (parameters_.do_recv) {
      socket_->StartRead();
      socket_->data_signal()->connect(
          [=](const std::string& data, const UdpSocket::endpoint& addr) {

            stats_["rx_count"]++;
            stats_["rx_size"] += data.size();
            stats_["rx[" + boost::lexical_cast<std::string>(addr) + "]"]++;
            recv_count_++;
            if (recv_count_ > parameters_.print_count) {
              return;
            }

            std::string preview = data.substr(0, 16);
            for (size_t i=0; i<preview.length(); i++) {
              if (preview[i] < 32 || preview[i] > 127) {
                preview[i] = '?';
              }
            }
            log_.infoStream() << "got #" << recv_count_
                              << ": " << data.size() << " bytes from "
                              << addr << ": [" << preview << "]";
            if (recv_count_ == parameters_.print_count) {
              log_.infoStream() << " suppressing further packets";
            }
          });
    } else {
      log_.info("not enabling receiver");
    }

    if (parameters_.send_addr != "" && parameters_.send_interval_s > 0) {
      tx_endpoint_ = socket_->ParseSendEndpoint(parameters_.send_addr);
      log_.infoStream() << "Will send " << parameters_.send_len
                        << " bytes to " << *tx_endpoint_ << " every "
                        << parameters_.send_interval_s << " seconds";
      StartSendTimer();
    }

    if (parameters_.stats_interval_s > 0) {
      PrintStats(true);
    }
  }

  void PrintStats(bool timer_only=false) {
    stats_timer_.expires_from_now(
        ConvertSecondsToDuration(parameters_.stats_interval_s));
    stats_timer_.async_wait([=](const boost::system::error_code& ec) {
        base::FailIf(ec);
        PrintStats(false);
      });
    if (timer_only) { return; }
    auto stream = log_.infoStream();
    stream << "stats:";
    for (const auto& val: stats_) {
      stream << " " << val.first << "=" << val.second;
    }
    if (!stats_.size()) {
      stream << " (no data)";
    }
    stats_.clear();
  }


 private:

  void StartSendTimer() {
    send_timer_.expires_from_now(
        ConvertSecondsToDuration(parameters_.send_interval_s));
    send_timer_.async_wait([=](const boost::system::error_code& ec) {
        base::FailIf(ec);
        std::string data = (
            boost::format("%08d %s\n") % send_count_
            % std::string(parameters_.send_len - 10, 'z')).str();
        socket_->SendTo(data, *tx_endpoint_);

        stats_["tx_count"]++;
        stats_["tx_size"] += data.size();

        send_count_++;

        StartSendTimer();
      });
  }

  boost::asio::io_service& service_;
  LogRef log_;
  const Parameters& parameters_;
  std::unique_ptr<UdpSocket> socket_;
  boost::optional<UdpSocket::endpoint> tx_endpoint_;
  boost::asio::deadline_timer send_timer_;
  boost::asio::deadline_timer stats_timer_;
  int send_count_ = 0;
  int recv_count_ = 0;
  std::map<std::string, int> stats_;
};

class UdpManualTest : boost::noncopyable {
 public:
  template <typename Context>
    UdpManualTest(Context& context)
        : service_(context.service),
          exit_timer_(service_) {
  }

  void AsyncStart(base::ErrorHandler handler) {
    socket1_.reset(
        new SocketTester(service_, "socket1", parameters_.socket1));
    socket2_.reset(
        new SocketTester(service_, "socket2", parameters_.socket2));

    socket1_->Start();
    socket2_->Start();

    if (parameters_.run_for_s > 0) {
      exit_timer_.expires_from_now(
          ConvertSecondsToDuration(parameters_.run_for_s));
      exit_timer_.async_wait([=](const boost::system::error_code& ec) {
          base::FailIf(ec);
          log_.info("run time expired, exiting");
          socket1_->PrintStats();
          socket2_->PrintStats();
          service_.stop();
        });
    }

    log_.info("running");
    service_.post(std::bind(handler, base::ErrorCode()));
  }

  struct Parameters {
    SocketTester::Parameters socket1;
    SocketTester::Parameters socket2;

    // If >0, stop after that many seconds
    double run_for_s = 0;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(socket1));
      a->Visit(MJ_NVP(socket2));
      a->Visit(MJ_NVP(run_for_s));
    }
  };

  Parameters* parameters() { return &parameters_; }

 private:
  boost::asio::io_service& service_;
  Parameters parameters_;
  std::unique_ptr<SocketTester> socket1_;
  std::unique_ptr<SocketTester> socket2_;
  base::LogRef log_ = base::GetLogInstance("manual_test");
  boost::asio::deadline_timer exit_timer_;

};
}
}
