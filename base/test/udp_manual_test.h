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

#include <boost/asio/deadline_timer.hpp>
#include <boost/format.hpp>

#include "common.h"
#include "component_archives.h"
#include "error_code.h"
#include "fail.h"
#include "logging.h"
#include "visitor.h"

#include "udp_socket.h"
#include "udp_data_link.h"

namespace mjmech {
namespace base {

class SocketTesterBase : boost::noncopyable {
 public:
  struct BaseParameters;

  SocketTesterBase(boost::asio::io_service& service,
                   const char* logname,
                   const BaseParameters& base_parameters)
      : service_(service),
        log_(base::GetLogInstance(logname)),
        base_parameters_(base_parameters),
        send_timer_(service_),
        stats_timer_(service_) {
  };

  virtual ~SocketTesterBase() {};

  struct BaseParameters {
    // print that many initial packets.
    int print_count = 10;
    // print stats every so often
    double stats_interval_s = 10.0;

    double send_interval_s = 0.0;
    int send_len = 1000;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(print_count));
      a->Visit(MJ_NVP(stats_interval_s));

      a->Visit(MJ_NVP(send_interval_s));
      a->Visit(MJ_NVP(send_len));
    }
  };

  virtual void Start() {
    if (base_parameters_.stats_interval_s > 0) {
      PrintStats(true);
    }
  }

  void PrintStats(bool timer_only=false) {
    stats_timer_.expires_from_now(
        ConvertSecondsToDuration(base_parameters_.stats_interval_s));
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


 protected:
  void HandleIncomingPacket(const std::string& data,
                            const std::string& sender) {
    stats_["rx_count"]++;
    stats_["rx_size"] += data.size();
    stats_["rx[" + sender + "]"]++;
    recv_count_++;
    if (recv_count_ > base_parameters_.print_count) {
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
                      << sender << ": [" << preview << "]";
    if (recv_count_ == base_parameters_.print_count) {
      log_.infoStream() << " suppressing further packets";
    }
  };

  virtual void SendData(const std::string& data) = 0;

  bool send_enabled() {
    return base_parameters_.send_interval_s > 0;
  };

  void StartSendTimer() {
    boost::posix_time::ptime now =
        boost::posix_time::microsec_clock::universal_time();

    // Try to maintain the requested send rate.
    if (last_send_time_.is_not_a_date_time()) {
      last_send_time_ = now;
    };
    last_send_time_ +=
        ConvertSecondsToDuration(base_parameters_.send_interval_s);
    if (last_send_time_ < now) {
      log_.warnStream()
          << "Cannot maintain send rate -- slipped by "
          << (now - last_send_time_).total_microseconds() << " us";
      last_send_time_ =
          now +
          ConvertSecondsToDuration(base_parameters_.send_interval_s);
    }

    send_timer_.expires_at(last_send_time_);
    send_timer_.async_wait([=](const boost::system::error_code& ec) {
        base::FailIf(ec);
        std::string data = (
            boost::format("%08d %s\n") % send_count_
            % std::string(base_parameters_.send_len - 10, 'z')).str();
        SendData(data);
        stats_["tx_count"]++;
        stats_["tx_size"] += data.size();

        send_count_++;

        StartSendTimer();
      });
  }

  boost::asio::io_service& service_;
  LogRef log_;

 private:
  const BaseParameters& base_parameters_;
  boost::asio::deadline_timer send_timer_;
  boost::asio::deadline_timer stats_timer_;
  boost::posix_time::ptime last_send_time_;
  int send_count_ = 0;
  int recv_count_ = 0;
  std::map<std::string, int> stats_;
};

class SocketTester: public SocketTesterBase {
 public:
  struct Parameters;

  SocketTester(boost::asio::io_service& service,
               const char* logname,
               const Parameters& parameters)
      : SocketTesterBase(service, logname, parameters.base),
        parameters_(parameters) {
  };

  struct Parameters {
    bool do_recv = false;
    std::string listen_addr;
    // Passed as-is to constructor
    bool server_mode = false;

    std::string send_addr;
    UdpSocket::Parameters opts;
    BaseParameters base;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(do_recv));
      a->Visit(MJ_NVP(listen_addr));
      a->Visit(MJ_NVP(server_mode));
      a->Visit(MJ_NVP(send_addr));
      a->Visit(MJ_NVP(opts));
      base.Serialize(a);
    }

    Parameters() {
      opts.default_port = 16429;
    }
  };

  virtual void Start() {
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
            std::string sender = boost::lexical_cast<std::string>(addr);
            HandleIncomingPacket(data, sender);
          });
    } else {
      log_.info("not enabling receiver");
    }

    if (parameters_.send_addr != "" && send_enabled()) {
      tx_endpoint_ = socket_->ParseSendEndpoint(parameters_.send_addr);
      log_.infoStream() << "Will send " << parameters_.base.send_len
                        << " bytes to " << *tx_endpoint_ << " every "
                        << parameters_.base.send_interval_s << " seconds";
      StartSendTimer();
    } else {
      log_.info("not enabling sender");
    }

    SocketTesterBase::Start();
  }

 protected:
  virtual void SendData(const std::string& data) {
    socket_->SendTo(data, *tx_endpoint_);
  }

 private:

  const Parameters& parameters_;
  std::unique_ptr<UdpSocket> socket_;
  boost::optional<UdpSocket::endpoint> tx_endpoint_;
};

class LinkTester: public SocketTesterBase {
 public:
  struct Parameters;

  LinkTester(boost::asio::io_service& service,
               const char* logname,
               const Parameters& parameters)
      : SocketTesterBase(service, logname, parameters.base),
        parameters_(parameters) {
  };

  struct Parameters {
    UdpDataLink::Parameters opts;
    BaseParameters base;

    template <typename Archive>
    void Serialize(Archive* a) {
      //a->Visit(MJ_NVP(opts));
      opts.Serialize(a);
      base.Serialize(a);
    }

    Parameters() {
      opts.socket_params.default_port = 16429;
    }
  };

  virtual void Start() {
    if (parameters_.opts.source == "" &&
        parameters_.opts.dest == "" &&
        !send_enabled()) {
      log_.info("not enabled");
      return;
    }
    link_.reset(
        new UdpDataLink(service_, log_,
                        parameters_.opts));

    link_->data_signal()->connect(
        [=](const std::string& data, const UdpDataLink::PeerInfo& cli) {
          HandleIncomingPacket(data, boost::lexical_cast<std::string>(cli.id));
        });

    if (send_enabled()) {
      log_.infoStream() << "Will send " << parameters_.base.send_len
                        << " bytes every "
                        << parameters_.base.send_interval_s << " seconds";
      StartSendTimer();
    } else {
      log_.info("not enabling sender");
    }

    SocketTesterBase::Start();
  }

 protected:
  virtual void SendData(const std::string& data) {
    link_->Send(data);
  }

 private:
  const Parameters& parameters_;
  std::unique_ptr<UdpDataLink> link_;
};


class UdpManualTest : boost::noncopyable {
 public:
  template <typename Context>
    UdpManualTest(Context& context)
        : service_(context.service),
          exit_timer_(service_) {
  }

  void AsyncStart(base::ErrorHandler handler) {
    testers_.push_back(
        TesterPtr(new SocketTester(service_, "socket1", parameters_.socket1)));
    testers_.push_back(
        TesterPtr(new SocketTester(service_, "socket2", parameters_.socket2)));
    testers_.push_back(
        TesterPtr(new LinkTester(service_, "link1", parameters_.link1)));
    testers_.push_back(
        TesterPtr(new LinkTester(service_, "link2", parameters_.link2)));

    for (auto& obj: testers_) {
      obj->Start();
    }

    if (parameters_.run_for_s > 0) {
      exit_timer_.expires_from_now(
          ConvertSecondsToDuration(parameters_.run_for_s));
      exit_timer_.async_wait([=](const boost::system::error_code& ec) {
          base::FailIf(ec);
          log_.info("run time expired, exiting");
          for (auto& obj: testers_) {
            obj->PrintStats();
          }
          service_.stop();
        });
    }

    log_.info("running");
    service_.post(std::bind(handler, base::ErrorCode()));
  }

  struct Parameters {
    SocketTester::Parameters socket1;
    SocketTester::Parameters socket2;
    LinkTester::Parameters link1;
    LinkTester::Parameters link2;

    // If >0, stop after that many seconds
    double run_for_s = 0;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(socket1));
      a->Visit(MJ_NVP(socket2));
      a->Visit(MJ_NVP(link1));
      a->Visit(MJ_NVP(link2));
      a->Visit(MJ_NVP(run_for_s));
    }
  };

  Parameters* parameters() { return &parameters_; }

 private:
  boost::asio::io_service& service_;
  Parameters parameters_;
  typedef std::shared_ptr<SocketTesterBase> TesterPtr;
  std::vector<TesterPtr> testers_;
  base::LogRef log_ = base::GetLogInstance("manual_test");
  boost::asio::deadline_timer exit_timer_;

};
}
}
