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

#include <boost/asio/executor.hpp>
#include <boost/program_options.hpp>

#include <fmt/format.h>

#include "mjlib/base/fail.h"
#include "mjlib/base/program_options_archive.h"
#include "mjlib/base/time_conversions.h"
#include "mjlib/io/now.h"
#include "mjlib/io/repeating_timer.h"
#include "mjlib/multiplex/threaded_client.h"

namespace base = mjlib::base;
namespace io = mjlib::io;
namespace mp = mjlib::multiplex;
namespace po = boost::program_options;
namespace pl = std::placeholders;

namespace {

constexpr int kSetupRegister = 0;  // mode
constexpr int kSetupValue = 0;  // kStopped
constexpr int kNonceRegister = 0x20;  // kPositionCommand

struct Options {
  double rate_s = 0.1;
  std::string port;
};

class CommandRunner {
 public:
  CommandRunner(const boost::asio::executor& executor,
                const Options& options)
      : executor_(executor),
        options_(options) {
    client_.emplace(
        executor_,
        [&]() {
          mp::ThreadedClient::Options options;
          options.port = options_.port;
          return options;
        }());
  }

  void MakeRequest() {
    nonce_count_ = (nonce_count_ + 1) % 100;

    reply_.replies.clear();

    request_.requests.resize(kTargets.size());
    for (size_t i = 0; i < kTargets.size(); i++) {
      const int id = kTargets[i];

      mp::ThreadedClient::SingleRequest request;
      request.id = id;
      request.request.WriteSingle(kSetupRegister, kSetupValue);

      // The position command needs to set 3 values.  For now, we'll
      // send 3 int16's.
      auto i16 = [](auto v) { return static_cast<int16_t>(v); };
      request.request.WriteMultiple(
          kNonceRegister, {i16(nonce_count_), i16(0), i16(0)});

      request.request.ReadMultiple(kNonceRegister, 3, 1);
      // And read 3 i8 regs, like voltage and fault
      request.request.ReadMultiple(0x000, 1, 0);

      request_.requests[i] = request;
    }
  }

  void Start() {
    StartTimer();
  }

  void StartTimer() {
    timer_.start(
        base::ConvertSecondsToDuration(options_.rate_s),
        std::bind(&CommandRunner::HandleTimer, this, pl::_1));
  }

  void HandleTimer(const base::error_code& ec) {
    if (ec == boost::asio::error::operation_aborted) { return; }

    DoTimer();
  }

  void DoTimer() {
    stats_.count++;
    if ((stats_.count % 100) == 0) {
      EmitStatus();
    }

    if (busy_) {
      stats_.skipped++;
      return;
    }

    busy_ = true;
    StartCycle();
  }

  void EmitStatus() {
    std::cout << fmt::format(
        "{:5d}  sk={:3d} st={:3d} qt={:3d} mr={:3d} "
        "er={:3d} wrt={:3d} wn={:3d}  lat={:.6f} ck={}\n",
        stats_.valid_reply,
        stats_.skipped,
        stats_.send_timeout,
        stats_.query_timeout,
        stats_.missing_reply,
        stats_.error_reply,
        stats_.wrong_reply_type,
        stats_.wrong_nonce,
        base::ConvertDurationToSeconds(stats_.total_valid_time) /
        stats_.valid_reply,
        client_->stats().checksum_errors);
  }

  void StartCycle() {
    start_time_ = mjlib::io::Now(executor_.context());

    MakeRequest();

    client_->AsyncRegister(
        &request_,
        &reply_,
        std::bind(&CommandRunner::HandleCycle, this, pl::_1));
  }

  void HandleCycle(const base::error_code& ec) {
    busy_ = false;
    const auto end_time = mjlib::io::Now(executor_.context());

    if (ec == boost::asio::error::operation_aborted) {
      stats_.send_timeout++;
      return;
    }

    base::FailIf(ec);

    for (size_t i = 0; i < kTargets.size(); i++) {
      // Check to see if the nonce value was correctly reported.
      if (reply_.replies.size() <= i) {
        stats_.missing_reply++;
        return;
      }

      const auto& reply = reply_.replies[i];
      if (reply.id != kTargets[i]) {
        stats_.missing_reply++;
        return;
      }

      if (reply.reply.count(kNonceRegister) == 0) {
        stats_.missing_reply++;
        return;
      }

      const auto& read_result = reply.reply.at(kNonceRegister);
      if (std::holds_alternative<uint32_t>(read_result)) {
        stats_.error_reply++;
        return;
      }

      auto value = std::get<mp::Format::Value>(read_result);
      if (!std::holds_alternative<int16_t>(value)) {
        stats_.wrong_reply_type++;
        return;
      }

      auto int16_value = std::get<int16_t>(value);
      if (int16_value != nonce_count_) {
        std::cout << fmt::format("got nonce {} expected {}\n",
                                 int16_value, nonce_count_);
        stats_.wrong_nonce++;
        return;
      }
    }

    FinishQuery(end_time);
  }


  void FinishQuery(boost::posix_time::ptime end_time) {
    stats_.total_valid_time += (end_time - start_time_);
    stats_.valid_reply++;
  }

  boost::asio::executor executor_;
  const Options options_;
  std::optional<mp::ThreadedClient> client_;

  io::RepeatingTimer timer_{executor_};
  mp::ThreadedClient::Request request_;
  mp::ThreadedClient::Reply reply_;

  bool busy_ = false;
  boost::posix_time::ptime start_time_;

  struct Stats {
    int count = 0;
    int skipped = 0;
    int send_timeout = 0;
    int query_timeout = 0;
    int missing_reply = 0;
    int error_reply = 0;
    int wrong_reply_type = 0;
    int wrong_nonce = 0;

    int valid_reply = 0;
    boost::posix_time::time_duration total_valid_time;
  };

  Stats stats_;
  int8_t nonce_count_ = 0;

  const std::vector<int> kTargets = {
    1, 2, 3,
    4, 5, 6,
    7, 8, 9,
    10, 11, 12
  };
};

}

int main(int argc, char** argv) {
  boost::asio::io_context context;

  po::options_description desc("Allowable options");

  Options options;

  desc.add_options()
      ("help,h", "display usage message")
      ("rate,r", po::value(&options.rate_s), "")
      ("port,p", po::value(&options.port), "")
      ;

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);

  if (vm.count("help")) {
    std::cout << desc;
    return 0;
  }

  CommandRunner command_runner{context.get_executor(), options};
  command_runner.Start();

  context.run();

  return 0;
}
