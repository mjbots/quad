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

#include <boost/asio/io_service.hpp>
#include <boost/program_options.hpp>

#include <fmt/format.h>

#include "mjlib/base/fail.h"
#include "mjlib/base/program_options_archive.h"
#include "mjlib/base/time_conversions.h"
#include "mjlib/io/repeating_timer.h"
#include "mjlib/io/stream_factory.h"
#include "mjlib/multiplex/asio_client.h"

#include "base/now.h"

namespace base = mjlib::base;
namespace io = mjlib::io;
namespace mp = mjlib::multiplex;
namespace po = boost::program_options;
namespace pl = std::placeholders;

namespace {

constexpr int kSetupRegister = 0;  // mode
constexpr int kSetupValue = 0;  // kStopped
constexpr int kNonceRegister = 0x10;  // kPwmPhaseA

struct Options {
  double rate_s = 0.1;
};

class CommandRunner {
 public:
  CommandRunner(boost::asio::io_service& service,
                io::StreamFactory* stream_factory,
                const io::StreamFactory::Options& stream_options,
                const Options& options)
      : service_(service),
        stream_factory_(stream_factory),
        options_(options) {
    stream_factory->AsyncCreate(
        stream_options,
        std::bind(&CommandRunner::HandleStream, this, pl::_1, pl::_2));
  }

  void HandleStream(const base::error_code& ec, io::SharedStream stream) {
    base::FailIf(ec);

    stream_ = stream;
    client_.emplace(stream_.get());

    MaybeStart();
  }

  void MaybeStart() {
    if (!stream_) { return; }

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
        "{:5d}  sk={:3d} st={:3d} qt={:3d} mr={:3d} er={:3d} wrt={:3d} wn={:3d}  lat={:.6f}\n",
        stats_.valid_reply,
        stats_.skipped,
        stats_.send_timeout,
        stats_.query_timeout,
        stats_.missing_reply,
        stats_.error_reply,
        stats_.wrong_reply_type,
        stats_.wrong_nonce,
        base::ConvertDurationToSeconds(stats_.total_valid_time) /
        stats_.valid_reply);
  }

  void StartCycle() {
    start_time_ = mjmech::base::Now(service_);

    nonce_count_ = (nonce_count_ + 1) % 100;

    StartServo(0);
  }

  void StartServo(int index) {
    const int servo_num = kTargets[index];
    // For each servo, write something to the setup register and the
    // nonce register, then a representative amount of data elsewhere.

    request_ = {};
    request_.WriteSingle(kSetupRegister, kSetupValue);

    // The position command needs to set 3 values.  For now, we'll
    // send 3 int16's.
    auto i16 = [](auto v) { return static_cast<int16_t>(v); };
    request_.WriteMultiple(kNonceRegister, {i16(nonce_count_), i16(0), i16(0)});

    request_.ReadMultiple(kNonceRegister, 3, 1);
    // And read two i8 regs, like voltage and fault
    request_.ReadMultiple(0x006, 2, 0);

    client_->AsyncRegister(servo_num,
                           request_,
                           std::bind(&CommandRunner::HandleServo, this,
                                     pl::_1, pl::_2, index));
  }

  void HandleServo(const base::error_code& ec,
                   const mp::RegisterReply& reply,
                   int index) {
    const auto end_time = mjmech::base::Now(service_);
    if (ec == boost::asio::error::operation_aborted) {
      stats_.send_timeout++;
      busy_ = false;
      return;
    }

    base::FailIf(ec);

    // Check to see if the nonce value was correctly reported.
    if (reply.count(kNonceRegister) == 0) {
      stats_.missing_reply++;
      busy_ = false;
      return;
    }

    const auto& read_result = reply.at(kNonceRegister);
    if (std::holds_alternative<uint32_t>(read_result)) {
      stats_.error_reply++;
      busy_ = false;
      return;
    }

    auto value = std::get<mp::Format::Value>(read_result);
    if (!std::holds_alternative<int16_t>(value)) {
      stats_.wrong_reply_type++;
      busy_ = false;
      return;
    }

    auto int16_value = std::get<int16_t>(value);
    if (int16_value != nonce_count_) {
      std::cout << fmt::format("got nonce {} expected {}\n",
                               int16_value, nonce_count_);
      stats_.wrong_nonce++;
      busy_ = false;
      return;
    }

    const int next_index = index + 1;
    if (next_index >= kTargets.size()) {
      busy_ = false;
      FinishQuery(end_time);
    } else {
      StartServo(next_index);
    }
  }


  void FinishQuery(boost::posix_time::ptime end_time) {
    stats_.total_valid_time += (end_time - start_time_);
    stats_.valid_reply++;
  }


  boost::asio::io_service& service_;
  io::StreamFactory* const stream_factory_;
  const Options options_;
  io::SharedStream stream_;
  std::optional<mp::AsioClient> client_;

  io::RepeatingTimer timer_{service_};
  mp::RegisterRequest request_;

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
  boost::asio::io_service service;
  io::StreamFactory factory{service};

  io::StreamFactory::Options stream_options;
  po::options_description desc("Allowable options");

  Options options;

  desc.add_options()
      ("help,h", "display usage message")
      ("rate,r", po::value(&options.rate_s), "")
      ;

  base::ProgramOptionsArchive(&desc).Accept(&stream_options);

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);

  if (vm.count("help")) {
    std::cout << desc;
    return 0;
  }

  CommandRunner command_runner{service, &factory, stream_options, options};

  service.run();

  return 0;
}
