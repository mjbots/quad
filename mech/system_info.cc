// Copyright 2020 Josh Pieper, jjp@pobox.com.
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

#include "mech/system_info.h"

#include <fstream>
#include <functional>

#include <boost/asio/executor.hpp>
#include <boost/asio/post.hpp>
#include <boost/signals2/signal.hpp>

#include "mjlib/base/time_conversions.h"
#include "mjlib/io/now.h"
#include "mjlib/io/repeating_timer.h"

#include "base/telemetry_registry.h"

namespace pl = std::placeholders;

namespace mjmech {
namespace mech {

namespace {
struct Data {
  boost::posix_time::ptime timestamp;
  double temp_C = 0.0;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(timestamp));
    a->Visit(MJ_NVP(temp_C));
  }
};

#ifdef COM_GITHUB_MJBOTS_RASPBERRYPI
void PopulateData(Data* data) {
  {
    std::ifstream inf("/sys/class/thermal/thermal_zone0/temp");
    int temp_milliC = 0;
    inf >> temp_milliC;
    data->temp_C = temp_milliC * 0.001;
  }
}
#else
void PopulateData(Data* data) {
  // Nothing for now.
}
#endif
}

class SystemInfo::Impl {
 public:
  Impl(base::Context& context)
      : executor_(context.executor) {
    context.telemetry_registry->Register("system_info", &data_signal_);
  }

  void AsyncStart(mjlib::io::ErrorCallback callback) {
    timer_.start(
        mjlib::base::ConvertSecondsToDuration(period_s_),
        std::bind(&Impl::HandleTimer, this, pl::_1));
    boost::asio::post(
        executor_,
        std::bind(std::move(callback), mjlib::base::error_code()));
  }

  void HandleTimer(const mjlib::base::error_code& ec) {
    mjlib::base::FailIf(ec);

    Data data;
    data.timestamp = mjlib::io::Now(executor_.context());

    PopulateData(&data);

    data_signal_(&data);
  }

  clipp::group program_options() {
    clipp::group result;
    result.push_back(
        clipp::option("period_s") & clipp::value("", period_s_));
    return result;
  }

  boost::asio::executor executor_;
  mjlib::io::RepeatingTimer timer_{executor_};

  double period_s_ = 1.0;
  boost::signals2::signal<void (const Data*)> data_signal_;
};

SystemInfo::SystemInfo(base::Context& context)
    : impl_(std::make_unique<Impl>(context)) {}

SystemInfo::~SystemInfo() {}

void SystemInfo::AsyncStart(mjlib::io::ErrorCallback callback) {
  impl_->AsyncStart(std::move(callback));
}

clipp::group SystemInfo::program_options() {
  return impl_->program_options();
}

}
}
