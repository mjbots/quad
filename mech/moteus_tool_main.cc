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

#include "moteus/tool/moteus_tool.h"

#include "mjlib/base/fail.h"

#include "mjlib/multiplex/stream_asio_client_builder.h"

#include "mech/rpi3_threaded_client.h"

namespace {
class Rpi3Client : public mjlib::multiplex::AsioClient {
 public:
  struct Options : mjmech::mech::Rpi3ThreadedClient::Options {
    Options() {
      query_timeout_s = 0.001;
    }
  };

  Rpi3Client(const boost::asio::executor& executor,
             const Options& options)
      : client_(executor, options) {}
  ~Rpi3Client() override {}

  mjlib::io::SharedStream MakeTunnel(
      uint8_t id,
      uint32_t channel,
      const TunnelOptions& options) override {
    return client_.MakeTunnel(id, channel, [options]() {
        mjmech::mech::Rpi3ThreadedClient::TunnelOptions dest_options;
        dest_options.poll_rate = options.poll_rate;
        return dest_options;
      }());
  }

  void AsyncStart(mjlib::io::ErrorCallback cbk) {
    cbk(mjlib::base::error_code());
  }

  void AsyncRegister(uint8_t id, const mjlib::multiplex::RegisterRequest&,
                     RegisterHandler) override {
    mjlib::base::AssertNotReached();
  }

  /// If only commands (and no queries are sent), multiple devices may
  /// be addressed in a single operation.
  void AsyncRegisterMultiple(const std::vector<IdRequest>&,
                             mjlib::io::ErrorCallback) override {
    mjlib::base::AssertNotReached();
  }

 private:
  mjmech::mech::Rpi3ThreadedClient client_;
};
}

int main(int argc, char** argv) {
  boost::asio::io_context context;
  mjlib::io::Selector<mjlib::multiplex::AsioClient> selector{
    context.get_executor(), "client_type"};
  selector.Register<mjlib::multiplex::StreamAsioClientBuilder>("stream");
  selector.Register<Rpi3Client>("rpi3");
  selector.set_default("rpi3");
  return moteus::tool::moteus_tool_main(context, argc, argv, &selector);
}
