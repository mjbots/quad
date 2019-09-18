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

#include "mech/rpi3_threaded_client.h"

namespace {
class Rpi3Client : public moteus::tool::ClientBase {
 public:
  Rpi3Client(boost::asio::executor executor)
      : client_(executor, []() {
          mjmech::mech::Rpi3ThreadedClient::Options options;
          options.query_timeout_s = 0.001;
          return options;
        }()) {}

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

 private:
  mjmech::mech::Rpi3ThreadedClient client_;
};

class Rpi3Creator : public moteus::tool::ClientMaker {
 public:
  void AddToProgramOptions(
      boost::program_options::options_description*) override {}

  void AsyncCreate(boost::asio::executor executor,
                   std::unique_ptr<moteus::tool::ClientBase>* client,
                   mjlib::io::ErrorCallback callback) override {
    *client = std::make_unique<Rpi3Client>(executor);
    callback(mjlib::base::error_code());
  }
};
}

int main(int argc, char** argv) {
  Rpi3Creator creator;
  return moteus::tool::moteus_tool_main(argc, argv, &creator);
}
