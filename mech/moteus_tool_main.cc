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

#include "mjlib/io/selector.h"
#include "mjlib/multiplex/asio_client.h"

#include "mech/rpi3_hat_spidev.h"
#include "mech/rpi3_threaded_client.h"

int main(int argc, char** argv) {
  boost::asio::io_context context;
  mjlib::io::Selector<mjlib::multiplex::AsioClient> client_selector{
    context.get_executor(), "client_type"};
  client_selector.Register<mjmech::mech::Rpi3ThreadedClient>("rpi3");
  client_selector.Register<mjmech::mech::Rpi3HatSpidev>("spidev");
  client_selector.set_default("rpi3");
  return moteus::tool::moteus_tool_main(context, argc, argv, &client_selector);
}
