// Copyright 2014-2015 Josh Pieper, jjp@pobox.com.  All rights reserved.
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

#include "mech_warfare.h"

#include <boost/property_tree/json_parser.hpp>

#include "base/property_tree_archive.h"

namespace mjmech {
namespace mech {

MechWarfare::MechWarfare(boost::asio::io_service& service)
    : service_(service),
      factory_(service),
      server_(service) {
}

void MechWarfare::AsyncStart(base::ErrorHandler handler) {
  try {
    RippleConfig ripple_config;
    ripple_config = LoadRippleConfig();

    m_.gait_driver->SetGait(std::unique_ptr<RippleGait>(
                                new RippleGait(ripple_config)));

    NetworkListen();
  } catch (base::SystemError& se) {
    service_.post(std::bind(handler, se.error_code()));
    return;
  }

  parameters_.children.Start(handler);
}

RippleConfig MechWarfare::LoadRippleConfig() {
  RippleConfig ripple_config;
  try {
    std::ifstream inf(parameters_.gait_config);
    if (!inf.is_open()) {
      throw base::SystemError::syserrno("error opening config");
    }

    boost::property_tree::ptree tree;
    boost::property_tree::read_json(inf, tree);
    auto optional_child = tree.get_child_optional("gaitconfig.ripple");
    if (!optional_child) {
      throw base::SystemError::einval("could not find ripple config in file");
    }

    base::PropertyTreeReadArchive(
        *optional_child,
        base::PropertyTreeReadArchive::kErrorOnMissing).Accept(&ripple_config);

    std::string type = tree.get<std::string>("ikconfig.iktype");
    auto& leg_configs = ripple_config.mechanical.leg_config;
    for (size_t i = 0; i < leg_configs.size(); i++) {
      if (type == "Mammal") {
        MammalIK::Config config;

        std::string field = (boost::format("ikconfig.leg.%d") % i).str();
        auto optional_child = tree.get_child_optional(field);
        if (!optional_child) {
          throw base::SystemError::einval("could not locate field: " + field);
        }

        base::PropertyTreeReadArchive(
            *optional_child,
            base::PropertyTreeReadArchive::kErrorOnMissing).Accept(&config);
        leg_configs[i].leg_ik =
            boost::shared_ptr<IKSolver>(new MammalIK(config));
      } else {
        throw base::SystemError::einval("unknown iktype: " + type);
      }
    }
  } catch (base::SystemError& se) {
    se.error_code().Append(
        "while opening config: '" + parameters_.gait_config + "'");
    throw;
  }

  return ripple_config;
}

void MechWarfare::NetworkListen() {
  boost::asio::ip::udp::endpoint endpoint(
      boost::asio::ip::udp::v4(), parameters_.port);
  server_.open(boost::asio::ip::udp::v4());
  server_.bind(endpoint);
  StartRead();
}

void MechWarfare::StartRead() {
  server_.async_receive_from(
      boost::asio::buffer(receive_buffer_),
      receive_endpoint_,
      std::bind(&MechWarfare::HandleRead, this,
                std::placeholders::_1,
                std::placeholders::_2));
}

void MechWarfare::HandleRead(base::ErrorCode ec, std::size_t size) {
  FailIf(ec);

  std::string data(receive_buffer_, size);
  StartRead();

  boost::property_tree::ptree tree;
  try {
    std::istringstream inf(data);
    boost::property_tree::read_json(inf, tree);
  } catch (std::exception& e) {
    std::cerr << "error reading network command: " << e.what() << "\n";
    return;
  }

  HandleMessage(tree);
}

void MechWarfare::HandleMessage(const boost::property_tree::ptree& tree) {
  auto optional_gait = tree.get_child_optional("gait");
  if (optional_gait) { HandleMessageGait(*optional_gait); }
}

void MechWarfare::HandleMessageGait(const boost::property_tree::ptree& tree) {
  Command command;
  base::PropertyTreeReadArchive(
      tree, base::PropertyTreeReadArchive::kErrorOnMissing).Accept(&command);
  m_.gait_driver->SetCommand(command);
}

}
}
