// Copyright 2016-2019 Josh Pieper, jjp@pobox.com.  All rights reserved.
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

#include "debug_i2c_generator.h"

#include "mjlib/base/program_options_archive.h"

namespace po = boost::program_options;

namespace mjmech {
namespace base {

DebugI2CGenerator::Parameters::Parameters() {
  mjlib::base::ProgramOptionsArchive(&options_description_).Accept(this);
}

DebugI2CGenerator::DebugI2CGenerator(boost::asio::io_service& service)
  : service_(service) {}
DebugI2CGenerator::~DebugI2CGenerator() {}

std::unique_ptr<I2CFactory::GeneratorParameters>
DebugI2CGenerator::MakeParameters() const {
  return std::unique_ptr<I2CFactory::GeneratorParameters>(new Parameters());
}

void DebugI2CGenerator::AsyncCreate(
    const I2CFactory::GeneratorParameters& generator_parameters,
    SharedI2CHandler handler) const {
  const Parameters& parameters =
      dynamic_cast<const Parameters&>(generator_parameters);
  auto it = handler_map_.find(parameters.key);
  if (it == handler_map_.end()) {
    service_.post(
        std::bind(
            handler,
            mjlib::base::error_code::einval(
                "unknown key '" + parameters.key + "'"), SharedI2C()));
  } else {
    service_.post(std::bind(handler, mjlib::base::error_code(), it->second));
  }
}

void DebugI2CGenerator::InstallHandler(
    const std::string& key, SharedI2C i2c) {
  handler_map_.insert(std::make_pair(key, i2c));
}

}
}
