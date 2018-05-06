// Copyright 2016-2018 Josh Pieper, jjp@pobox.com.  All rights reserved.
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

#include "i2c_factory.h"

#include <fmt/format.h>

#include "program_options.h"

namespace po = boost::program_options;

namespace mjmech {
namespace base {

class I2CFactory::Impl {
 public:
  Impl(boost::asio::io_service& service) : service_(service) {}

  boost::asio::io_service& service_;
  std::map<std::string, std::unique_ptr<Generator> > generators_;
};

I2CFactory::I2CFactory(boost::asio::io_service& service)
  : impl_(new Impl(service)) {}

I2CFactory::~I2CFactory() {}

void I2CFactory::Register(std::unique_ptr<Generator> generator) {
  impl_->generators_[generator->name()] = std::move(generator);
}

std::unique_ptr<I2CFactory::Parameters> I2CFactory::MakeParameters() const {
  std::unique_ptr<Parameters> result(new Parameters);

  result->options_description_.add_options()
      ("type", po::value(&result->type_), "")
      ;

  for (const auto& pair: impl_->generators_) {
    result->generators_[pair.first] =
        std::move(pair.second->MakeParameters());
  }

  for (auto& pair: result->generators_) {
    MergeProgramOptions(pair.second->options(),
                        pair.first + ".",
                        &result->options_description_);
  }

  return result;
}

void I2CFactory::AsyncCreate(const Parameters& parameters,
                             SharedI2CHandler handler) const {
  auto it = impl_->generators_.find(parameters.type_);
  if (it == impl_->generators_.end()) {
    impl_->service_.post(
        std::bind(
            handler,
            ErrorCode::einval(
                fmt::format("unknown type '{}'",
                            parameters.type_)),
            SharedI2C()));
    return;
  }

  auto parameters_it = parameters.generators_.find(parameters.type_);
  BOOST_ASSERT(parameters_it != parameters.generators_.end());
  it->second->AsyncCreate(*parameters_it->second, handler);
}

}
}
