// Copyright 2016 Josh Pieper, jjp@pobox.com.  All rights reserved.
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

#include <map>

#include <boost/program_options.hpp>

#include "async_i2c.h"

namespace mjmech {
namespace base {

class I2CFactory : boost::noncopyable {
 public:
  I2CFactory(boost::asio::io_service&);
  ~I2CFactory();

  class GeneratorParameters {
   public:
    virtual ~GeneratorParameters() {}
    virtual boost::program_options::options_description*
      options_description() = 0;
  };

  class Generator {
   public:
    virtual ~Generator() {}

    virtual std::string name() const = 0;
    virtual std::unique_ptr<GeneratorParameters> MakeParameters() const = 0;
    virtual void AsyncCreate(const GeneratorParameters&,
                             SharedI2CHandler) const = 0;
  };

  void Register(std::unique_ptr<Generator>);

  class Parameters {
   public:
    boost::program_options::options_description* options_description() {
      return &options_description_;
    }

    std::string type_;
    std::map<std::string, std::unique_ptr<GeneratorParameters> > generators_;

    boost::program_options::options_description options_description_;
  };

  std::unique_ptr<Parameters> MakeParameters() const;

  void AsyncCreate(const Parameters&, SharedI2CHandler handler) const;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};
}
}
