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

#include "i2c_factory.h"

#include "visitor.h"

namespace mjmech {
namespace base {

class LinuxI2CGenerator : public I2CFactory::Generator {
 public:
  LinuxI2CGenerator(boost::asio::io_service& service);
  virtual ~LinuxI2CGenerator();

  class Parameters : public I2CFactory::GeneratorParameters {
   public:
    Parameters();
    virtual ~Parameters() {}

    boost::program_options::options_description*
    options() override { return &options_description_; }

    std::string device;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(device));
    }

   private:
    boost::program_options::options_description options_description_;
  };

  std::string name() const override { return "linux"; }

  std::unique_ptr<I2CFactory::GeneratorParameters>
  MakeParameters() const override;

  void AsyncCreate(const I2CFactory::GeneratorParameters&,
                   SharedI2CHandler) const override;

 private:
  boost::asio::io_service& service_;
};

}
}
