// Copyright 2015 Josh Pieper, jjp@pobox.com.  All rights reserved.
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

#include "concrete_comm_factory.h"

#include "comm_factory.h"
#include "program_options_archive.h"

namespace mjmech {
namespace base {

namespace {
typedef base::StreamFactory<
  base::StdioGenerator,
  base::SerialPortGenerator,
  base::TcpClientGenerator,
  base::PipeGenerator> Factory;
}

class ConcreteStreamFactory::Parameters::Impl : boost::noncopyable {
 public:
  Factory::Parameters parameters_;
  boost::program_options::options_description options_description_;
};

ConcreteStreamFactory::Parameters::Parameters()
    : impl_(new Impl()) {
  ProgramOptionsArchive(&impl_->options_description_).Accept(&impl_->parameters_);
}

ConcreteStreamFactory::Parameters::~Parameters() {}

boost::program_options::options_description*
ConcreteStreamFactory::Parameters::options_description() {
  return &impl_->options_description_;
}

class ConcreteStreamFactory::Impl : boost::noncopyable {
 public:
  Impl(boost::asio::io_service& service) : service_(service) {
  }

  boost::asio::io_service& service_;
  Factory factory_{service_};
};

ConcreteStreamFactory::ConcreteStreamFactory(boost::asio::io_service& service)
    : impl_(new Impl(service)) {}

ConcreteStreamFactory::~ConcreteStreamFactory() {}

PipeGenerator* ConcreteStreamFactory::pipe_generator() {
  return impl_->factory_.generator<PipeGenerator>();
}

void ConcreteStreamFactory::AsyncCreate(const Parameters& parameters,
                                        StreamHandler handler) {
  impl_->factory_.AsyncCreate(parameters.impl_->parameters_, handler);
}

}
}
