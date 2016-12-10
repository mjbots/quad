// Copyright 2015-2016 Josh Pieper, jjp@pobox.com.  All rights reserved.
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

#include <boost/asio/io_service.hpp>
#include <boost/noncopyable.hpp>
#include <boost/program_options/options_description.hpp>

#include "comm.h"

namespace mjmech {
namespace base {
class PipeGenerator;

/// This provides a dynamic mechanism for creating streams at run
/// time.  It has a pre-determined set of streams available.
class ConcreteStreamFactory : boost::noncopyable {
 public:
  ConcreteStreamFactory(boost::asio::io_service&);
  ~ConcreteStreamFactory();

  class Parameters {
   public:
    Parameters();
    ~Parameters();

    /// Ownership remains with the ConcreteStreamFactory, the return
    /// object is guaranteed to live as long as the callee does.
    boost::program_options::options_description* options();

    class Impl;
    std::unique_ptr<Impl> impl_;
  };

  PipeGenerator* pipe_generator();

  void AsyncCreate(const Parameters&, StreamHandler);

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}
}
