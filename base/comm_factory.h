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

#pragma once

#include "meta/meta.hpp"

#include "comm.h"
#include "comm_factory_generators.h"
#include "visitor.h"

// TODO jpieper:
//
//  * Support generators which have constructors that take arguments.
//  * Support changing parameters after a stream has been created
//    (mostly for serial baud rate).

namespace mjmech {
namespace base {

SharedStream MakeStdioDebugStream(SharedStream);

/// This provides a dynamic mechanism for creating streams at
/// run-time.  It can be instantiated with any number of unique
/// Generators.  Each Generator must provide:
///
///  1) Serialize-able Parameters sub-type
///  2) static const char* type()
///  3) AsyncCreate
template <typename... StreamGenerators>
class StreamFactory : boost::noncopyable {
 public:
  StreamFactory(boost::asio::io_service& service) : service_(service) {}

  using Types = meta::list<StreamGenerators...>;

  struct ExtractParameter {
    template <typename T>
    using apply = typename T::Parameters;
  };

  using GeneratorParameters = meta::transform<Types, ExtractParameter>;

  // The overall set of parameters.  It has a unique sub-field for
  // each type of generator it can instantiate.  These are easily
  // accessed programmatically through the "Get" member function.
  struct Parameters {
    std::string type;
    bool stdio_debug = false;

    meta::apply_list<meta::quote<std::tuple>, GeneratorParameters> generators;

    template <typename Generator>
    typename Generator::Parameters* Get() {
      using Index = meta::find_index<Types, Generator>;
      return &std::get<Index{}>(generators);
    }

    template <typename Generator>
    const typename Generator::Parameters* Get() const {
      using Index = meta::find_index<Types, Generator>;
      return &std::get<Index{}>(generators);
    }

    template <typename Archive>
    struct VisitGenerator {
      VisitGenerator(Archive* a, Parameters* p) : archive_(a), parameters_(p) {}

      template <typename GeneratorIndex>
      void operator()(const GeneratorIndex&) {
        archive_->Visit(
            MakeNameValuePair(
                &std::get<GeneratorIndex::value>(parameters_->generators),
                meta::at_c<meta::list<StreamGenerators...>,
                           GeneratorIndex::value>::type()));
      }

      Archive* const archive_;
      Parameters* const parameters_;
    };

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(type));
      a->Visit(MJ_NVP(stdio_debug));

      VisitGenerator<Archive> visit_generator(a, this);
      meta::for_each(
          meta::as_list<meta::make_index_sequence<Types::size()>>{},
          visit_generator);
    }
  };

  template <typename Generator>
  Generator* generator() {
    using Index = meta::find_index<Types, Generator>;
    return &std::get<Index{}>(generators_);
  }

  template <typename Generator>
  const Generator* generator() const {
    using Index = meta::find_index<Types, Generator>;
    return &std::get<Index{}>(generators_);
  }


  /// Create a new stream given the current set of configuration.
  template <typename Handler>
  void AsyncCreate(const Parameters& parameters,
                   Handler handler) {
    try {
      bool visited = false;
      TryCreate try_create(
          this,
          parameters,
          [handler](ErrorCode ec, SharedStream stream) mutable {
            handler(ec.error_code(), stream);
          },
          &visited);
      meta::for_each(Types{}, try_create);
      if (!visited) {
        throw SystemError::einval(
            "Unknown stream type: '" + parameters.type + "'");
      }
    } catch (SystemError& e) {
      handler(e.error_code(), SharedStream());
    }
  }

 private:
  void HandleCreate(ErrorCode ec,
                    SharedStream stream,
                    const Parameters& parameters,
                    StreamHandler handler) {
    if (ec) {
      handler(ec, SharedStream());
    } else if (parameters.stdio_debug) {
      handler(ec, MakeStdioDebugStream(stream));
    } else {
      handler(ec, stream);
    }
  }

  struct TryCreate {
    TryCreate(StreamFactory* f,
              const Parameters& p,
              StreamHandler h,
              bool* v)
        : factory(f), parameters(p), handler(h), visited(v) {}
    template <typename Generator>
    void operator()(const Generator&) {
      if (parameters.type == Generator::type()) {
        BOOST_ASSERT(!*visited);
        *visited = true;
        using Index = meta::find_index<Types, Generator>;
        std::get<Index{}>(factory->generators_).AsyncCreate(
            factory->service_,
            *parameters.template Get<Generator>(),
            std::bind(&StreamFactory::HandleCreate, factory,
                      std::placeholders::_1,
                      std::placeholders::_2, parameters, handler));
      }
    }
    StreamFactory* const factory;
    const Parameters& parameters;
    StreamHandler handler;
    bool* visited;
  };

  boost::asio::io_service& service_;
  std::tuple<StreamGenerators...> generators_;
};

}
}
