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

namespace mjmech {
namespace base {

template <typename Derived, typename ResultType, typename... Generators>
class GenericFactory : boost::noncopyable {
 public:
  GenericFactory(boost::asio::io_service& service) : service_(service) {}

  using Types = meta::list<Generators...>;

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
          [handler](ErrorCode ec, ResultType result) mutable {
            handler(ec.error_code(), result);
          },
          &visited);
      meta::for_each(Types{}, try_create);
      if (!visited) {
        throw SystemError::einval(
            "Unknown type: '" + parameters.type + "'");
      }
    } catch (SystemError& e) {
      handler(e.error_code(), ResultType());
    }
  }

 private:
  template <typename Handler>
  void HandleCreate(ErrorCode ec,
                    ResultType stream,
                    const Parameters& parameters,
                    Handler handler) {
    if (ec) {
      handler(ec, ResultType());
    } else {
      handler(ec, stream);
    }
  }

  struct TryCreate {
    TryCreate(GenericFactory* f,
              const Parameters& p,
              std::function<void (ErrorCode, ResultType)> h,
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
    std::function<void (ErrorCode, ResultType)> handler;
    bool* const visited;
  };

  boost::asio::io_service& service_;
  std::tuple<Generators...> generators_;
};
}
}
