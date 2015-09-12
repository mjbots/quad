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

#include <map>

#include <boost/signals2/signal.hpp>

#include "meta/meta.hpp"

namespace legtool {
/// Maintain a local publish-subscribe model used for data objects.
template <typename... Registrars>
class TelemetryRegistry : boost::noncopyable {
 public:
  using RegistrarList = meta::list<Registrars...>;

  template <typename... Arg>
  TelemetryRegistry(Arg&&... args)
      : registrars_(std::forward<Arg>(args)...) {}

  /// Register a serializable object, and return a function object
  /// which when called will disseminate the
  /// object to any observers.
  template <typename DataObject>
  std::function<void (const DataObject*)>
  Register(const std::string& record_name) {
    // NOTE jpieper: Ideally this would return auto, and just let the
    // compiler sort out what type the lambda is.  But since C++11
    // doesn't support that yet, we return the type erasing
    // std::function.

    auto* ptr = new Concrete<DataObject>();
    auto result = [ptr](const DataObject* object) { ptr->signal(object); };

    records_.insert(
        std::make_pair(
            record_name, std::move(std::unique_ptr<Base>(ptr))));

    VisitRegistrar<DataObject> visit_registrant(this, record_name, ptr);
    meta::for_each(
        meta::as_list<meta::make_index_sequence<RegistrarList::size()> >{},
        visit_registrant);

    return result;
  };

  template <typename DataObject>
  void Register(const std::string& record_name,
                boost::signals2::signal<void (const DataObject*)>* signal) {
    signal->connect(Register<DataObject>(record_name));
  }

  template <typename Registrar>
  Registrar* registrar() {
    using Index = meta::find_index<RegistrarList, Registrar>;
    return &std::get<Index{}>(registrars_);
  }

  template <typename Registrar>
  const Registrar* registrar() const {
    using Index = meta::find_index<RegistrarList, Registrar>;
    return &std::get<Index{}>(registrars_);
  }

 private:
  struct Base {
    virtual ~Base() {}
  };

  template <typename Serializable>
  struct Concrete : public Base {
    virtual ~Concrete() {}

    boost::signals2::signal<void (const Serializable*)> signal;
  };

  template <typename DataObject>
  struct VisitRegistrar {
    VisitRegistrar(TelemetryRegistry* registry,
                   const std::string& name,
                   Concrete<DataObject>* concrete)
        : registry_(registry),
          name_(name),
          concrete_(concrete) {}

    template <typename RegistrarIndex>
    void operator()(const RegistrarIndex&) {
      std::get<RegistrarIndex::value>(registry_->registrars_).
          Register(name_, &concrete_->signal);
    }

    TelemetryRegistry* const registry_;
    const std::string name_;
    Concrete<DataObject>* const concrete_;
  };

  std::map<std::string, std::unique_ptr<Base> > records_;
  std::tuple<Registrars...> registrars_;
};

}
