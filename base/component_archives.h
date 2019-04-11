// Copyright 2014-2019 Josh Pieper, jjp@pobox.com.  All rights reserved.
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

#include "handler_util.h"
#include "parameters_archive.h"

namespace mjmech {
namespace base {
struct EnableArchive {
  EnableArchive(std::map<std::string, bool>& enabled): enabled(enabled) {}

  template <typename T>
  EnableArchive& Accept(T* value) {
    value->Serialize(this);
    return *this;
  }

  template <typename NameValuePair>
  void Visit(const NameValuePair& pair) {
    Helper(pair.name(), pair.value(), 0);
  }

  template <typename T>
  auto Helper(const char* name, T* value, int)
      -> decltype((*value)->AsyncStart(mjlib::io::ErrorCallback())) {
    enabled[name] = true;
  }

  template <typename T>
  void Helper(const char*, T*, long) {}

  std::map<std::string, bool>& enabled;
};

struct StartArchive {
  StartArchive(std::map<std::string, bool>& enabled,
               mjlib::io::ErrorCallback handler)
      : enabled(enabled),
        joiner(std::make_shared<ErrorHandlerJoiner>(handler)) {}

  template <typename T>
  StartArchive& Accept(T* value) {
    value->Serialize(this);
    return *this;
  }

  template <typename NameValuePair>
  void Visit(const NameValuePair& pair) {
    Helper(pair.name(), pair.value(), 0);
  }

  template <typename T>
  auto Helper(const char* name, T* value, int)
      -> decltype((*value)->AsyncStart(mjlib::io::ErrorCallback())) {
    if (enabled[name]) {
      (*value)->AsyncStart(
          joiner->Wrap(std::string("starting: '") + name + "'"));
    }
  }

  template <typename T>
  void Helper(const char*, T*, long) {}

  std::map<std::string, bool>& enabled;
  std::shared_ptr<ErrorHandlerJoiner> joiner;
};

template <typename Members>
struct ComponentParameters {
  std::map<std::string, bool> enabled;

  template <typename Archive>
  void Serialize(Archive* a) {
    for (auto& pair: enabled) {
      a->Visit(mjlib::base::MakeNameValuePair(
                   &pair.second, (pair.first + "_enable").c_str()));
    }

    ParametersArchive<Archive>(a).Accept(members_);
  }

  ComponentParameters(Members* members) : members_(members) {
    EnableArchive(enabled).Accept(members);
  }

  void Start(mjlib::io::ErrorCallback handler) {
    StartArchive(enabled, handler).Accept(members_);
  }

  Members* const members_;
};

}
}
