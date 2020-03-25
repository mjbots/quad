// Copyright 2014-2020 Josh Pieper, jjp@pobox.com.  All rights reserved.
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

#include "mjlib/base/clipp_archive.h"

#include "base/handler_util.h"


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
  StartArchive(mjlib::io::ErrorCallback handler)
      : joiner(std::make_shared<ErrorHandlerJoiner>(std::move(handler))) {}

  template <typename Serializable>
  static void Start(Serializable* serializable,
                    mjlib::io::ErrorCallback callback) {
    StartArchive archive(std::move(callback));
    archive.Accept(serializable);
  }

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
    (*value)->AsyncStart(
        joiner->Wrap(std::string("starting: '") + name + "'"));
  }

  template <typename T>
  void Helper(const char*, T*, long) {}

  std::shared_ptr<ErrorHandlerJoiner> joiner;
};

class ClippComponentArchive {
 public:
  template <typename T>
  ClippComponentArchive& Accept(T* value) {
    value->Serialize(this);
    return *this;
  }

  template <typename NameValuePair>
  void Visit(const NameValuePair& pair) {
    VisitHelper(pair, pair.value(), static_cast<int32_t>(0));
  }

  template <typename NameValuePair, typename Serializable>
  auto VisitHelper(const NameValuePair& pair,
                   Serializable* serializable,
                   int32_t) -> decltype((*serializable)->program_options()) {
    group_.push_back(
        clipp::with_prefix(std::string(pair.name()) + ".",
                           (*pair.value())->program_options()));
    return {};
  }

  template <typename NameValuePair, typename Serializable>
  auto VisitHelper(const NameValuePair& pair,
                   Serializable*,
                   int64_t) {
    group_.push_back(
        mjlib::base::ClippArchive(std::string(pair.name()) + ".")
        .Accept((*pair.value())->parameters()).release());
  }

  clipp::group release() {
    return std::move(group_);
  }

  clipp::group group() {
    return group_;
  }

 private:
  clipp::group group_;
};

}
}
