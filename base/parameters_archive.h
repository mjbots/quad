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

#include <tuple>

#include "program_options.h"
#include "visitor.h"

namespace mjmech {
namespace base {

template <typename BaseArchive>
class ParametersArchive {
 public:
  ParametersArchive(BaseArchive* base) : base_(base) {}

  template <typename T>
  T* Accept(T* t) {
    t->Serialize(this);
    return t;
  }

  template <typename NameValuePair>
  void Visit(const NameValuePair& pair) {
    VisitHelper(pair, pair.value(), static_cast<int32_t>(0));
  }

  template <typename NameValuePair, typename Serializable>
  auto VisitHelper(const NameValuePair& pair,
                   Serializable* serializable,
                   int32_t value) -> decltype((*serializable)->options()) {
    MergeProgramOptions((*serializable)->options(),
                        std::string(pair.name()) + ".",
                        base_->options());
    return (*serializable)->options();
  }

  template <typename NameValuePair, typename Serializable>
  void VisitHelper(const NameValuePair& pair,
                   Serializable* serializable,
                   int64_t) {
    base_->Visit(
        MakeNameValuePair((*pair.value())->parameters(), pair.name()));
  }

  BaseArchive* const base_;
};
}
}
