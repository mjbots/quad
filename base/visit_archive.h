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

namespace legtool {
/// A helper base class for classes which want to model the Archive
/// concept and differentiate between visited elements which are
/// themselves serializable or not.
template <typename Derived>
struct VisitArchive {
  template <typename Serializable>
  Derived& Accept(Serializable* serializable) {
    serializable->Serialize(static_cast<Derived*>(this));
    return *static_cast<Derived*>(this);
  }

  template <typename NameValuePair>
  void Visit(const NameValuePair& pair) {
    VisitHelper(pair, 0);
  }

 private:
  template <typename NameValuePair>
  auto VisitHelper(const NameValuePair& pair, int) ->
      decltype(pair.value()->Serialize(
                   static_cast<Derived*>(nullptr))) {
    static_cast<Derived*>(this)->VisitSerializable(pair);
  }

  template <typename NameValuePair>
  void VisitHelper(const NameValuePair& pair, long) {
    static_cast<Derived*>(this)->VisitScalar(pair);
  }
};

}
