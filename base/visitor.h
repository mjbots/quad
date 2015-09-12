// Copyright 2014-2015 Josh Pieper, jjp@pobox.com.  All rights reserved.
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

namespace legtool {

template <typename T>
struct NameValuePair : public std::tuple<T*, const char*> {
  NameValuePair(T* value, const char* name)
      : std::tuple<T*, const char*>(value, name) {}
  T* value() { return std::get<0>(*this); }
  T* value() const { return std::get<0>(*this); }
  const char* name() const { return std::get<1>(*this); }
};

template <typename T>
NameValuePair<T> MakeNameValuePair(T* value, const char* name) {
  return NameValuePair<T>(value, name);
}
}

#define LT_NVP(x) legtool::MakeNameValuePair(&x, #x)
