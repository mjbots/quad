// Copyright 2020 Josh Pieper, jjp@pobox.com.
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
namespace ffmpeg {

template <typename T, typename Derived>
class RefBase {
 public:
  RefBase() {}
  RefBase(RefBase&& rhs) : value_(rhs.value_) {
    rhs.value_ = nullptr;
  }

  RefBase& operator=(RefBase&& rhs) {
    static_cast<Derived*>(this)->reset();
    std::swap(value_, rhs.value_);
    return *this;
  }

  ~RefBase() {
    static_cast<Derived*>(this)->reset();
  }

  T& operator*() { return *value_; }
  const T& operator*() const { return *value_; }
  T* operator->() { return value_; }
  const T*  operator->() const { return value_; }

  static Derived MakeInternal(T* value) {
    return Derived(value);
  }

 protected:
  RefBase(T* value) : value_(value) {}

  T* value_ = nullptr;
};

}
}
