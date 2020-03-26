// Copyright 2015 Josh Pieper, jjp@pobox.com.
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
/// A dumb circular buffer which is based on a std::vector.  Thus it
/// has the property that in steady state no allocations are required,
/// unlike std::list, but actually works with move-only objects,
/// unlike boost::circular buffer as of boost 1.55.
template <typename T>
class circular_buffer {
 public:
  circular_buffer() { data_.resize(2); }

  void push_back(T&& value) {
    if (full()) { resize(data_.size() * 2); }
    data_[insert_] = std::move(value);
    insert_ = (insert_ + 1) % data_.size();
  }

  void pop_front() {
    remove_ = (remove_ + 1) % data_.size();
  }

  T& front() { return data_[remove_]; }
  const T& front() const { return data_[remove_]; }

  T& back() { return data_[insert_]; }
  const T& back() const { return data_[insert_]; }

  bool empty() const { return insert_ == remove_; }
  bool full() const {
    return ((insert_ + 1) % data_.size()) == remove_;
  }

  size_t capacity() const { return data_.size() - 1; }

 private:
  void resize(size_t size) {
    std::vector<T> new_data(size);
    size_t new_offset = 0;
    while (!empty()) {
      new_data[new_offset] = std::move(front());
      pop_front();
      new_offset++;
    }
    data_.swap(new_data);
    remove_ = 0;
    insert_ = new_offset;
  }

  std::vector<T> data_;
  size_t insert_ = 0;
  size_t remove_ = 0;
  size_t size_ = 0;
};
}
}
