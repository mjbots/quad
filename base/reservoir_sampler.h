// Copyright 2020 Josh Pieper, jjp@pobox.com.  All rights reserved.
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

#include <random>

namespace mjmech {
namespace base {

/// Recursively estimate a sample of a large data set.
///
/// The algorithm used here is "R" from "Random Sampling with a
/// Reservoir", J. Vitter.
template <typename T>
class ReservoirSampler {
 public:
  ReservoirSampler(std::size_t size)
      : size_(size) {}

  using Container = std::vector<T>;
  using iterator = Container::iterator;
  using const_iterator = Container::const_iterator;

  void Add(T value) {
    count_++;

    if (samples_.size() < size_) {
      samples.push_back(std::move(value));
    } else {
      const std::size_t M = static_cast<std::size_t>(
          std::uniform_int<>(0, count_ - 1)(rng_));
      if (M < size_) {
        samples_[M] = value;
      }
    }
  }

  iterator begin() { return samples_.begin(); }
  iterator end() { return samples_.end(); }

  const_iterator begin() const { return samples_.begin(); }
  const_iterator end() const { return samples_.end(); }

 private:
  Container samples_;
  std::size_t size_;

  std::mt19937 rng_;
  std::size_t count_ = 0;
}

}
}
