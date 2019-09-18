// Copyright 2019 Josh Pieper, jjp@pobox.com.
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

#include <sys/mman.h>

#include "mjlib/base/system_error.h"

namespace mjmech {
namespace base {

/// Manages ownership of an mmap'ed region of a given file descriptor.
class SystemMmap {
 public:
  SystemMmap() {}

  SystemMmap(int fd, size_t size, uint64_t offset) {
    ptr_ = ::mmap(0, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, offset);
    size_ = size;
    mjlib::base::system_error::throw_if(ptr_ == MAP_FAILED);
  }

  ~SystemMmap() {
    if (ptr_ != MAP_FAILED) {
      mjlib::base::system_error::throw_if(::munmap(ptr_, size_) < 0);
    }
  }

  SystemMmap(SystemMmap&& rhs) {
    std::swap(ptr_, rhs.ptr_);
    std::swap(size_, rhs.size_);
  }

  SystemMmap& operator=(SystemMmap&& rhs) {
    std::swap(ptr_, rhs.ptr_);
    std::swap(size_, rhs.size_);
    return *this;
  }

  SystemMmap(const SystemMmap&) = delete;
  SystemMmap& operator=(const SystemMmap&) = delete;

  void* ptr() { return ptr_; }

  // Since this is intended to be whatever, we just allow it to be
  // converted to any old pointer at will without extra hoops.
  template <typename T>
  operator T*() { return ptr_; }

  template <typename T>
  operator const T*() const { return ptr_; }

 private:
  void* ptr_ = MAP_FAILED;
  size_t size_ = 0;
};

}
}
