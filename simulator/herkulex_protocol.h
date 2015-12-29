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

#include "async_types.h"
#include "pool_ptr.h"

class AsyncStream;

class HerkulexProtocol {
 public:
  class Operations {
   public:
    virtual ~Operations() {}

    virtual uint8_t address() const = 0;
    virtual void WriteRam(uint8_t addr, uint8_t val) = 0;
    virtual uint8_t ReadRam(uint8_t addr) = 0;
    virtual void Reboot() = 0;
  };

  HerkulexProtocol(Pool& pool, AsyncStream&, Operations&);
  ~HerkulexProtocol();

  void AsyncStart(ErrorCallback);

 private:
  class Impl;
  PoolPtr<Impl> impl_;
};
