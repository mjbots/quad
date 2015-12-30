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

#include "base/comm.h"

namespace mjmech {
namespace simulator {

class HerkulexProtocol {
 public:
  class Operations {
   public:
    virtual ~Operations() {}

    virtual bool address_valid(int) const = 0;

    typedef std::pair<int, double> ServoAngle;
    virtual void SJog(const std::vector<ServoAngle>&) = 0;

    virtual void Reboot(int servo) = 0;
    virtual void WriteRam(int servo, uint8_t addr, uint8_t data) = 0;
    virtual uint8_t ReadRam(int servo, uint8_t addr) = 0;
  };

  HerkulexProtocol(base::AsyncStream&, Operations&);
  ~HerkulexProtocol();

  void AsyncStart(base::ErrorHandler);

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}
}
