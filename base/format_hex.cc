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

#include "base/format_hex.h"

#include <sstream>

#include <fmt/format.h>

namespace mjmech {
namespace base {

std::string FormatHex(std::string_view data) {
  std::ostringstream ostr;
  for (char c : data) {
    ostr << fmt::format("{:02x}", static_cast<int>(static_cast<uint8_t>(c)));
  }
  return ostr.str();
}

}
}
