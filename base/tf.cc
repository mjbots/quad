// Copyright 2014-2018 Josh Pieper, jjp@pobox.com.  All rights reserved.
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

#include "tf.h"

#include <fmt/format.h>

namespace mjmech {
namespace base {

std::string Frame::str() const {
  return fmt::format("<Frame t=%s r=%s>",
                     transform.translation.str(),
                     transform.rotation.str());
}

}
}
