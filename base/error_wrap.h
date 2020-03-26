// Copyright 2014-2018 Josh Pieper, jjp@pobox.com.
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

#include <sstream>

#include <boost/asio/spawn.hpp>
#include <boost/system/system_error.hpp>

#include <fmt/format.h>

#include "stringify.h"

namespace mjmech {
namespace base {

/// The following routine can be used to wrap coroutines such that
/// boost::system_error information is captured.  The default
/// boost::exception_ptr ignores this exception, making it challenging
/// to even report what happened.
template <typename Coroutine>
auto ErrorWrap(Coroutine coro) {
  return [=](boost::asio::yield_context yield) {
    try {
      return coro(yield);
    } catch (boost::system::system_error& e) {
      std::throw_with_nested(
          std::runtime_error(
              fmt::format("system_error: {}: {}",
                          e.what(), Stringify(e.code()))));
    } catch (std::runtime_error& e) {
      std::throw_with_nested(std::runtime_error(e.what()));
    }
  };
}

}
}
