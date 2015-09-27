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

#include "fail.h"

#include <cxxabi.h>
#include <execinfo.h>
#include <string.h>

#include <boost/format.hpp>

namespace mjmech {
namespace base {

void AssertNotReached() {
  Fail("assert not reached");
}

namespace {
std::string FormatFrame(const std::string& frame) {
  size_t openparen = frame.find_first_of('(');
  if (openparen == std::string::npos) { return frame; }
  size_t closeparen = frame.find_first_of(')');
  if (closeparen == std::string::npos) { return frame; }

  if (closeparen < openparen) { return frame; }

  const std::string file = frame.substr(0, openparen);
  const std::string symbol_offset =
      frame.substr(openparen + 1, closeparen - openparen - 1);
  const std::string rest = frame.substr(closeparen + 1);

  size_t plus = symbol_offset.find_last_of('+');
  if (plus == std::string::npos) { return frame; }

  const std::string symbol = symbol_offset.substr(0, plus);
  const std::string offset = symbol_offset.substr(plus + 1);

  int status = -1;
  char* demangled =
      abi::__cxa_demangle(symbol.c_str(), nullptr, nullptr, &status);
  if (status != 0) {
    if (demangled) { ::free(demangled); }
    return frame;
  }

  std::string result = file + "(" + demangled + "+" + offset + ")" + rest;
  ::free(demangled);
  return result;
}
}

void Fail(const std::string& message) {
  const int kMaxFrames = 100;
  void *buffer[kMaxFrames] = {};
  int frames = ::backtrace(buffer, kMaxFrames);

  std::cerr << "Fatal error:\n";
  char **strings = ::backtrace_symbols(buffer, frames);

  for (int i = 0; i < frames; i++) {
    std::cerr << FormatFrame(strings[i]) << "\n";
  }

  std::cerr << "\n" << message << "\n\n";

  ::abort();
}

void Fail(const boost::format& fmt) {
  Fail(fmt.str());
}

void FailIf(const ErrorCode& ec) {
  if (ec) { Fail(ec.message()); }
}

void FailIfErrno(bool terminate) {
  if (terminate) { Fail(strerror(errno)); }
}

}
}
