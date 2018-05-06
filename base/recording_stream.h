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

#include <boost/iostreams/categories.hpp>
#include <boost/iostreams/operations.hpp>

namespace mjmech {
namespace base {
struct RecordingInputSource {
  typedef char char_type;
  typedef boost::iostreams::source_tag category;

  RecordingInputSource(std::istream& istr) : context(new Context{istr}) {}

  std::streamsize read(char* s, std::streamsize n) {
    context->istr.read(s, n);
    std::streamsize result = context->istr.gcount();
    context->ostr.write(s, result);
    return result;
  }

  std::string str() const { return context->ostr.str(); }

  struct Context {
    std::istream& istr;
    std::ostringstream ostr;

    Context(std::istream& istr_in) : istr(istr_in) {}
  };
  std::shared_ptr<Context> context;
};
}
}
