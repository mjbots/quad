// Copyright 2015-2016 Josh Pieper, jjp@pobox.com.  All rights reserved.
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

#include "visitor.h"
#include "visit_archive.h"

#include "program_options.h"
#include "program_options_archive_detail.h"

namespace mjmech {
namespace base {

class ProgramOptionsArchive : public VisitArchive<ProgramOptionsArchive> {
 public:
  ProgramOptionsArchive(
      boost::program_options::options_description* description,
      std::string prefix = "")
      : description_(description),
        prefix_(prefix) {}

  template <typename NameValuePair>
  auto VisitSerializable(const NameValuePair& pair) {
    ProgramOptionsArchive sub(description_, prefix_ + pair.name() + ".");
    sub.Accept(pair.value());
  }

  template <typename NameValuePair>
  void VisitScalar(const NameValuePair& pair) {
    VisitOptions(pair, 0);
  }

  template <typename NameValuePair>
  auto VisitOptions(const NameValuePair& pair, int) ->
      decltype(pair.value()->options_description()) {
    MergeProgramOptions(pair.value()->options_description(),
                        prefix_ + pair.name() + ".",
                        description_);
    return 0;
  }

  template <typename NameValuePair>
  void VisitOptions(const NameValuePair& pair, long) {
    (*description_).add_options()(
        (prefix_ + pair.name()).c_str(),
        new detail::ProgramOptionsArchiveValue<NameValuePair>(pair));
  }

  boost::program_options::options_description* const description_;
  const std::string prefix_;
};

}
}
