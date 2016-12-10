// Copyright 2016 Josh Pieper, jjp@pobox.com.  All rights reserved.
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

#include <boost/program_options.hpp>

namespace mjmech {
namespace base {

/// Take all options in @p source, and create options corresponding to
/// them in @p destination, prefixing the long option name with @p
/// destination_prefix.
void MergeProgramOptions(
    boost::program_options::options_description* source,
    const std::string& destination_prefix,
    boost::program_options::options_description* destination);

void SetOption(
    boost::program_options::options_description* source,
    const std::string& key,
    const std::string& value);

}
}
