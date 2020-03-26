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

#include "mech/mime_type.h"

namespace mjmech {
namespace mech {

std::string_view GetMimeType(std::string_view path) {
  const auto ext = [&]() {
    const auto pos = path.rfind(".");
    if (pos == std::string_view::npos) { return std::string_view(); }
    return path.substr(pos);
  }();

  struct Mapping {
    std::string_view extension;
    std::string_view mime_type;
  };
  constexpr Mapping mappings[] = {
    { ".htm", "text/html" },
    { ".html", "text/html" },
    { ".css", "text/css" },
    { ".txt", "text/plain" },
    { ".js", "application/javascript" },
    { ".json", "application/json" },
    { ".xml", "application/xml" },
    { ".png", "image/png" },
    { ".jpeg", "image/jpeg" },
    { ".jpg", "image/jpg" },
    { ".gif", "image/gif" },
    { ".ico", "image/vnd.microsoft.icon" },
    { ".svg", "image/svg+xml" },
  };
  for (const auto& mapping : mappings) {
    if (ext == mapping.extension) {
      return mapping.mime_type;
    }
  }
  return "application/text";
}

}
}
