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

#include <iostream>
#include <vector>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/optional.hpp>

#include "visit_archive.h"
#include "visitor.h"

namespace mjmech {
namespace base {
class JsonWriteArchive : public VisitArchive<JsonWriteArchive> {
 public:
  typedef VisitArchive<JsonWriteArchive> Base;

  JsonWriteArchive(std::ostream& ostr, int indent=0)
      : ostr_(ostr), indent_(indent) {}

  template <typename Serializable>
  static std::string Write(Serializable* value) {
    std::ostringstream ostr;
    JsonWriteArchive(ostr).Accept(value);
    ostr << "\n";
    return ostr.str();
  }

  template <typename NameValuePair>
  void Visit(const NameValuePair& nvp) {
    if (previous_) { ostr_ << ",\n"; }
    ostr_ << MakeChildIndent() << "\"" << nvp.name() << "\":";
    VisitArchive<JsonWriteArchive>::Visit(nvp);
    previous_ = true;
  }

  template <typename Serializable>
  JsonWriteArchive& Accept(Serializable* value) {
    ostr_ << "{\n";
    VisitArchive<JsonWriteArchive>::Accept(value);
    ostr_ << "\n" << MakeParentIndent() << "}";
    return *this;
  }

  template <typename NameValuePair>
  void VisitSerializable(const NameValuePair& pair) {
    JsonWriteArchive(ostr_, indent_ + 2).Accept(pair.value());
  }

  template <typename NameValuePair>
  void VisitScalar(const NameValuePair& pair) {
    VisitValue(pair, pair.value(), 0);
  }

  template <typename NameValuePair, typename T>
  void VisitValue(const NameValuePair& nvp, std::vector<T>* data, int) {
    DoArray(data);
  }

  template <typename NameValuePair, typename T, size_t N>
  void VisitValue(const NameValuePair& nvp, std::array<T, N>* data, int) {
    DoArray(data);
  }

  template <typename Array>
  void DoArray(Array* a) {
    ostr_ << "[";
    JsonWriteArchive sub_archive(ostr_, indent_ + 2);
    for (size_t i = 0; i < a->size(); ++i) {
      if (i != 0) { ostr_ << ","; }
      ostr_ << "\n" << sub_archive.MakeChildIndent();
      static_cast<Base*>(&sub_archive)->Visit(MakeNameValuePair(&(*a)[i], ""));
    }
    ostr_ << "\n" << MakeChildIndent() << "]";
  }

  template <typename NameValuePair, typename T>
  void VisitValue(const NameValuePair& nvp, boost::optional<T>* data, int) {
    if (!(*data)) {
      ostr_ << "null";
    } else {
      VisitArchive<JsonWriteArchive>::Visit(
          MakeNameValuePair(&(*data), nvp.name()));
    }
  }

  template <typename NameValuePair>
  void VisitValue(const NameValuePair& nvp, std::string* value, int) {
    ostr_ << "\"" << *value << "\"";
  }

  template <typename NameValuePair>
  void VisitValue(const NameValuePair& nvp, bool* value, int) {
    ostr_ << ((*value) ? "true" : "false");
  }

  template <typename NameValuePair>
  void VisitValue(const NameValuePair& nvp,
                  boost::posix_time::ptime* value, int) {
    ostr_ << "\"" << *value << "\"";
  }

  template <typename NameValuePair, typename T>
  void VisitValue(const NameValuePair& nvp, T* value, long) {
    ostr_ << *value;
  }

 private:
  std::string MakeParentIndent() const {
    return std::string(indent_, ' ');
  }

  std::string MakeChildIndent() const {
    return std::string(indent_ + 2, ' ');
  }

  std::ostream& ostr_;
  const int indent_;
  bool previous_ = false;
};
}
}
