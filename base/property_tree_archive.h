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

#include <tuple>
#include <vector>

#include <boost/property_tree/ptree.hpp>

#include "error_code.h"
#include "visit_archive.h"
#include "visitor.h"

namespace mjmech {
namespace base {
class PropertyTreeWriteArchive
    : public VisitArchive<PropertyTreeWriteArchive> {
 public:
  typedef boost::property_tree::ptree ptree;

  template <typename NameValuePair>
  void Visit(const NameValuePair& pair) {
    ptree tree;
    VisitHelper(pair.value(), &tree, 0);
    tree_.put_child(pair.name(), tree);
  }

  template <typename Value>
  auto VisitHelper(Value* value, ptree* tree, int) ->
      decltype(value->Serialize((PropertyTreeWriteArchive*)nullptr)) {
    PropertyTreeWriteArchive sub;
    sub.Accept(value);
    *tree = sub.tree();
  }

  template <typename Value>
  void VisitHelper(std::vector<Value>* value_vector,
                   ptree* tree, int) {
    DoArray(value_vector, tree);
  }

  template <typename Array>
  void DoArray(Array* array, ptree* tree) {
    for (std::size_t i = 0; i < array->size(); i++) {
      ptree element;
      VisitHelper(&(*array)[i], &element, 0);
      tree->push_back(ptree::value_type("", element));
    }
  }

  template <typename Value, size_t N>
  void VisitHelper(std::array<Value, N>* value_array,
                   ptree* tree, int) {
    DoArray(value_array, tree);
  }

  template <typename Value>
  void VisitHelper(Value* value, ptree* tree, long) {
    tree->put_value(*value);
  }

  boost::property_tree::ptree tree() const { return tree_; }

 private:
  boost::property_tree::ptree tree_;
};

class PropertyTreeReadArchive
    : public VisitArchive<PropertyTreeReadArchive> {
 public:
  enum {
    kErrorOnMissing = 1 << 0,
  };
  PropertyTreeReadArchive(const boost::property_tree::ptree& tree,
                          int flags=0)
      : tree_(tree),
        flags_(flags) {}

  template <typename NameValuePair>
  void VisitSerializable(const NameValuePair& pair) {
    if (!tree_.get_child_optional(pair.name())) {
      HandleMissing(pair.name());
      return;
    }
    PropertyTreeReadArchive(tree_.get_child(pair.name())).
        Accept(pair.value());
  }

  template <typename NameValuePair>
  void VisitEnumeration(const NameValuePair& pair) {
    auto optional_child = tree_.get_child_optional(pair.name());
    if (!optional_child) { HandleMissing(pair.name()); return; }
    auto child = *optional_child;
    // First, try to parse this as a stringified enumeration, then as
    // an integer if that fails.
    std::string maybe_enum = child.template get_value<std::string>();
    for (const auto& kp: pair.enumeration_mapper()) {
      if (kp.second == maybe_enum) {
        pair.set_value(static_cast<int>(kp.first));
        return;
      }
    }

    // No strings matched, so try treating it as a number.
    pair.set_value(child.template get_value<int>());
  }

  template <typename NameValuePair>
  void VisitScalar(const NameValuePair& pair) {
    typename std::decay<decltype(pair.get_value())>::type* dummy = nullptr;
    VisitHelper(pair, dummy, 0);
  }

  template <typename T>
  class FakeNvp {
   public:
    FakeNvp(const char* name, T* value): name_(name), value_(value) {}

    const char* name() const { return name_; }
    T* value() const { return value_; }
    const T& get_value() const { return *value_; }
    void set_value(const T& value) const { *value_ = value; }

   private:
    const char* const name_;
    T* const value_;
  };

  template <typename NameValuePair, typename T>
  void VisitHelper(const NameValuePair& pair,
                   std::vector<T>*,
                   int) {
    auto value_vector = pair.value();
    auto optional_child = tree_.get_child_optional(pair.name());
    if (!optional_child) { HandleMissing(pair.name()); return; }
    auto child = *optional_child;
    value_vector->clear();
    for (auto it = child.begin(); it != child.end(); ++it) {
      value_vector->push_back(T());
      PropertyTreeReadArchive(it->second).
          Visit(FakeNvp<T>("", &value_vector->back()));
    }
  }

  template <typename NameValuePair, typename T>
  void VisitHelper(const NameValuePair& pair,
                   boost::optional<T>*,
                   int) {
    auto optional = pair.value();
    auto optional_child = tree_.get_child_optional(pair.name());
    if (!optional_child) {
      *optional = boost::none;
      return;
    }
    auto child = *optional_child;
    if (child.template get_value<std::string>() == "null") {
      *optional = boost::none;
      return;
    }
    T value;
    PropertyTreeReadArchive(tree_).
        Visit(FakeNvp<T>(pair.name(), &value));
    *optional = value;
  }

  template <typename NameValuePair, typename T>
  void VisitHelper(const NameValuePair& pair,
                   T*,
                   long) {
    auto optional_child = tree_.get_child_optional(pair.name());
    if (!optional_child) { HandleMissing(pair.name()); return; }
    auto child = *optional_child;
    pair.set_value(child.template get_value<T>());
  }

 private:
  void HandleMissing(const char* name) {
    if (flags_ & kErrorOnMissing) {
      throw SystemError::einval(
          std::string("missing field: '") + name + "'");
    }
  }

  const boost::property_tree::ptree tree_;
  const int flags_;
};
}
}
