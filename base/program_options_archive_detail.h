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

#include <type_traits>

namespace mjmech {
namespace base {

namespace detail {
template <typename NameValuePair>
class ProgramOptionsArchiveValue : public boost::program_options::value_semantic {
 public:
  ProgramOptionsArchiveValue(const NameValuePair& nvp) : nvp_(nvp) {}
  virtual ~ProgramOptionsArchiveValue() {}

  virtual std::string name() const override { return ""; }
  virtual unsigned min_tokens() const override { return 1; }
  virtual unsigned max_tokens() const override { return 1; }
  virtual bool is_composing() const override { return false; }
  virtual bool is_required() const override { return false; }
  virtual void parse(boost::any& value_store,
                     const std::vector<std::string>& new_tokens,
                     bool utf8) const override {
    value_store = boost::lexical_cast<
      typename std::decay<decltype(nvp_.get_value())>::type>(new_tokens.at(0));
  }

  virtual bool apply_default(boost::any&) const override {
    return false;
  }

  virtual void notify(const boost::any& value_store) const override {
    if (value_store.empty()) { return; }
    nvp_.set_value(boost::any_cast<decltype(nvp_.get_value())>(value_store));
  }

#if BOOST_VERSION >= 106100 && BOOST_VERSION <= 106400
  virtual bool adjacent_tokens_only() const override {
    return false;
  }
#endif

 private:
  NameValuePair nvp_;
};

}

}
}
