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

#include "program_options.h"

namespace mjmech {
namespace base {

namespace {
class ProgramOptionsArchiveWrapValue :
      public boost::program_options::value_semantic {
 public:
  ProgramOptionsArchiveWrapValue(
      boost::shared_ptr<const boost::program_options::value_semantic> base)
      : base_(base) {}

  virtual ~ProgramOptionsArchiveWrapValue() {}

  virtual std::string name() const override { return base_->name(); }
  virtual unsigned min_tokens() const override { return base_->min_tokens(); }
  virtual unsigned max_tokens() const override { return base_->max_tokens(); }
  virtual bool is_composing() const override { return base_->is_composing(); }
  virtual bool is_required() const override { return base_->is_required(); }
  virtual void parse(boost::any& value_store,
                     const std::vector<std::string>& new_tokens,
                     bool utf8) const override {
    base_->parse(value_store, new_tokens, utf8);
  }

  virtual bool apply_default(boost::any& value) const override {
    return base_->apply_default(value);
  }

  virtual void notify(const boost::any& value_store) const override {
    base_->notify(value_store);
  }
#if BOOST_VERSION >= 106100 && BOOST_VERSION <= 106400
  virtual bool adjacent_tokens_only() const override {
    return base_->adjacent_tokens_only();
  }
#endif
 private:
  const boost::shared_ptr<const boost::program_options::value_semantic> base_;
};
}

void MergeProgramOptions(
    boost::program_options::options_description* source,
    const std::string& destination_prefix,
    boost::program_options::options_description* destination) {
  for (auto option: source->options()) {
    (*destination).add_options()(
        (destination_prefix + option->long_name()).c_str(),
        new ProgramOptionsArchiveWrapValue(option->semantic()),
        option->description().c_str());
  }
}

void SetOption(
    boost::program_options::options_description* source,
    const std::string& key,
    const std::string& value) {
  auto semantic = source->find(key, false).semantic();
  boost::any any_value;
  semantic->parse(any_value, std::vector<std::string>({value}), true);
  semantic->notify(any_value);
}

}
}
