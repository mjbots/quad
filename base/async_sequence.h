// Copyright 2019 Josh Pieper, jjp@pobox.com.  All rights reserved.
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

#include <boost/asio/io_service.hpp>

#include <memory>

#include "mjlib/base/error_code.h"
#include "mjlib/io/async_types.h"

namespace mjmech {
namespace base {

/// Creates a list of asynchronous operations which are invoked in
/// order.  If any produces an error, then the execution chain is
/// short-circuited and the error is immediately propagated to the
/// final callback.
class AsyncSequence {
 public:
  AsyncSequence(boost::asio::io_service& service) : service_(service) {}

  template <typename Step>
  AsyncSequence& Add(Step step,
                     const std::string_view& reason = std::string_view()) {
    AddItem(std::make_shared<ConcreteItem<Step>>(service_, step, reason));
    return *this;
  }

  void Start(mjlib::io::ErrorCallback callback) {
    BOOST_ASSERT(!!last_item_);
    AddItem(std::make_shared<LastItem>(service_, callback));
    service_.post([first = first_item_]() {
        first->Start();
      });
  }

 private:
  class Item {
   public:
    virtual ~Item() {}
    virtual void Start() = 0;
    virtual void Callback(const mjlib::base::error_code&) = 0;

    void SetNext(std::shared_ptr<Item> next) { next_ = next; }

    std::shared_ptr<Item> next_;
  };

  class LastItem : public Item {
   public:
    LastItem(boost::asio::io_service& service,
             mjlib::io::ErrorCallback callback) :
        service_(service),
        callback_(callback) {}
    ~LastItem() override {}

    void Start() override {
      callback_(mjlib::base::error_code());
    }

    void Callback(const mjlib::base::error_code& ec) override {
      callback_(ec);
    }

    boost::asio::io_service& service_;
    mjlib::io::ErrorCallback callback_;
  };

  template <typename T>
  class ConcreteItem : public Item,
                       public std::enable_shared_from_this<ConcreteItem<T>> {
   public:
    ConcreteItem(boost::asio::io_service& service,
                 T callback,
                 const std::string_view& reason)
        : service_(service),
          callback_(callback),
          reason_(reason) {}
    ~ConcreteItem() override {}

    void Start() override {
      callback_([st = this->shared_from_this()](auto ec) {
          if (ec) {
            ec.Append("When: " + st->reason_);
            st->next_->Callback(ec);
          } else {
            st->service_.post([st]() {
                st->next_->Start();
              });
          }
        });
    }

    void Callback(const mjlib::base::error_code& ec) override {
      next_->Callback(ec);
    }

    boost::asio::io_service& service_;
    T callback_;
    std::string reason_;
  };

  void AddItem(std::shared_ptr<Item> item) {
    if (last_item_) {
      last_item_->SetNext(item);
    }
    last_item_ = item;
    if (!first_item_) {
      first_item_ = item;
    }
  }

  boost::asio::io_service& service_;
  std::shared_ptr<Item> first_item_;
  std::shared_ptr<Item> last_item_;
};

}
}
