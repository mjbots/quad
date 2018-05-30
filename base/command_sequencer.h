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

#include <list>

#include <boost/asio/io_service.hpp>

namespace mjmech {
namespace base {
/// Ensure that a given resource is operated on by at most one command
/// at a time.
class CommandSequencer : boost::noncopyable {
 public:
  CommandSequencer(boost::asio::io_service& service) : service_(service) {}

  template <typename Command, typename Handler>
  void Invoke(Command command, Handler handler) {
    auto ptr = std::shared_ptr<Base>(
        new Concrete<Command, Handler>(this, command, handler));
    queued_.push_back(ptr);
    MaybeStart();
  };

  boost::asio::io_service& get_io_service() { return service_; }

 private:
  void WaitingFinished() {
    BOOST_ASSERT(waiting_);
    waiting_.reset();
    MaybeStart();
  }

  void MaybeStart() {
    if (waiting_) { return; }
    if (queued_.empty()) { return; }
    waiting_ = queued_.front();
    queued_.pop_front();
    waiting_->Invoke();
  }

  class Base : boost::noncopyable {
   public:
    virtual ~Base() {}
    virtual void Invoke() = 0;
  };

  template <typename Command, typename Handler>
  class Concrete : public Base {
   public:
    Concrete(CommandSequencer* parent,
             const Command& command,
             const Handler& handler)
        : parent_(parent), command_(command), handler_(handler) {}

    virtual ~Concrete() {}

    virtual void Invoke() {
      command_(HandlerWrapper(this));
    }

   private:
    class HandlerWrapper {
     public:
      HandlerWrapper(Concrete* concrete) : concrete_(concrete) {}

      template <typename... Args>
      void operator()(Args&&... args) {
        concrete_->handler_(std::forward<Args>(args)...);

        BOOST_ASSERT(concrete_->parent_->waiting_);
        concrete_->parent_->get_io_service().post(
            std::bind(&CommandSequencer::WaitingFinished, concrete_->parent_));
      }

     private:
      Concrete* const concrete_;
    };

    CommandSequencer* const parent_;
    Command command_;
    Handler handler_;
  };

  boost::asio::io_service& service_;
  std::shared_ptr<Base> waiting_;
  std::list<std::shared_ptr<Base>> queued_;
};
}
}
