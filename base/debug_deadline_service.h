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

#include <map>

#include "virtual_deadline_timer.h"

namespace mjmech {
namespace base {

class DebugDeadlineService : public VirtualDeadlineTimerService {
 public:
  DebugDeadlineService(boost::asio::io_service& service)
      : service_(service) {}
  virtual ~DebugDeadlineService() {}

  static DebugDeadlineService* Install(boost::asio::io_service& service) {
    auto& deadline_service =
        boost::asio::use_service<VirtualDeadlineTimerServiceHolder>(service);
    DebugDeadlineService* result = new DebugDeadlineService(service);
    deadline_service.Reset(
        std::unique_ptr<VirtualDeadlineTimerService>(result));
    return result;
  }

  static DebugDeadlineService* Get(boost::asio::io_service& service) {
    auto& deadline_service =
        boost::asio::use_service<VirtualDeadlineTimerServiceHolder>(service);
    return dynamic_cast<DebugDeadlineService*>(deadline_service.Get());
  }

  void construct(implementation_type& impl) override {
    Impl* data = nullptr;
    impl = data = new Impl();
    data->position_ = queue_.end();
  }

  void destroy(implementation_type& impl) override {
    delete impl;
    impl = nullptr;
  }

  std::size_t cancel(implementation_type& impl,
                     boost::system::error_code& ec) override {
    std::size_t result = 0;
    auto& item = data(impl);
    if (item.position_ != queue_.end()) {
      result++;
      auto& entry = item.position_->second;
      BOOST_ASSERT(&item == entry.item);
      service_.post(
          std::bind(entry.handler, boost::asio::error::operation_aborted));
      queue_.erase(item.position_);
      item.position_ = queue_.end();
    }
    return result;
  }

  boost::posix_time::ptime expires_at(
      const implementation_type& impl) const override {
    auto& item = data(impl);
    return item.timestamp_;
  }

  std::size_t expires_at(implementation_type& impl,
                         boost::posix_time::ptime timestamp,
                         boost::system::error_code& ec) override {
    std::size_t result = cancel(impl, ec);
    if (ec) { return result; }

    auto& item = data(impl);
    item.timestamp_ = timestamp;
    return result;
  }

  boost::posix_time::time_duration expires_from_now(
      const implementation_type& impl) const override {
    auto& item = data(impl);
    return item.timestamp_ - now();
  }

  std::size_t expires_from_now(
      implementation_type& impl,
      boost::posix_time::time_duration duration,
      boost::system::error_code& ec) override {
    return expires_at(impl, now() + duration, ec);
  }

  boost::system::error_code wait(
      implementation_type& impl,
      boost::system::error_code& ec) override {
    BOOST_ASSERT(false);
  }

  void async_wait(implementation_type& impl,
                  ErrorHandler handler) override {
    auto& item = data(impl);
    Entry entry;
    entry.item = &item;
    entry.handler = handler;
    item.position_ = queue_.insert(
        std::make_pair(item.timestamp_, entry));
  }

  boost::posix_time::ptime now() const override {
    return current_time_;
  }

  void shutdown_service() override {
  }

  void SetTime(boost::posix_time::ptime new_time) {
    current_time_ = new_time;

    while (!queue_.empty() && queue_.begin()->first <= current_time_) {
      auto& entry = queue_.begin()->second;
      auto& item = *entry.item;
      item.position_ = queue_.end();
      service_.post(
          std::bind(entry.handler, boost::system::error_code()));
      queue_.erase(queue_.begin());
    }
  }

 private:
  class Impl;

  struct Entry {
    Impl* item = nullptr;
    ErrorHandler handler;
  };
  typedef std::multimap<boost::posix_time::ptime, Entry> Queue;

  class Impl : public VirtualDeadlineTimerImpl {
   public:
    virtual ~Impl() {}

    boost::posix_time::ptime timestamp_;
    Queue::iterator position_;
  };

  Impl& data(implementation_type& impl) {
    Impl* result = dynamic_cast<Impl*>(impl);
    BOOST_ASSERT(result);
    return *result;
  }

  const Impl& data(const implementation_type& impl) const {
    const Impl* result = dynamic_cast<const Impl*>(impl);
    BOOST_ASSERT(result);
    return *result;
  }

  boost::asio::io_service& service_;
  boost::posix_time::ptime current_time_;
  Queue queue_;
};

}
}
