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

#include <boost/asio/basic_deadline_timer.hpp>
#include <boost/asio/io_service.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <boost/noncopyable.hpp>

#include "async_types.h"

namespace mjmech {
namespace base {

class VirtualDeadlineTimerImpl : boost::noncopyable {
 public:
  virtual ~VirtualDeadlineTimerImpl() {}
};

class VirtualDeadlineTimerService {
 public:
  typedef VirtualDeadlineTimerImpl* implementation_type;

  virtual ~VirtualDeadlineTimerService() {}

  virtual void construct(implementation_type&) = 0;
  virtual void destroy(implementation_type&) = 0;
  virtual std::size_t cancel(implementation_type&,
                             boost::system::error_code&) = 0;
  virtual boost::posix_time::ptime expires_at(
      const implementation_type&) const = 0;
  virtual std::size_t expires_at(
      implementation_type&,
      boost::posix_time::ptime,
      boost::system::error_code&) = 0;
  virtual boost::posix_time::time_duration expires_from_now(
      const implementation_type&) const = 0;
  virtual std::size_t expires_from_now(
      implementation_type&,
      boost::posix_time::time_duration,
      boost::system::error_code&) = 0;
  virtual boost::system::error_code wait(
      implementation_type&,
      boost::system::error_code&) = 0;
  virtual void async_wait(
      implementation_type&,
      ErrorHandler) = 0;
  virtual void shutdown_service() = 0;
  virtual boost::posix_time::ptime now() const = 0;
};

class AsioTimerService : public VirtualDeadlineTimerService {
 public:
  AsioTimerService(boost::asio::io_service& service) : service_(service) {}

  void construct(implementation_type& impl) override {
    impl = new Impl();
    base().construct(data(impl));
  }

  void destroy(implementation_type& impl) override {
    base().destroy(data(impl));
    delete impl;
    impl = nullptr;
  }

  std::size_t cancel(implementation_type& impl,
                     boost::system::error_code& ec) override {
    return base().cancel(data(impl), ec);
  }

  boost::posix_time::ptime expires_at(
      const implementation_type& impl) const override {
    return base().expires_at(data(impl));
  }

  std::size_t expires_at(
      implementation_type& impl,
      boost::posix_time::ptime timestamp,
      boost::system::error_code& ec) override {
    return base().expires_at(data(impl), timestamp, ec);
  }

  boost::posix_time::time_duration expires_from_now(
      const implementation_type& impl) const override {
    return base().expires_from_now(data(impl));
  }

  std::size_t expires_from_now(
      implementation_type& impl,
      boost::posix_time::time_duration duration,
      boost::system::error_code& ec) override {
    return base().expires_from_now(data(impl), duration, ec);
  }

  boost::system::error_code wait(
      implementation_type& impl,
      boost::system::error_code& ec) {
    base().wait(data(impl), ec);
    return ec;
  }

  void async_wait(implementation_type& impl,
                  ErrorHandler handler) override {
    base().async_wait(data(impl), handler);
  }

  void shutdown_service() override {
  }

  boost::posix_time::ptime now() const override {
    return boost::posix_time::microsec_clock::universal_time();
  }

 private:
  typedef boost::asio::deadline_timer_service<
   boost::posix_time::ptime> Base;

  class Impl : public VirtualDeadlineTimerImpl {
   public:
    virtual ~Impl() {}
    Base::implementation_type data;
  };

  Base& base() {
    return boost::asio::use_service<Base>(service_);
  }

  const Base& base() const {
    return boost::asio::use_service<Base>(service_);
  }

  Base::implementation_type& data(implementation_type& impl) {
    return dynamic_cast<Impl*>(impl)->data;
  }

  const Base::implementation_type& data(
      const implementation_type& impl) const {
    return dynamic_cast<const Impl*>(impl)->data;
  }

  boost::asio::io_service& service_;
};

class VirtualDeadlineTimerServiceHolder
    : public boost::asio::io_service::service {
 public:
  typedef VirtualDeadlineTimerImpl* implementation_type;
  typedef boost::posix_time::ptime time_type;
  typedef boost::posix_time::time_duration duration_type;

  VirtualDeadlineTimerServiceHolder(boost::asio::io_service& service)
      : boost::asio::io_service::service(service),
        child_(new AsioTimerService(service)) {}

  void Reset(std::unique_ptr<VirtualDeadlineTimerService> child) {
    child_ = std::move(child);
  }

  VirtualDeadlineTimerService* Get() {
    return child_.get();
  }

  void construct(implementation_type& impl) {
    child_->construct(impl);
  }

  void destroy(implementation_type& impl) {
    child_->destroy(impl);
  }

  std::size_t cancel(implementation_type& impl,
                     boost::system::error_code& ec) {
    return child_->cancel(impl, ec);
  }

  duration_type expires_from_now(const implementation_type& impl) {
    return child_->expires_from_now(impl);
  }

  std::size_t expires_from_now(implementation_type& impl,
                               const duration_type& duration,
                               boost::system::error_code& ec) {
    return child_->expires_from_now(impl, duration, ec);
  }

  time_type expires_at(const implementation_type& impl) {
    return child_->expires_at(impl);
  }

  std::size_t expires_at(implementation_type& impl,
                         const time_type& timestamp,
                         boost::system::error_code& ec) {
    return child_->expires_at(impl, timestamp, ec);
  }

  boost::system::error_code wait(implementation_type& impl,
                                 boost::system::error_code& ec) {
    return child_->wait(impl, ec);
  }

  template <typename Handler>
  BOOST_ASIO_INITFN_RESULT_TYPE(Handler,
                                void(boost::system::error_code))
  async_wait(implementation_type& impl, Handler handler) {
    boost::asio::detail::async_result_init<
      Handler,
      void (boost::system::error_code)> init(
          BOOST_ASIO_MOVE_CAST(Handler)(handler));

    this->child_->async_wait(
        impl,
        [init](ErrorCode ec) mutable {
          init.handler(ec.error_code());
        });

    return init.result.get();
  }

  void shutdown_service() override {
    child_->shutdown_service();
  }

  boost::posix_time::ptime now() const {
    return child_->now();
  }

  static boost::asio::io_service::id id;

 private:
  std::unique_ptr<VirtualDeadlineTimerService> child_;
};
}
}
