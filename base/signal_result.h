// Copyright 2015-2019 Josh Pieper, jjp@pobox.com.  All rights reserved.
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

#include <boost/signals2/signal.hpp>
#include <boost/signals2/connection.hpp>

#include "mjlib/io/deadline_timer.h"

#include "common.h"

namespace mjmech {
namespace base {
class TimeoutError : public std::runtime_error, public boost::exception {
 public:
  TimeoutError() : std::runtime_error("timeout") {}
  virtual ~TimeoutError() {}
};

/// This class presents a future which is satisfied when the given
/// signal emits, or when a timeout occurs, whichever occurs first.
class SignalResult : boost::noncopyable {
 public:

  template <typename Handler, typename T>
  static void Wait(boost::asio::io_service& service,
                   boost::signals2::signal<void (const T*)>* signal,
                   double timeout_s,
                   Handler handler) {
    struct Context {
      mjlib::io::DeadlineTimer timer;
      bool active = true;
      boost::signals2::connection connection;

      Context(boost::asio::io_service& service) : timer(service) {}
    };

    // TODO jpieper: It would be nice to implement this in a way that
    // didn't require a shared_ptr heap allocation for every wait.
    // However, the fact that asio's timer's have a two-stage enqueue,
    // then call semantics, means that it isn't possibly to reliably
    // cancel one.  Since this coroutine may go out of scope entirely,
    // along with its stack by the time the canceled timer callback is
    // invoked, this is the only way I can think of to ensure it
    // doesn't access anything it shouldn't.
    std::shared_ptr<Context> context(new Context(service));

    context->timer.expires_from_now(ConvertSecondsToDuration(timeout_s));
    context->timer.async_wait(
        [handler, context](
            boost::system::error_code ec) mutable -> void {
        if (ec == boost::asio::error::operation_aborted) { return; }
        if (!context->active) { return; }
        context->active = false;
        context->connection.disconnect();
        T result{};
        handler(boost::asio::error::operation_aborted, result);
      });

    context->connection =
        signal->connect([handler, context](
                            const T* value) mutable -> void {
            if (!context->active) { return; }
            context->active = false;
            context->connection.disconnect();
            handler(boost::system::error_code(), *value);
          });
  }
};
}
}
