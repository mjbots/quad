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

#include <functional>
#include <vector>

#include <boost/asio/buffer.hpp>
#include <boost/asio/io_service.hpp>
#include <boost/system/error_code.hpp>

#include "async_types.h"
#include "error_code.h"

namespace mjmech {
namespace base {
typedef std::function<void (ErrorCode, std::size_t)> SizeHandler;
typedef SizeHandler ReadHandler;
typedef SizeHandler WriteHandler;

template <typename Buffer>
class BufferSequence {
 public:
  typedef Buffer value_type;
  typedef typename std::vector<value_type>::const_iterator const_iterator;

  BufferSequence() {}

  template <typename Sequence>
  BufferSequence(const Sequence& s) : data_(s.begin(), s.end()) {}

  const_iterator begin() const { return data_.begin(); }
  const_iterator end() const { return data_.end(); }

 protected:
  std::vector<value_type> data_;
};

class MutableBufferSequence
    : public BufferSequence<boost::asio::mutable_buffer> {
 public:
  MutableBufferSequence() {}

  template <typename Buffers>
  MutableBufferSequence(const Buffers& t)
  : BufferSequence<boost::asio::mutable_buffer>(t) {}
};

class ConstBufferSequence : public BufferSequence<boost::asio::const_buffer> {
 public:
  ConstBufferSequence() {}

  template <typename Buffers>
  ConstBufferSequence(const Buffers& t)
  : BufferSequence<boost::asio::const_buffer>(t) {}
};

class AsyncStream : boost::noncopyable {
 public:
  virtual ~AsyncStream() {}
  virtual boost::asio::io_service& get_io_service() = 0;

  template <typename Buffers, typename Handler>
  void async_read_some(Buffers buffers, Handler handler) {
    this->virtual_async_read_some(
        buffers,
        [handler](ErrorCode ec, std::size_t size) mutable {
          handler(ec.error_code(), size);
        });
  }

  template <typename Buffers, typename Handler>
  void async_write_some(Buffers buffers, Handler handler) {
    this->virtual_async_write_some(
        buffers, [handler](ErrorCode ec, std::size_t size) mutable {
          handler(ec.error_code(), size);
        });
  }

  virtual void cancel() = 0;

  virtual void virtual_async_read_some(MutableBufferSequence, ReadHandler) = 0;
  virtual void virtual_async_write_some(ConstBufferSequence, WriteHandler) = 0;
};

typedef std::shared_ptr<AsyncStream> SharedStream;
typedef std::function<void (ErrorCode, SharedStream)> StreamHandler;

}
}
