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

// TODO jpieper: Audit internal callbacks to make sure they are
// shared_ptr safe even if the parent container is deleted but the
// io_service is still running.

#include "comm_factory.h"

#include <boost/algorithm/string.hpp>
#include <boost/asio/buffers_iterator.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/posix/stream_descriptor.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/format.hpp>

namespace mjmech {
namespace base {
namespace {
using namespace std::placeholders;

class StdioStream : public AsyncStream {
 public:
  StdioStream(boost::asio::io_service& service,
              const StdioGenerator::Parameters& parameters)
      : service_(service),
        stdin_(service, ::dup(parameters.in)),
        stdout_(service, ::dup(parameters.out)) {}

  virtual boost::asio::io_service& get_io_service() { return service_; }

  virtual void virtual_async_read_some(MutableBufferSequence buffers,
                                       ReadHandler handler) {
    stdin_.async_read_some(buffers, handler);
  }

  virtual void virtual_async_write_some(ConstBufferSequence buffers,
                                        WriteHandler handler) {
    stdout_.async_write_some(buffers, handler);
  }

  virtual void cancel() {
    stdin_.cancel();
    stdout_.cancel();
  }

 private:
  boost::asio::io_service& service_;
  boost::asio::posix::stream_descriptor stdin_;
  boost::asio::posix::stream_descriptor stdout_;
};
}

void StdioGenerator::AsyncCreate(
    boost::asio::io_service& service,
    const Parameters& parameters,
    StreamHandler handler) {
  service.post(
      std::bind(handler, ErrorCode(),
                std::shared_ptr<AsyncStream>(
                    new StdioStream(service, parameters))));
}

namespace {
class SerialStream : public AsyncStream {
 public:
  SerialStream(boost::asio::io_service& service,
               const SerialPortGenerator::Parameters& parameters)
      : service_(service),
        port_(service, parameters.serial_port) {
    port_.set_option(
        boost::asio::serial_port_base::baud_rate(parameters.baud_rate));
    port_.set_option(
        boost::asio::serial_port_base::character_size(parameters.data_bits));
    auto make_parity = [](std::string string) {
      boost::to_lower(string);
      if (string == "n" || string == "none") {
        return boost::asio::serial_port_base::parity::none;
      } else if (string == "e" || string == "even") {
        return boost::asio::serial_port_base::parity::even;
      } else if (string == "o" || string == "odd") {
        return boost::asio::serial_port_base::parity::odd;
      }
      throw SystemError::einval("unknown parity: " + string);
    };
    port_.set_option(
        boost::asio::serial_port_base::parity(make_parity(parameters.parity)));
  }

  virtual boost::asio::io_service& get_io_service() { return service_; }

  virtual void virtual_async_read_some(MutableBufferSequence buffers,
                                       ReadHandler handler) {
    port_.async_read_some(buffers, handler);
  }

  virtual void virtual_async_write_some(ConstBufferSequence buffers,
                                        WriteHandler handler) {
    port_.async_write_some(buffers, handler);
  }

  virtual void cancel() {
    port_.cancel();
  }

 private:
  boost::asio::io_service& service_;
  boost::asio::serial_port port_;
};
}

void SerialPortGenerator::AsyncCreate(
    boost::asio::io_service& service,
    const Parameters& parameters,
    StreamHandler handler) {
  auto stream = SharedStream(new SerialStream(service, parameters));
  service.post(std::bind(handler, ErrorCode(), stream));
}

namespace {
typedef boost::asio::ip::tcp tcp;

class TcpStream : public AsyncStream {
 public:
  TcpStream(boost::asio::io_service& service,
            const TcpClientGenerator::Parameters& parameters)
      : service_(service),
        parameters_(parameters),
        resolver_(service),
        socket_(service) {
    tcp::resolver::query query(parameters.host,
                               (boost::format("%d") % parameters.port).str());
    resolver_.async_resolve(
        query, std::bind(&TcpStream::HandleResolve, this, _1, _2));
  }

  virtual boost::asio::io_service& get_io_service() { return service_; }

  virtual void virtual_async_read_some(MutableBufferSequence buffers,
                                       ReadHandler handler) {
    socket_.async_read_some(buffers, handler);
  }

  virtual void virtual_async_write_some(ConstBufferSequence buffers,
                                        WriteHandler handler) {
    socket_.async_write_some(buffers, handler);
  }

  virtual void cancel() {
    resolver_.cancel();
    socket_.cancel();
  }

  ErrorHandler start_handler_;

 private:
  void HandleResolve(ErrorCode ec,
                     tcp::resolver::iterator it) {
    if (ec) {
      ec.Append(boost::format("when resolving: %s:%d") %
                parameters_.host % parameters_.port);
      service_.post(std::bind(start_handler_, ec));
      return;
    }

    socket_.async_connect(*it, std::bind(&TcpStream::HandleConnect, this, _1));
  }

  void HandleConnect(ErrorCode ec) {
    if (ec) {
      ec.Append(boost::format("when connecting to: %s:%d") %
                parameters_.host % parameters_.port);
    }
    service_.post(std::bind(start_handler_, ec));
  }

  boost::asio::io_service& service_;
  const TcpClientGenerator::Parameters parameters_;
  tcp::resolver resolver_;
  tcp::socket socket_;
};
}

void TcpClientGenerator::AsyncCreate(
    boost::asio::io_service& service,
    const Parameters& parameters,
    StreamHandler handler) {
  TcpStream* tcp_stream = nullptr;
  std::shared_ptr<AsyncStream> stream(
      (tcp_stream = new TcpStream(service, parameters)));
  tcp_stream->start_handler_ = std::bind(handler, _1, stream);
}

namespace {
class TcpServerStream : public AsyncStream {
 public:
  TcpServerStream(boost::asio::io_service& service,
                  const TcpServerGenerator::Parameters& parameters)
      : service_(service),
        parameters_(parameters),
        acceptor_(service),
        socket_(service) {
    tcp::endpoint endpoint(tcp::v4(), parameters_.port);
    acceptor_.open(endpoint.protocol());
    acceptor_.set_option(tcp::acceptor::reuse_address(true));
    acceptor_.bind(endpoint);
    acceptor_.listen();

    Accept();
  }

  void Accept() {
    acceptor_.async_accept(
        socket_,
        std::bind(&TcpServerStream::HandleAccept, this, _1));
  }

  virtual boost::asio::io_service& get_io_service() { return service_; }

  virtual void virtual_async_read_some(MutableBufferSequence buffers,
                                       ReadHandler handler) {
    BOOST_ASSERT(started_);

    read_buffers_ = buffers;
    read_handler_ = handler;

    if (connected_) {
      socket_.async_read_some(
          buffers, std::bind(&TcpServerStream::HandleRead, this, _1, _2));
    } else {
      read_queued_ = true;
    }
  }

  virtual void virtual_async_write_some(ConstBufferSequence buffers,
                                        WriteHandler handler) {
    BOOST_ASSERT(started_);

    write_buffers_ = buffers;
    write_handler_ = handler;

    if (connected_) {
      socket_.async_write_some(
          buffers, std::bind(&TcpServerStream::HandleWrite, this, _1, _2));
    } else {
      write_queued_ = true;
    }
  }

  virtual void cancel() {
    BOOST_ASSERT(started_);

    if (connected_) {
      socket_.cancel();
    } else {
      if (read_queued_) {
        read_queued_ = false;
        service_.post(
            std::bind(read_handler_,
                      boost::asio::error::operation_aborted, 0));
      }
      if (write_queued_) {
        write_queued_ = false;
        service_.post(
            std::bind(write_handler_,
                      boost::asio::error::operation_aborted, 0));
      }
    }
  }

  ErrorHandler start_handler_;

 private:
  void HandleAccept(const boost::system::error_code& ec) {
    if (!started_) {
      service_.post(std::bind(start_handler_, ec));
      started_ = true;
    }
    connected_ = true;

    if (read_queued_) {
      read_queued_ = false;
      virtual_async_read_some(read_buffers_, read_handler_);
    }
    if (write_queued_) {
      write_queued_ = false;
      virtual_async_write_some(write_buffers_, write_handler_);
    }
  }

  void HandleRead(const boost::system::error_code& ec,
                  std::size_t size) {
    if (ec == boost::asio::error::eof) {
      read_queued_ = true;
      connected_ = false;
      socket_.close();
      Accept();
    } else {
      service_.post(std::bind(read_handler_, ec, size));
    }
  }

  void HandleWrite(const boost::system::error_code& ec,
                   std::size_t size) {
    if (ec == boost::asio::error::eof) {
      write_queued_ = true;
      connected_ = false;
      socket_.close();
      Accept();
    } else {
      service_.post(std::bind(write_handler_, ec, size));
    }
  }

  boost::asio::io_service& service_;
  const TcpServerGenerator::Parameters parameters_;
  tcp::acceptor acceptor_;
  tcp::socket socket_;
  bool started_ = false;
  bool connected_ = false;

  MutableBufferSequence read_buffers_;
  ReadHandler read_handler_;
  bool read_queued_ = false;

  ConstBufferSequence write_buffers_;
  WriteHandler write_handler_;
  bool write_queued_ = false;
};
}

void TcpServerGenerator::AsyncCreate(
    boost::asio::io_service& service,
    const Parameters& parameters,
    StreamHandler handler) {
  TcpServerStream* server_stream = nullptr;
  std::shared_ptr<AsyncStream> stream(
      (server_stream = new TcpServerStream(service, parameters)));
  server_stream->start_handler_ = std::bind(handler, _1, stream);
}

namespace {
class HalfPipe : public AsyncStream {
 public:
  HalfPipe(boost::asio::io_service& service)
      : service_(service) {}
  virtual ~HalfPipe() {}

  void SetOther(HalfPipe* other) { other_ = other; }

  virtual boost::asio::io_service& get_io_service() { return service_; }
  virtual void virtual_async_read_some(MutableBufferSequence buffers,
                                       ReadHandler handler) {
    BOOST_ASSERT(other_);
    BOOST_ASSERT(!read_handler_);

    if (boost::asio::buffer_size(buffers) == 0) {
      // Post immediately.
      service_.post(
          std::bind(handler, ErrorCode(), 0));
      return;
    }

    if (other_->write_handler_) {
      const std::size_t written =
          boost::asio::buffer_copy(buffers, *other_->write_buffers_);
      service_.post(
          std::bind(*other_->write_handler_, ErrorCode(), written));
      service_.post(
          std::bind(handler, ErrorCode(), written));

      other_->write_handler_ = boost::none;
      other_->write_buffers_ = boost::none;
    } else {
      read_buffers_ = buffers;
      read_handler_ = handler;
    }
  }

  virtual void virtual_async_write_some(ConstBufferSequence buffers,
                                        WriteHandler handler) {
    BOOST_ASSERT(other_);
    BOOST_ASSERT(!write_handler_);

    if (boost::asio::buffer_size(buffers) == 0) {
      // Post immediately.
      service_.post(
          std::bind(handler, ErrorCode(), 0));
      return;
    }

    if (other_->read_handler_) {
      const std::size_t written =
          boost::asio::buffer_copy(*other_->read_buffers_, buffers);
      service_.post(
          std::bind(*other_->read_handler_, ErrorCode(), written));
      service_.post(
          std::bind(handler, ErrorCode(), written));

      other_->read_handler_ = boost::none;
      other_->read_buffers_ = boost::none;
    } else {
      write_buffers_ = buffers;
      write_handler_ = handler;
    }
  }

  virtual void cancel() {
    if (read_handler_) {
      service_.post(
          std::bind(*read_handler_, boost::asio::error::operation_aborted, 0));
      read_handler_ = boost::none;
      read_buffers_ = boost::none;
    }

    if (write_handler_) {
      service_.post(
          std::bind(*write_handler_,
                    boost::asio::error::operation_aborted, 0));
      write_handler_ = boost::none;
      write_buffers_ = boost::none;
    }
  }

 private:
  boost::asio::io_service& service_;
  HalfPipe* other_ = nullptr;

  boost::optional<MutableBufferSequence> read_buffers_;
  boost::optional<ReadHandler> read_handler_;

  boost::optional<ConstBufferSequence> write_buffers_;
  boost::optional<WriteHandler> write_handler_;
};

class BidirectionalPipe : boost::noncopyable {
 public:
  BidirectionalPipe(boost::asio::io_service& service)
      : direction_a_(service),
        direction_b_(service) {
    direction_a_.SetOther(&direction_b_);
    direction_b_.SetOther(&direction_a_);
  }

  HalfPipe direction_a_;
  HalfPipe direction_b_;
};

/// A reference to a HalfPipe which also maintains the shared pointer
/// ownership of its parent.  This ensures that the parent stays
/// around as long as any callers have a stream.
class HalfPipeRef : public AsyncStream {
 public:
  HalfPipeRef(std::shared_ptr<BidirectionalPipe> parent,
              HalfPipe* pipe)
      : parent_(parent),
        pipe_(pipe) {}

  virtual boost::asio::io_service& get_io_service() {
    return pipe_->get_io_service();
  }

  virtual void cancel() { pipe_->cancel(); }

  virtual void virtual_async_read_some(MutableBufferSequence buffers,
                                       ReadHandler handler) {
    return pipe_->virtual_async_read_some(buffers, handler);
  }

  virtual void virtual_async_write_some(ConstBufferSequence buffers,
                                        WriteHandler handler) {
    return pipe_->virtual_async_write_some(buffers, handler);
  }

 private:
  std::shared_ptr<BidirectionalPipe> parent_;
  HalfPipe* const pipe_;
};

}

class PipeGenerator::Impl : boost::noncopyable {
 public:
  std::map<std::string, std::shared_ptr<BidirectionalPipe>> pipes_;
};

PipeGenerator::PipeGenerator() : impl_(new Impl()) {}
PipeGenerator::~PipeGenerator() {}

void PipeGenerator::AsyncCreate(boost::asio::io_service& service,
                                const Parameters& parameters,
                                StreamHandler handler) {
  SharedStream stream = GetStream(service, parameters.key, Mode::kDirectionA);
  service.post(std::bind(handler, ErrorCode(), stream));
}

SharedStream PipeGenerator::GetStream(
    boost::asio::io_service& service,
    const std::string& key,
    Mode mode) {
  if (!impl_->pipes_.count(key)) {
    impl_->pipes_.insert(
        std::make_pair(key,
                       std::shared_ptr<BidirectionalPipe>(
                           new BidirectionalPipe(service))));
  }

  std::shared_ptr<BidirectionalPipe> parent = impl_->pipes_[key];
  return SharedStream(
      new HalfPipeRef(
          parent,
          (mode == Mode::kDirectionA ?
           &parent->direction_a_ : &parent->direction_b_)));
}

namespace {
class StdioDebugStream : public AsyncStream {
 public:
  StdioDebugStream(SharedStream base) : context_(new Context()) {
    context_->base = base;
  }

  virtual ~StdioDebugStream() {}
  virtual boost::asio::io_service& get_io_service() {
    return context_->base->get_io_service();
  }

  virtual void cancel() { context_->base->cancel(); }

  virtual void virtual_async_read_some(
      MutableBufferSequence buffer, ReadHandler handler) {

    context_->base->virtual_async_read_some(
        buffer,
        [context=this->context_, buffer, handler](
            ErrorCode ec, std::size_t size) {

          if (size != 0) {
            if (context->last_write) {
              std::cout << "\nREAD:";
              context->last_write = false;
            }

            typedef boost::asio::buffers_iterator<
              MutableBufferSequence> Iterator;
            std::size_t count = 0;
            for (auto it = Iterator::begin(buffer);
                 count < size; ++it, ++count) {
              std::cout << boost::format(" %02X") %
                  static_cast<int>(static_cast<uint8_t>(*it));
            }
          }

          handler(ec, size);
        });
  }

  virtual void virtual_async_write_some(
      ConstBufferSequence buffer, WriteHandler handler) {
    context_->base->virtual_async_write_some(
        buffer,
        [context=this->context_, buffer, handler](
            ErrorCode ec, std::size_t size) {
          if (size != 0) {
            if (!context->last_write) {
              std::cout << "\nWRITE:";
              context->last_write = true;
            }

            typedef boost::asio::buffers_iterator<
              ConstBufferSequence> Iterator;
            std::size_t count = 0;
            for (auto it = Iterator::begin(buffer);
                 count < size; ++it, ++count) {
              std::cout << boost::format(" %02X") %
                  static_cast<int>(static_cast<uint8_t>(*it));
            }
          }

          handler(ec, size);
        });
  }

 private:
  struct Context {
    SharedStream base;
    bool last_write = false;
  };
  std::shared_ptr<Context> context_;
};
}

SharedStream MakeStdioDebugStream(SharedStream stream) {
  return SharedStream(new StdioDebugStream(stream));
}

}
}
