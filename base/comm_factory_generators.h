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

#include <boost/asio/io_service.hpp>

#include "base/comm.h"
#include "base/visitor.h"

namespace mjmech {
namespace base {

/// A Generator to make stdio backed streams.
class StdioGenerator : boost::noncopyable {
 public:
  struct Parameters {
    int in = 0;
    int out = 1;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(in));
      a->Visit(MJ_NVP(out));
    }
  };

  static const char* type() { return "stdio"; }

  static void AsyncCreate(
      boost::asio::io_service&,
      const Parameters&,
      StreamHandler handler);
};

/// A Generator to make serial port backed streams.
class SerialPortGenerator : boost::noncopyable {
 public:
  struct Parameters {
    std::string serial_port;
    int baud_rate = 115200;
    std::string parity = "n";
    int data_bits = 8;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(serial_port));
      a->Visit(MJ_NVP(baud_rate));
      a->Visit(MJ_NVP(parity));
      a->Visit(MJ_NVP(data_bits));
    }
  };

  static const char* type() { return "serial"; }

  static void AsyncCreate(
      boost::asio::io_service&,
      const Parameters&,
      StreamHandler handler);
};

/// A Generator to make TCP client backed streams.
class TcpClientGenerator : boost::noncopyable {
 public:
  struct Parameters {
    std::string host;
    int port = 0;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(host));
      a->Visit(MJ_NVP(port));
    }
  };

  static const char* type() { return "tcp"; }

  static void AsyncCreate(
      boost::asio::io_service&,
      const Parameters&,
      StreamHandler handler);
};

/// A Generate to make TCP server backed streams.
class TcpServerGenerator : boost::noncopyable {
 public:
  struct Parameters {
    int port = 0;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(port));
    }
  };

  static const char* type() { return "tcp_server"; }

  static void AsyncCreate(
      boost::asio::io_service&,
      const Parameters&,
      StreamHandler handler);

};

/// Serves up bi-direction synthetic streams.  The AsyncCreate method
/// always returns streams of kDirectionA.  GetStream users may get
/// streams of either direction.
class PipeGenerator : boost::noncopyable {
 public:
  struct Parameters {
    std::string key;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(key));
    }
  };

  static const char* type() { return "pipe"; }

  PipeGenerator();
  ~PipeGenerator();

  void AsyncCreate(boost::asio::io_service&,
                   const Parameters&,
                   StreamHandler handler);

  enum class Mode {
    kDirectionA,
    kDirectionB,
  };

  SharedStream GetStream(boost::asio::io_service&,
                         const std::string& key,
                         Mode mode);

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}
}
