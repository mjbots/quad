// Copyright 2014-2019 Josh Pieper, jjp@pobox.com.
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

#include <boost/asio/executor.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <boost/dynamic_bitset.hpp>
#include <boost/noncopyable.hpp>

#include "mjlib/io/async_types.h"

namespace mjmech {
namespace base {
/// Support reading events from a linux input device.
class LinuxInput : boost::noncopyable {
 public:
  explicit LinuxInput(const boost::asio::executor&);
  LinuxInput(const boost::asio::executor&, const std::string& device);
  void Open(const std::string& device);

  ~LinuxInput();

  boost::asio::executor get_executor();

  /// @return the underlying file descriptor for this device.
  int fileno() const;

  /// @return the human readable name the kernel provides for this
  /// device
  std::string name() const;

  struct AbsInfo {
    int axis = -1;
    int value = 0;
    int minimum = 0;
    int maximum = 0;
    int fuzz = 0;
    int flat = 0;
    int resolution = 0;

    double scaled() const {
      double center = 0.5 * (minimum + maximum);
      return (value - center) / (0.5 * (maximum - minimum));
    }
  };

  /// @return the absolute value associated with a given joystick axis.
  ///
  /// @param axis is one of the ABS_* defines from linux/input.h
  AbsInfo abs_info(int axis) const;

  struct Features {
    int ev_type = -1;

    /// This bitset contains bits set as defined in the relevant type
    /// specific defines.
    boost::dynamic_bitset<> capabilities;
  };

  /// @return an object describing the capabilities of this input
  /// device
  ///
  /// @param ev_type should be one of the EV_* defines.
  Features features(int ev_type) const;

  struct Event {
    boost::posix_time::ptime time;
    int ev_type = 0;
    int code = 0;
    int value = 0;
  };

  /// Read one event asynchronously.  This function returns
  /// immediately, @p handler will be invoked using boost::asio::post
  /// and @p event must be valid until the handler is invoked.
  void AsyncRead(Event* event, mjlib::io::ErrorCallback handler);

  /// Cancel all asynchronous operations associated with this device.
  void cancel();

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

// Overloads for diagnostic output.
std::ostream& operator<<(std::ostream&, const LinuxInput&);
std::ostream& operator<<(std::ostream&, const LinuxInput::AbsInfo&);
std::ostream& operator<<(std::ostream&, const LinuxInput::Features&);
std::ostream& operator<<(std::ostream&, const LinuxInput::Event&);
}
}
