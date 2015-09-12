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

#include <string>

#include <boost/format/format_fwd.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/system/error_code.hpp>

namespace legtool {
/// Similar to a boost::system::error_code, but provides a facility
/// for attaching additional context rather than just the error code.
class ErrorCode {
 public:
  /// This constructor is purposefully non-explicit, so that we can
  /// transparently capture the boost variety and later allow context
  /// to be added.
  ErrorCode(const boost::system::error_code& ec)
      : ec_(ec),
        message_(boost::lexical_cast<std::string>(ec_) + " " + ec.message()) {}

  ErrorCode(int val, const boost::system::error_category& category,
            const std::string& message = "")
      : ErrorCode(boost::system::error_code(val, category)) {
    if (!message.empty()) { message_ += "\n" + message; }
  }

  template <typename ErrorCodeEnum>
  ErrorCode(ErrorCodeEnum value)
      : ErrorCode(boost::system::error_code(value)) {}

  ErrorCode() {}

  static ErrorCode einval(const std::string& message) {
    return ErrorCode(boost::system::errc::invalid_argument,
                     boost::system::generic_category(),
                     message);
  }

  static ErrorCode syserrno(const std::string& message) {
    return ErrorCode(errno, boost::system::generic_category(), message);
  }

  /// @return a string describing the message, along with all context
  /// which has been added.
  std::string message() const { return message_; }

  /// @return the boost error_code associated with this error.
  const boost::system::error_code& error_code() const { return ec_; }

  operator boost::system::error_code() const { return ec_; }
  explicit operator bool() const { return !!ec_; }

  bool operator==(const ErrorCode& rhs) const {
    return ec_ == rhs.ec_;
  }

  bool operator!=(const ErrorCode& rhs) const {
    return ec_ != rhs.ec_;
  }

  // Append context to this error.  These all boil down to adding
  // additional textual context.
  void AppendError(const ErrorCode& ec) { Append(ec.message()); }
  void Append(const boost::system::error_code& ec) { Append(ec.message()); }
  void Append(const boost::format&);
  void Append(const std::string& message) {
    if (!message_.empty()) { message_ += "\n"; }
    message_ += message;
  }

 private:
  const boost::system::error_code ec_;
  std::string message_;
};

/// Similar to a boost::system::system_error, but works with
/// ErrorCodes.
class SystemError : public std::runtime_error {
 public:
  SystemError(const ErrorCode& ec)
      : std::runtime_error(""),
        ec_(ec) {}

  SystemError(int val, const boost::system::error_category& category,
              const std::string& message = "")
      : SystemError(ErrorCode(val, category, message)) {}

  static SystemError einval(const std::string& message) {
    return SystemError(ErrorCode::einval(message));
  }

  static SystemError syserrno(const std::string& message) {
    return SystemError(ErrorCode::syserrno(message));
  }

  virtual ~SystemError() throw() {}

  const char* what() const throw() { return ec_.message().c_str(); }
  ErrorCode& error_code() { return ec_; }

 private:
  ErrorCode ec_;
};
}
