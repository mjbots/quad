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

#include "linux_i2c_generator.h"

#include <linux/i2c-dev.h>

#include <boost/format.hpp>

#include "program_options_archive.h"

namespace po = boost::program_options;

namespace mjmech {
namespace base {

LinuxI2CGenerator::Parameters::Parameters() {
  ProgramOptionsArchive(&options_description_).Accept(this);
}

namespace {

// TODO jpieper: For now, this uses the blocking Linux APIs.  It
// should be updated to do its work in a background thread.
class LinuxI2C : public AsyncI2C {
 public:
  LinuxI2C(boost::asio::io_service& service,
           const std::string& device)
      : service_(service),
        fd_(::open(device.c_str(), O_RDWR)) {
    if (fd_ < 0) {
      throw base::SystemError(
          errno, boost::system::generic_category(),
          "error opening '" + device + "'");
    }
  }

  virtual ~LinuxI2C() {}

  boost::asio::io_service& get_io_service() override {
    return service_;
  }

  void AsyncRead(uint8_t device, uint8_t address,
                 MutableBufferSequence buffers,
                 ReadHandler handler) override {
    if (!ErrWrap(::ioctl(fd_, I2C_SLAVE, static_cast<int>(device)),
                 "when selecting device", handler)) { return; }

    union i2c_smbus_data data;
    auto size = boost::asio::buffer_size(buffers);
    BOOST_ASSERT(size >= 0 && size <= 255);
    data.block[0] = size;
    if (!ErrWrap(::i2c_smbus_access(fd_, I2C_SMBUS_READ, address,
                                    I2C_SMBUS_I2C_BLOCK_DATA, &data),
                 "during transfer", handler)) { return; }

    if (data.block[0] != size) {
      service_.post(
          std::bind(
              handler,
              ErrorCode::einval(
                  (boost::format("asked for length %d got %d") %
                   static_cast<int>(size) %
                   static_cast<int>(data.block[0])).str()),
              0));
      return;
    }

    boost::asio::buffer_copy(
        buffers, boost::asio::buffer(&data.block[1], size));

    service_.post(std::bind(handler, ErrorCode(), size));
  }

  void AsyncWrite(uint8_t device, uint8_t address,
                  ConstBufferSequence buffers,
                  WriteHandler handler) override {
    if (!ErrWrap(::ioctl(fd_, I2C_SLAVE, static_cast<int>(device)),
                 "when selecting device", handler)) { return; }

    union i2c_smbus_data i2c_data;
    auto size = boost::asio::buffer_size(buffers);
    BOOST_ASSERT(size <= 32);
    i2c_data.block[0] = size;
    boost::asio::buffer_copy(
        boost::asio::buffer(&i2c_data.block[1], size), buffers);
    if (!ErrWrap(::i2c_smbus_access(fd_, I2C_SMBUS_WRITE, address,
                                    I2C_SMBUS_I2C_BLOCK_BROKEN, &i2c_data),
                 "during transfer", handler)) { return; }

    service_.post(std::bind(handler, ErrorCode(), size));
  }

 private:
  template <typename Handler>
  bool ErrWrap(int value,
               const std::string& message,
               Handler handler) {
    if (value == 0) { return true; }
    service_.post(std::bind(handler, ErrorCode::einval(message), 0));
    return false;
  }

  boost::asio::io_service& service_;
  const int fd_;
};
}

LinuxI2CGenerator::LinuxI2CGenerator(boost::asio::io_service& service)
  : service_(service) {}
LinuxI2CGenerator::~LinuxI2CGenerator() {}

std::unique_ptr<I2CFactory::GeneratorParameters>
LinuxI2CGenerator::MakeParameters() const {
  return std::unique_ptr<I2CFactory::GeneratorParameters>(new Parameters());
}

void LinuxI2CGenerator::AsyncCreate(
    const I2CFactory::GeneratorParameters& generator_parameters,
    SharedI2CHandler handler) const {
  const Parameters& parameters =
      dynamic_cast<const Parameters&>(generator_parameters);
  SharedI2C shared(new LinuxI2C(service_, parameters.device));
  service_.post(std::bind(handler, ErrorCode(), shared));
}

}
}
