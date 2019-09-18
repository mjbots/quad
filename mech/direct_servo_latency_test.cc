// Copyright 2019 Josh Pieper, jjp@pobox.com.
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

#include <linux/serial.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <iostream>

#include <fmt/format.h>

#include <boost/program_options.hpp>

#include "mjlib/base/buffer_stream.h"
#include "mjlib/base/system_error.h"

#include "mjlib/multiplex/format.h"
#include "mjlib/multiplex/frame.h"
#include "mjlib/multiplex/register.h"
#include "mjlib/multiplex/stream.h"

#include "base/system_fd.h"

namespace po = boost::program_options;

namespace base = mjlib::base;
namespace multiplex = mjlib::multiplex;

namespace {
constexpr int kSetupRegister = 0;  // mode
constexpr int8_t kSetupValue = 0;  // kStopped
constexpr int kNonceRegister = 0x10;  // kPwmPhaseA

using mjmech::base::SystemFd;

SystemFd open_serial(const std::string& serial_port) {
  SystemFd fd{::open(serial_port.c_str(), O_RDWR | O_CLOEXEC | O_NOCTTY)};
  if (fd < 0) {
    throw base::system_error::syserrno("opening: " + serial_port);
  }

  {
    struct termios options = {};
    int result = ::tcgetattr(fd, &options);
    if (result) { throw base::system_error::syserrno("getting serial params"); }

    const auto baud = B3000000;

    options.c_cflag = baud | CS8 | CLOCAL | CREAD;

    // Set up timeouts: Calls to read() will return as soon as there is
    // at least one byte available or when 500 ms has passed.
    options.c_cc[VTIME] = 5;
    options.c_cc[VMIN] = 0;

    ::tcflush(fd, TCIOFLUSH);
    ::tcsetattr(fd, TCSANOW, &options);
  }


  {
    struct serial_struct serial;
    ::ioctl(fd, TIOCGSERIAL, &serial);
    serial.flags |= ASYNC_LOW_LATENCY;
    ::ioctl(fd, TIOCSSERIAL, &serial);
  }

  return fd;
}

bool complete_packet_present(const char* data, std::size_t size) {
  auto* const result = static_cast<const char*>(
      ::memmem(data, size, &multiplex::Format::kHeader, 2));
  if (result == nullptr) { return false; }

  const auto offset = result - data;
  const char* packet = data + offset;
  size -= offset;

  if (size < 7) {
    // below the minimum size
    return false;
  }

  base::BufferReadStream base_stream({packet + 4, size - 4});
  multiplex::ReadStream<base::BufferReadStream> stream(base_stream);

  const auto packet_size = stream.ReadVaruint();
  if (packet_size > (size - 6)) {
    return false;
  }

  // We've got enough, call it good.  We won't bother doing checksums
  // for now.
  return true;
}

struct Target {
  multiplex::Frame frame;
  std::string to_write;
  const char* to_write_buf = nullptr;
  std::size_t to_write_size = 0;
};
}

int main(int argc, char** argv) {
  po::options_description desc("Allowable options");

  std::string serial_port;

  desc.add_options()
      ("help,h", "display usage message")
      ("port,p", po::value(&serial_port), "serial port to use")
      ;

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);

  if (vm.count("help")) {
    std::cout << desc;
    return 0;
  }

  auto fd = open_serial(serial_port);

  multiplex::RegisterRequest request;
  request.WriteSingle(kSetupRegister, kSetupValue);

  // The position command needs to set 3 values.  For now, we'll
  // send 3 int16's.
  auto i16 = [](auto v) { return static_cast<int16_t>(v); };
  request.WriteMultiple(0x10, {i16(20), i16(0), i16(1)});

  request.ReadMultiple(kNonceRegister, 3, 1);
  // And read two i8 regs, like voltage and fault
  request.ReadMultiple(0x006, 2, 0);

  std::vector<Target> frames{12};
  for (int i = 0; i < 12; i++) {
    auto& frame = frames[i].frame;
    frame.source_id = 0;
    frame.dest_id = i + 1;
    frame.request_reply = true;
    frame.payload = request.buffer();

    frames[i].to_write = frame.encode();
    frames[i].to_write_buf = frames[i].to_write.c_str();
    frames[i].to_write_size = frames[i].to_write.size();
  }

  std::cout << fmt::format("output size: {}\n", frames[0].to_write_size);

  char read_buf[2048] = {};

  uint32_t count = 0;
  int complete = 0;
  int skipped = 0;
  int bytes = 0;

  int this_target = 0;

  while (true) {
    auto& target = frames[this_target];

    count++;
    if ((count % 1000) == 0) {
      std::cout << fmt::format("{}  skipped={} bytes={}\n",
                               complete, skipped, bytes);
    }

    {
      const int result = ::write(fd, target.to_write_buf, target.to_write_size);
      if (result < 0) { throw base::system_error::syserrno("writing"); }
    }

    std::size_t offset = 0;
    while (true) {
      const int result = ::read(fd, read_buf + offset, sizeof(read_buf));
      if (result < 0) { throw base::system_error::syserrno("reading"); }

      bytes += result;

      if (result == 0) {
        // timeout
        skipped++;
        break;
      }

      offset += result;

      // Check to see if we have a complete packet.
      if (complete_packet_present(read_buf, offset)) {
        // mark complete
        complete++;
        break;
      }
    }

    this_target = (this_target + 1) % 12;
    if (this_target == 0) {
      ::usleep(1000);
    }
  }

  return 0;
}
