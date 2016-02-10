// Copyright 2015-2016 Josh Pieper, jjp@pobox.com.  All rights reserved.
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

#include <functional>
#include <string>

#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/asio/posix/stream_descriptor.hpp>
#include <boost/program_options.hpp>

#include "base/common.h"
#include "base/concrete_comm_factory.h"
#include "base/error_wrap.h"
#include "base/program_options_archive.h"

#include "herkulex.h"
#include "herkulex_servo_interface.h"

using namespace mjmech::base;
using namespace mjmech::mech;
namespace po = boost::program_options;
namespace pl = std::placeholders;

namespace {
struct Options {
  int address = 254;
};

typedef HerkuleX Servo;
typedef HerkuleXConstants HC;

struct CommandContext {
  boost::asio::io_service& service;
  const Options& options;
  Servo& servo;
  ServoInterface& servo_interface;
  std::string args;
  boost::asio::yield_context yield;
};
typedef std::function<void (CommandContext&)> CommandFunction;

enum CommandArg {
  kNoArgs,
  kArg,
};

struct Command {
  CommandArg args;
  CommandFunction function;
};

struct CommandText {
  std::string name;
  std::string args;
};

HC::Register ParseRegister(const std::string& args) {
  // First, try an integer register number.
  int reg_address = 0;
  int reg_size = 1;
  try {
    size_t pos = args.find(':');
    std::string address_str;
    std::string size_str;
    if (pos == std::string::npos) {
      address_str = args;
    } else {
      address_str = args.substr(0, pos);
      size_str = args.substr(pos + 1);
    }

    reg_address = std::stoi(address_str, 0, 0);
    if (reg_address < 0 || reg_address > 0xfe) {
      throw std::runtime_error(
          (boost::format("Address out of range: %d") % reg_address).str());
    }
    if (!size_str.empty()) {
      reg_size = std::stoi(size_str, 0, 0);
    }
  } catch (std::invalid_argument&) {
    // OK, try a named one.
    HC constants;
    auto it = constants.ram_registers.find(args);
    if (it == constants.ram_registers.end()) {
      throw std::runtime_error(
          (boost::format("Could not parse RAM address '%s'") %
           args).str());
    }
    return it->second;
  }

  return HC::Register{
    static_cast<uint8_t>(reg_address),
        static_cast<uint8_t>(reg_size)};
}

void DoStdio(CommandContext&);

void MemReadCommand(CommandContext& ctx, Servo::Command command) {
  HC::Register reg = ParseRegister(ctx.args);

  auto response = ctx.servo.MemRead(
      command, ctx.options.address,
      reg.position, reg.length, ctx.yield);
  std::cout << (boost::format("Servo %d: Address %d: ") %
                static_cast<int>(ctx.options.address) %
                static_cast<int>(reg.position));
  for (char c: response.register_data) {
    std::cout << boost::format(" %02X") % static_cast<int>(c);
  }
  if (response.register_data.size() > 1) {
    int value = 0;
    for (size_t i = 0; i < response.register_data.size(); i++) {
      value |= (static_cast<uint8_t>(
                    response.register_data[i]) << (i * reg.bit_align));
    }

    if (reg.sign) {
      const int most_positive = 1 << (reg.bit_align * reg.length - 1);
      if (value >= most_positive) {
        value = value - (1 << (reg.bit_align * reg.length));
      }
    }

    std::cout << boost::format(" (%d)") % value;
  }
  std::cout << "\n";
}

void MemWriteCommand(CommandContext& ctx, Servo::Command command) {
  std::vector<std::string> args;
  boost::split(args, ctx.args, boost::is_any_of(":"));
  if (args.size() < 2) {
    throw std::runtime_error("ram_write requires ADDR:HEXDATA");
  }

  HC::Register reg = ParseRegister(args.at(0));
  std::ostringstream ostr;
  std::string hexdata = args.at(1);
  for (size_t i = 0; i < hexdata.size(); i += 2) {
    std::string hexbyte = hexdata.substr(i, 2);
    uint8_t hexvalue = std::stoi(hexbyte, 0, 16);
    ostr.write(reinterpret_cast<const char*>(&hexvalue), 1);
  }

  ctx.servo.MemWrite(command, ctx.options.address,
                     reg.position, ostr.str(), ctx.yield);
}

const std::map<std::string, Command> g_commands = {
  { "stdio", { kNoArgs, DoStdio } },
  { "sleep", { kArg, [](CommandContext& ctx) {
        DeadlineTimer timer(ctx.service);
        double delay_s = std::stod(ctx.args);
        timer.expires_from_now(ConvertSecondsToDuration(delay_s));
        timer.async_wait(ctx.yield);
      } } },
  { "reboot", { kNoArgs, [](CommandContext& ctx) {
        ctx.servo.Reboot(ctx.options.address, ctx.yield);
      } } },
  { "status", { kNoArgs, [](CommandContext& ctx) {
        auto response = ctx.servo.Status(ctx.options.address, ctx.yield);
        std::cout << (boost::format("Servo %d: (0x%02x 0x%02x)\n") %
                      static_cast<int>(ctx.options.address) %
                      static_cast<int>(response.reg48) %
                      static_cast<int>(response.reg49));
      } } },
  { "voltage", { kNoArgs, [](CommandContext& ctx) {
        int value = ctx.servo.RamRead(ctx.options.address, HC().voltage(),
                                      ctx.yield);
        std::cout << (boost::format("Servo %d: %d\n") %
                      static_cast<int>(ctx.options.address) %
                      value);
      } } },
  { "ram_read", { kArg, std::bind(MemReadCommand, pl::_1, Servo::RAM_READ) } },
  { "eep_read", { kArg, std::bind(MemReadCommand, pl::_1, Servo::EEP_READ) } },
  { "ram_write", { kArg, std::bind(MemWriteCommand, pl::_1, Servo::RAM_WRITE) } },
  { "eep_write", { kArg, std::bind(MemWriteCommand, pl::_1, Servo::EEP_WRITE) } },
  { "value_write", { kArg, [](CommandContext& ctx) {
        std::vector<std::string> args;
        boost::split(args, ctx.args, boost::is_any_of(":"));
        if (args.size() < 2) {
          throw std::runtime_error("value_write requires ADDR:VALUE");
        }

        HC::Register reg = ParseRegister(args.at(0));
        const int value = std::stoi(args.at(1), 0, 0);
        std::ostringstream ostr;
        const int mask = (1 << reg.bit_align) - 1;
        for (size_t i = 0; i < reg.length; i++) {
          const uint8_t byte = (value >> (i * reg.bit_align)) & mask;
          ostr.write(reinterpret_cast<const char*>(&byte), 1);
        }

        ctx.servo.MemWrite(ctx.servo.RAM_WRITE, ctx.options.address,
                           reg.position, ostr.str(), ctx.yield);
      } } },
  { "set_address", { kArg, [](CommandContext& ctx) {
        int new_address = std::stoi(ctx.args, 0, 0);
        ctx.servo.EepWrite(ctx.options.address, HC::id(),
                           new_address, ctx.yield);
      } } },
  { "enumerate", { kNoArgs, [](CommandContext& ctx) {
        for (int i = 0; i < 254; i++) {
          try {
            ctx.servo.Status(i, ctx.yield);
            std::cout << boost::format("%d ") % i;
            std::cout.flush();
          } catch (boost::system::system_error& ec) {
            if (ec.code() != boost::asio::error::operation_aborted) {
              throw;
            }
          }
        }
        std::cout << "\n";
      } } },
  { "set_pose", { kArg, [](CommandContext& ctx) {
        std::vector<std::string> args;
        boost::split(args, ctx.args, boost::is_any_of(","));

        std::vector<ServoInterface::Joint> joints;
        for (std::string jstr: args) {
          std::vector<std::string> fields;
          boost::split(fields, jstr, boost::is_any_of(":"));
          joints.emplace_back(ServoInterface::Joint{
              std::stoi(fields.at(0)),
                  std::stod(fields.at(1))});
        }

        boost::asio::detail::async_result_init<
          boost::asio::yield_context,
          void (boost::system::error_code)> init(
              BOOST_ASIO_MOVE_CAST(boost::asio::yield_context)(ctx.yield));

        ctx.servo_interface.SetPose(joints, ErrorHandler(init.handler));

        init.result.get();
      } } },
  { "list_registers", { kNoArgs, [](CommandContext& ctx) {
        HC constants;
        for (const auto& pair: constants.ram_registers) {
          std::cout << boost::format("%-30s 0x%02X length=%d stride=%d") %
              pair.first %
              static_cast<int>(pair.second.position) %
              static_cast<int>(pair.second.length) %
              static_cast<int>(pair.second.bit_align) << "\n";
        }
      } } },
};

class CommandValue : public boost::program_options::value_semantic {
 public:
  CommandValue(const std::string& name, CommandArg command_arg,
               std::vector<CommandText>* output)
      : name_(name), command_arg_(command_arg), output_(output) {}

  virtual std::string name() const { return name_; }
  virtual unsigned int min_tokens() const {
    return command_arg_ == kArg ? 1 : 0;
  }
  virtual unsigned int max_tokens() const { return min_tokens(); }
  virtual bool is_composing() const { return false; }
  virtual bool is_required() const { return false; }
  virtual bool apply_default(boost::any&) const { return false; }
  virtual void notify(const boost::any&) const {}
  virtual void parse(boost::any& value_store,
                     const std::vector<std::string>& new_tokens,
                     bool utf8) const {
    output_->push_back(
        CommandText{name_, command_arg_ == kArg ?
              new_tokens.at(0) : ""});
  }

 private:
  const std::string name_;
  const CommandArg command_arg_;
  std::vector<CommandText>* const output_;
};

int work(int argc, char** argv) {
  boost::asio::io_service service;
  ConcreteStreamFactory factory(service);
  Servo servo(service, factory);
  HerkuleXServoInterface<Servo> servo_interface(&servo);

  // Default to something moderately useful.
  boost::any value = std::string("serial");
  servo.parameters()->stream.options_description()->
      find("type", false).semantic()->apply_default(value);

  Options options;

  po::options_description desc("Allowable options");
  desc.add_options()
      ("help,h", "display usage mesage")
      ("address,a", po::value(&options.address)->default_value(options.address),
       "servo to communicate with")
      ;

  std::vector<CommandText> command_sequence;
  for (const auto& pair: g_commands) {
    desc.add_options()
        (pair.first.c_str(), new CommandValue(
            pair.first, pair.second.args, &command_sequence), "");
  }

  ProgramOptionsArchive(&desc, "servo.").Accept(servo.parameters());

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);

  if (vm.count("help")) {
    std::cerr << desc;
    return 0;
  }

  boost::asio::spawn(service, ErrorWrap([&](boost::asio::yield_context yield) {
        servo.AsyncStart(yield);

        for (const auto& command: command_sequence) {
          CommandContext ctx{
            service, options, servo, servo_interface,
                command.args, yield};
          auto it = g_commands.find(command.name);
          BOOST_ASSERT(it != g_commands.end());
          try {
            it->second.function(ctx);
          } catch (boost::system::system_error& se) {
            std::cerr << "error: " << se.what() << "\n";
          }
        }

        service.stop();
      }));

  service.run();
  return 0;
}

void DoStdio(CommandContext& ctx) {
  boost::asio::posix::stream_descriptor descriptor(ctx.service, ::dup(0));
  std::string last_line;
  while (true) {
    boost::asio::streambuf stream;
    boost::asio::async_read_until(descriptor, stream, '\n', ctx.yield);
    std::ostringstream ostr;
    ostr << &stream;

    std::string line = ostr.str();
    BOOST_ASSERT(!line.empty());
    BOOST_ASSERT(line[line.size() - 1] == '\n');
    line = line.substr(0, line.size() - 1); // strip newline

    if (line.empty()) {
      line = last_line;
    }

    last_line = line;

    std::size_t pos = line.find_first_of(' ');
    std::string command_name;
    std::string args;
    if (pos == std::string::npos) {
      command_name = line;
    } else {
      command_name = line.substr(0, pos);
      args = line.substr(pos + 1);
    }

    CommandContext sub_context{
      ctx.service, ctx.options, ctx.servo, ctx.servo_interface,
          args, ctx.yield};
    auto it = g_commands.find(command_name);
    if (it == g_commands.end()) {
      std::cerr << "Unknown command: '" + command_name + "'\n";
    } else {
      try {
        it->second.function(sub_context);
      } catch (boost::system::system_error& se) {
        std::cerr << "error: " << se.what() << "\n";
      }
    }
  }
}
}

int main(int argc, char** argv) {
  try {
    return work(argc, argv);
  } catch (std::exception& e) {
    std::cerr << "error: " << e.what() << "\n";
    return 1;
  }
  return 0;
}
