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

#include <functional>
#include <iostream>
#include <string>

#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/asio/posix/stream_descriptor.hpp>
#include <boost/program_options.hpp>

#include <fmt/format.h>

#include "mjlib/io/deadline_timer.h"
#include "mjlib/io/stream_factory.h"

#include "base/common.h"
#include "base/program_options.h"

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
  CommandContext(boost::asio::io_context& service_in,
                 const Options& options_in,
                 Servo& servo_in,
                 ServoInterface& servo_interface_in,
                 const std::string& args_in)
      : service(service_in),
        options(options_in),
        servo(servo_in),
        servo_interface(servo_interface_in),
        args(args_in) {}

  boost::asio::io_context& service;
  const Options& options;
  Servo& servo;
  ServoInterface& servo_interface;
  std::string args;
};
typedef std::function<
  void (CommandContext&, mjlib::io::ErrorCallback)> CommandFunction;

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
          fmt::format("Address out of range: {}", reg_address));
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
          fmt::format("Could not parse RAM address '{}'", args));
    }
    return it->second;
  }

  return HC::Register{
    static_cast<uint8_t>(reg_address),
        static_cast<uint8_t>(reg_size)};
}

void DoStdio(CommandContext&, mjlib::io::ErrorCallback);

void MemReadCommand(CommandContext& ctx, mjlib::io::ErrorCallback handler,
                    Servo::Command command) {
  HC::Register reg = ParseRegister(ctx.args);

  ctx.servo.MemRead(
      command, ctx.options.address,
      reg.position, reg.length,
      [handler, ctx, reg](const auto& ec, const auto response) {
        FailIf(ec);

        std::cout << fmt::format("Servo {}: Address {}: ",
                                 static_cast<int>(ctx.options.address),
                                 static_cast<int>(reg.position));
        for (char c: response.register_data) {
          std::cout << fmt::format(" {:02X}", static_cast<int>(c));
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

          std::cout << fmt::format(" ({})", value);
        }
        std::cout << "\n";
        handler(ec);
      });
}

void MemWriteCommand(CommandContext& ctx, mjlib::io::ErrorCallback handler,
                     Servo::Command command) {
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
                     reg.position, ostr.str(), handler);
}

class EnumerateCommand : public std::enable_shared_from_this<EnumerateCommand> {
 public:
  EnumerateCommand(CommandContext& context, mjlib::io::ErrorCallback handler)
      : context_(context),
        handler_(handler) {}

  void Start() {
    if (next_address_ == 254) {
      std::cout << "\n";
      handler_(mjlib::base::error_code());
      return;
    }

    const auto to_send = next_address_;
    next_address_++;

    context_.servo.Status(
        to_send, std::bind(
            &EnumerateCommand::HandleStatus, shared_from_this(),
            pl::_1, pl::_2, to_send));
  }

 private:
  void HandleStatus(const mjlib::base::error_code& ec,
                    Servo::StatusResponse, int address) {
    if (ec == boost::asio::error::operation_aborted) {
    } else {
      FailIf(ec);

      std::cout << fmt::format("{:d} ", address);
      std::cout.flush();
    }

    Start();
  }

  CommandContext& context_;
  mjlib::io::ErrorCallback handler_;
  int next_address_ = 0;
};

const std::map<std::string, Command> g_commands = {
  { "stdio", { kNoArgs, DoStdio } },
  { "sleep", { kArg, [](CommandContext& ctx, mjlib::io::ErrorCallback handler) {
        auto timer = std::make_shared<mjlib::io::DeadlineTimer>(ctx.service);
        double delay_s = std::stod(ctx.args);
        timer->expires_from_now(ConvertSecondsToDuration(delay_s));
        timer->async_wait([timer, handler](const mjlib::base::error_code& ec) {
            handler(ec);
          });
      } } },
  { "reboot", { kNoArgs, [](CommandContext& ctx, mjlib::io::ErrorCallback handler) {
        ctx.servo.Reboot(ctx.options.address, handler);
      } } },
  { "status", { kNoArgs, [](CommandContext& ctx, mjlib::io::ErrorCallback handler) {
        ctx.servo.Status(
            ctx.options.address,
            [handler, address=ctx.options.address](
                const mjlib::base::error_code& ec, Servo::StatusResponse response) {
              std::cout << fmt::format("Servo {}: (0x{:02x} 0x{:02x})\n",
                                       static_cast<int>(address),
                                       static_cast<int>(response.reg48),
                                       static_cast<int>(response.reg49));
              handler(ec);
            });
      } } },
  { "voltage", { kNoArgs, [](CommandContext& ctx, mjlib::io::ErrorCallback handler) {
        ctx.servo.RamRead(
            ctx.options.address, HC().voltage(),
            [handler, address=ctx.options.address](
                const mjlib::base::error_code& ec, int value) {
              std::cout << fmt::format(
                  "Servo {}: {}\n",
                  static_cast<int>(address),
                  value);
              handler(ec);
            });
      } } },
  { "ram_read", { kArg, std::bind(MemReadCommand, pl::_1, pl::_2, Servo::RAM_READ) } },
  { "eep_read", { kArg, std::bind(MemReadCommand, pl::_1, pl::_2, Servo::EEP_READ) } },
  { "ram_write", { kArg, std::bind(MemWriteCommand, pl::_1, pl::_2, Servo::RAM_WRITE) } },
  { "eep_write", { kArg, std::bind(MemWriteCommand, pl::_1, pl::_2, Servo::EEP_WRITE) } },
  { "value_write", { kArg, [](CommandContext& ctx, mjlib::io::ErrorCallback handler) {
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
                           reg.position, ostr.str(), handler);
      } } },
  { "set_address", { kArg, [](CommandContext& ctx, mjlib::io::ErrorCallback handler) {
        int new_address = std::stoi(ctx.args, 0, 0);
        ctx.servo.EepWrite(ctx.options.address, HC::id(),
                           new_address, handler);
      } } },
  { "enumerate", { kNoArgs, [](CommandContext& ctx, mjlib::io::ErrorCallback handler) {
        auto command = std::make_shared<EnumerateCommand>(ctx, handler);
        command->Start();
      } } },
  { "set_pose", { kArg, [](CommandContext& ctx, mjlib::io::ErrorCallback handler) {
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

        ctx.servo_interface.SetPose(joints, handler);
      } } },
  { "list_registers", { kNoArgs, [](CommandContext&, mjlib::io::ErrorCallback handler) {
        HC constants;
        for (const auto& pair: constants.ram_registers) {
          std::cout << fmt::format("{:<30s} 0x{:02X} length={} stride={}",
                                   pair.first,
                                   static_cast<int>(pair.second.position),
                                   static_cast<int>(pair.second.length),
                                   static_cast<int>(pair.second.bit_align))
                    << "\n";
        }
        handler(mjlib::base::error_code());
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
  virtual void parse(boost::any&,
                     const std::vector<std::string>& new_tokens,
                     bool /* utf8 */) const {
    output_->push_back(
        CommandText{name_, command_arg_ == kArg ?
              new_tokens.at(0) : ""});
  }
  virtual bool adjacent_tokens_only() const {
    return false;
  }

 private:
  const std::string name_;
  const CommandArg command_arg_;
  std::vector<CommandText>* const output_;
};

class CommandRunner {
 public:
  CommandRunner() {
    // Default to something moderately useful.
    mjmech::base::SetOption(servo_.options(), "stream.type", "serial");
  }

  template <typename Commands>
  void Run(const Commands& commands) {
    std::copy(commands.begin(), commands.end(), std::back_inserter(commands_));

    servo_.AsyncStart([this](mjlib::base::error_code ec) {
        FailIf(ec);
        this->Start();
      });

    service_.run();
  }

  Options* options() { return &options_; }
  Servo* servo() { return &servo_; }

 private:
  void Start() {
    if (commands_.empty()) {
      service_.stop();
      return;
    }

    const auto this_command = commands_.front();
    commands_.pop_front();

    auto ctx = std::make_shared<CommandContext>(
      service_, options_, servo_, servo_interface_,
      this_command.args);
    auto it = g_commands.find(this_command.name);
    BOOST_ASSERT(it != g_commands.end());

    it->second.function(*ctx, [this, ctx](const auto& ec) {
        FailIf(ec);
        this->Start();
      });
  }

  boost::asio::io_context service_;
  mjlib::io::StreamFactory factory_{service_};
  Servo servo_{service_, factory_};
  HerkuleXServoInterface<Servo> servo_interface_{&servo_};
  Options options_;
  std::deque<CommandText> commands_;
};

int work(int argc, char** argv) {
  CommandRunner runner;

  po::options_description desc("Allowable options");
  desc.add_options()
      ("help,h", "display usage mesage")
      ("address,a", po::value(&runner.options()->address)->default_value(runner.options()->address),
       "servo to communicate with")
      ;

  std::vector<CommandText> command_sequence;
  for (const auto& pair: g_commands) {
    desc.add_options()
        (pair.first.c_str(), new CommandValue(
            pair.first, pair.second.args, &command_sequence), "");
  }

  MergeProgramOptions(runner.servo()->options(), "servo.", &desc);

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);

  if (vm.count("help")) {
    std::cerr << desc;
    return 0;
  }

  runner.Run(command_sequence);

  return 0;
}

class StdioHandler : public std::enable_shared_from_this<StdioHandler> {
 public:
  StdioHandler(CommandContext& context, mjlib::io::ErrorCallback handler)
      : context_(context),
        final_handler_(handler) {}

  void Start() {
    boost::asio::async_read_until(
        descriptor_, stream_, '\n',
        std::bind(&StdioHandler::HandleRead, shared_from_this(),
                  std::placeholders::_1));
  }

 private:
  void HandleRead(const mjlib::base::error_code& ec) {
    FailIf(ec);

    std::ostringstream ostr;
    ostr << &stream_;

    std::string line = ostr.str();
    BOOST_ASSERT(!line.empty());
    BOOST_ASSERT(line[line.size() - 1] == '\n');
    line = line.substr(0, line.size() - 1); // strip newline

    if (line.empty()) {
      line = last_line_;
    }

    last_line_ = line;

    std::size_t pos = line.find_first_of(' ');
    std::string command_name;
    std::string args;
    if (pos == std::string::npos) {
      command_name = line;
    } else {
      command_name = line.substr(0, pos);
      args = line.substr(pos + 1);
    }

    auto sub_context = std::make_shared<CommandContext>(
      context_.service, context_.options, context_.servo, context_.servo_interface,
      args);
    auto it = g_commands.find(command_name);
    if (it == g_commands.end()) {
      std::cerr << "Unknown command: '" + command_name + "'\n";
      Start();
    } else {
      it->second.function(
          *sub_context,
          [self=shared_from_this(), sub_context](mjlib::base::error_code ec) {
            FailIf(ec);
            self->Start();
          });
    }
  }

  CommandContext& context_;
  boost::asio::posix::stream_descriptor descriptor_{
    context_.service, ::dup(0)};
  std::string last_line_;
  mjlib::io::ErrorCallback final_handler_;
  boost::asio::streambuf stream_;
};

void DoStdio(CommandContext& ctx, mjlib::io::ErrorCallback handler) {
  auto stdio = std::make_shared<StdioHandler>(ctx, handler);
  stdio->Start();
}
}

extern "C" {
int main(int argc, char** argv) {
  try {
    return work(argc, argv);
  } catch (std::exception& e) {
    std::cerr << "error: " << e.what() << "\n";
    return 1;
  }
  return 0;
}
}
