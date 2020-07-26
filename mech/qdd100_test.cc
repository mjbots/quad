// Copyright 2020 Josh Pieper, jjp@pobox.com.
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

#include <linux/input.h>

#include <atomic>
#include <fstream>
#include <functional>
#include <thread>

#include <fmt/format.h>

#include <boost/asio.hpp>
#include <boost/signals2.hpp>

#include "mjlib/base/clipp_archive.h"
#include "mjlib/base/clipp.h"
#include "mjlib/base/fail.h"
#include "mjlib/base/time_conversions.h"
#include "mjlib/io/repeating_timer.h"
#include "mjlib/multiplex/stream_asio_client_builder.h"
#include "mjlib/telemetry/file_writer.h"

#include "base/linux_input.h"
#include "base/telemetry_log_registrar.h"
#include "mech/moteus.h"

namespace pl = std::placeholders;

namespace mjmech {
namespace mech {

namespace {
using Value = mjlib::multiplex::Format::Value;

struct Options {
  std::string joystick;
  mjlib::multiplex::StreamAsioClientBuilder::Options stream;
  double period_s = 0.05;
  double max_torque_Nm = 1.0;

  std::string load_cell;
  double load_cell_tare_g = 110.0;
  double torque_scale = 0.00980665;

  std::string power_supply;

  std::string log;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(joystick));
    a->Visit(MJ_NVP(stream));
    a->Visit(MJ_NVP(period_s));
    a->Visit(MJ_NVP(max_torque_Nm));
    a->Visit(MJ_NVP(load_cell));
    a->Visit(MJ_NVP(load_cell_tare_g));
    a->Visit(MJ_NVP(torque_scale));
    a->Visit(MJ_NVP(power_supply));
    a->Visit(MJ_NVP(log));
  }
};

class PowerSupply {
 public:
  struct Data {
    double voltage = 0.0;
    double current_A = 0.0;
    double power_W = 0.0;
  };

  using Signal = boost::signals2::signal<void (const Data*)>;

  PowerSupply(const boost::asio::executor& executor,
              const std::string& path)
      : executor_(executor),
        path_(path) {
    if (!path.empty()) {
      thread_ = std::thread(std::bind(&PowerSupply::Run, this));
    }
  }

  ~PowerSupply() {
    done_.store(true);
    thread_.join();
  }

  Signal* signal() { return &signal_; }

  void Run() {
    // We can't use any buffered IO, because the tmc device works on a
    // per-read-write call, so we have to know exactly how many
    // syscalls we are making.
    const int fd = ::open(path_.c_str(), O_RDWR);
    mjlib::base::system_error::throw_if(
        fd < 0, fmt::format("Error opening '{}'", path_));

    auto write = [&](const std::string& str) {
      const int r = ::write(fd, str.data(), str.size());
      mjlib::base::system_error::throw_if(
          r < 0, fmt::format("Error writing"));
    };

    char buf[256] = {};
    auto read = [&]() {
      const int r = ::read(fd, buf, sizeof(buf));
      if (r < 0) { return std::string(); }
      return std::string(buf, r);
    };

    while (!done_.load()) {
      ::usleep(100000);

      Data data;

      write("MEAS:CURR?\n");
      data.current_A = std::stod(read());

      write("MEAS:VOLT?\n");
      data.voltage = std::stod(read());

      data.power_W = data.voltage * data.current_A;

      boost::asio::post(
          executor_,
          [this, data]() {
            this->signal_(&data);
          });
    }

    ::close(fd);
  }

  boost::asio::executor executor_;
  std::string path_;
  std::thread thread_;
  Signal signal_;

  std::atomic<bool> done_{false};
};

class LoadCell {
 public:
  LoadCell(const boost::asio::executor& executor,
           const std::string& path) {
    if (!path.empty()) {
      port_.emplace(executor, path);
      port_->set_option(boost::asio::serial_port::baud_rate(115200));
      StartRead();
    }
  }

  void StartRead() {
    boost::asio::async_read_until(
        *port_, streambuf_,
        "\n", [this](auto ec, auto size) {
          this->HandleRead(ec, size);
        });
  }

  void HandleRead(const mjlib::base::error_code& ec, size_t) {
    mjlib::base::FailIf(ec);

    std::istream istr(&streambuf_);
    while (!istr.eof()) {
      std::string line;
      std::getline(istr, line);
      try {
        const double load_g = std::stod(line);
        Emit(load_g);
      } catch (std::invalid_argument&) {
      }
    }
    StartRead();
  }

  void Emit(double load_g) {
    load_signal_(load_g);
  }

  boost::signals2::signal<void (double)>* signal() { return &load_signal_; }

 private:
  std::optional<boost::asio::serial_port> port_;
  boost::asio::streambuf streambuf_;
  boost::signals2::signal<void (double)> load_signal_;
};

class Application {
 public:
  Application(boost::asio::io_context& context,
              const Options& options)
      : executor_(context.get_executor()),
        options_(options),
        timer_(executor_),
        linux_input_(executor_, options.joystick),
        asio_client_(executor_, options.stream),
        load_cell_(executor_, options.load_cell),
        power_supply_(executor_, options.power_supply),
        file_writer_(),
        log_registrar_(context, &file_writer_) {
    if (!options.log.empty()) {
      file_writer_.Open(options.log);
    }

    load_cell_.signal()->connect(
        std::bind(&Application::HandleLoad, this, pl::_1));
    power_supply_.signal()->connect(
        std::bind(&Application::HandlePowerSupply, this, pl::_1));

    log_registrar_.Register("torque_test", &signal_);
  }

  void Start() {
    asio_client_.AsyncStart(std::bind(&Application::HandleStart, this, pl::_1));
  }

  void HandleStart(const mjlib::base::error_code& ec) {
    mjlib::base::FailIf(ec);

    // Start by sending a stop command to reset any state the servo
    // might have.
    Stop();

    timer_.start(mjlib::base::ConvertSecondsToDuration(options_.period_s),
                 std::bind(&Application::HandleTimer, this, pl::_1));

    StartRead();
    UpdateDisplay();
  }

  void Stop() {
    MJ_ASSERT(outstanding_ == false);

    request_.resize(1);
    auto& item = request_[0];
    item = {};
    item.id = 1;
    item.request.WriteSingle(
        moteus::kMode, Value(static_cast<int8_t>(moteus::Mode::kStopped)));

    outstanding_ = true;
    reply_ = {};
    asio_client_.AsyncTransmit(
        &request_, &reply_,
        std::bind(&Application::HandleStop, this, pl::_1));
  }

  void HandleStop(const mjlib::base::error_code& ec) {
    outstanding_ = false;
    mjlib::base::FailIf(ec);
  }

  void HandleTimer(const mjlib::base::error_code& ec) {
    if (ec == boost::asio::error::operation_aborted) { return; }
    mjlib::base::FailIf(ec);

    if (outstanding_) { return; }

    outstanding_ = true;

    const float command_Nm = GetCommandTorqueNm();

    request_.resize(1);
    auto& item = request_[0];
    item = {};
    item.id = 1;
    item.request.WriteSingle(
        moteus::kMode, Value(static_cast<int8_t>(moteus::Mode::kPosition)));
    item.request.WriteMultiple(
        moteus::kCommandPosition,
        {Value(0.0f),  // position
              Value(0.0f),  // velocity
              Value(command_Nm),
              Value(0.0f),  // kp_scale
              Value(0.0f),  // kd_scale
              });

    item.request.ReadMultiple(moteus::kMode, 5, moteus::kFloat);
    item.request.ReadMultiple(moteus::kVoltage, 3, moteus::kFloat);

    current_reply_ = {};
    asio_client_.AsyncTransmit(
        &request_, &current_reply_,
        std::bind(&Application::HandleClient, this, pl::_1));
  }

  void StartRead() {
    linux_input_.AsyncRead(
        &event_, std::bind(
            &Application::HandleJoystick, this, pl::_1));
  }

  void HandleJoystick(const mjlib::base::error_code& ec) {
    mjlib::base::FailIf(ec);

    UpdateDisplay();

    StartRead();
  }

  void HandleClient(const mjlib::base::error_code& ec) {
    outstanding_ = false;

    if (ec == boost::asio::error::operation_aborted) { return; }
    mjlib::base::FailIf(ec);

    UpdateDisplay();
  }

  double GetCommandTorqueNm() {
    return linux_input_.abs_info(ABS_Y).scaled() *
        options_.max_torque_Nm;
  }

  void HandleLoad(double load_g) {
    measured_torque_Nm_ = (load_g - options_.load_cell_tare_g) *
                          options_.torque_scale;
    UpdateDisplay();
  }

  void HandlePowerSupply(const PowerSupply::Data* data) {
    power_supply_data_ = *data;
    UpdateDisplay();
  }

  void UpdateDisplay() {
    double command = GetCommandTorqueNm();

    auto get = [&](auto moteus_reg) -> double {
      for (const auto& item : current_reply_) {
        if (item.reg == moteus_reg) {
          return std::get<float>(std::get<Value>(item.value));
        }
      }
      return std::numeric_limits<double>::signaling_NaN();
    };

    Data data;
    data.timestamp = mjlib::io::Now(executor_.context());
    data.command_Nm = -command;
    data.measured.torque_Nm = measured_torque_Nm_;
    data.measured.voltage = power_supply_data_.voltage;
    data.measured.current_A = power_supply_data_.current_A;
    data.measured.power_W = power_supply_data_.power_W;

    data.qdd100.mode = get(moteus::kMode);
    data.qdd100.position_deg = get(moteus::kPosition) * 360.0;
    data.qdd100.velocity_dps = get(moteus::kVelocity) * 360.0;
    data.qdd100.torque_Nm = get(moteus::kTorque);
    data.qdd100.q_current_A = get(moteus::kQCurrent);
    data.qdd100.temperature_C = get(moteus::kTemperature);

    std::cout << fmt::format(
        " cmd={:5.1f}Nm  mt={:5.1f}Nm  mp={:4.1f}W  qdd100[ mode={:4} "
        "pos={:7.1f}deg vel={:6.1f}dps torque={:5.1f}Nm i={:4.1f}A "
        "t={:3.0f}C ]  \r",
        command,
        measured_torque_Nm_,
        power_supply_data_.power_W,
        MapMode(data.qdd100.mode),
        data.qdd100.position_deg,
        data.qdd100.velocity_dps,
        data.qdd100.torque_Nm,
        data.qdd100.q_current_A,
        data.qdd100.temperature_C);
    std::cout.flush();

    signal_(&data);
  }

  std::string MapMode(float value) {
    if (!std::isfinite(value)) { return "unk"; }
    return mode_text_.at(static_cast<int>(value));
  }

  struct Data {
    boost::posix_time::ptime timestamp;
    double command_Nm = 0.0;

    struct Measured {
      double torque_Nm = 0.0;
      double voltage = 0.0;
      double current_A = 0.0;
      double power_W = 0.0;

      template <typename Archive>
      void Serialize(Archive* a) {
        a->Visit(MJ_NVP(torque_Nm));
        a->Visit(MJ_NVP(voltage));
        a->Visit(MJ_NVP(current_A));
        a->Visit(MJ_NVP(power_W));
      }
    };

    Measured measured;

    struct Qdd100 {
      double mode = 0.0;
      double position_deg = 0.0;
      double velocity_dps = 0.0;
      double torque_Nm = 0.0;
      double q_current_A = 0.0;
      double temperature_C = 0.0;

      template <typename Archive>
      void Serialize(Archive* a) {
        a->Visit(MJ_NVP(position_deg));
        a->Visit(MJ_NVP(velocity_dps));
        a->Visit(MJ_NVP(torque_Nm));
        a->Visit(MJ_NVP(q_current_A));
        a->Visit(MJ_NVP(temperature_C));
      }
    };

    Qdd100 qdd100;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(timestamp));
      a->Visit(MJ_NVP(command_Nm));
      a->Visit(MJ_NVP(measured));
      a->Visit(MJ_NVP(qdd100));
    }
  };

  boost::asio::executor executor_;
  const Options options_;
  mjlib::io::RepeatingTimer timer_;
  base::LinuxInput linux_input_;
  mjlib::multiplex::StreamAsioClientBuilder asio_client_;
  LoadCell load_cell_;
  PowerSupply power_supply_;

  mjlib::telemetry::FileWriter file_writer_;
  base::TelemetryLogRegistrar log_registrar_;

  base::LinuxInput::Event event_;

  using AsioClient = mjlib::multiplex::AsioClient;

  AsioClient::Request request_;
  AsioClient::Reply current_reply_;
  AsioClient::Reply reply_;
  bool outstanding_ = false;

  double measured_torque_Nm_ = 0.0;
  PowerSupply::Data power_supply_data_;

  boost::signals2::signal<void (const Data*)> signal_;

  const std::map<int, std::string> mode_text_ = {
    { 0, "stop" },
    { 1, "flt" },
    { 2, "" },
    { 3, "" },
    { 4, "" },
    { 5, "pwm" },
    { 6, "volt" },
    { 7, "foc" },
    { 8, "vdq" },
    { 9, "cur" },
    { 10, "pos" },
    { 11, "tmt" },
    { 12, "zero" },
  };
};

}

int do_main(int argc, char**argv) {
  Options options;

  auto group = mjlib::base::ClippArchive().Accept(&options).group();
  mjlib::base::ClippParse(argc, argv, group);

  boost::asio::io_context context;
  Application application(context, options);

  application.Start();

  boost::asio::signal_set signals(context, SIGINT, SIGTERM);
  signals.async_wait([&](auto _1, auto _2) {
      context.stop();
    });

  context.run();
  return 0;
}

}
}

int main(int argc, char** argv) {
  return mjmech::mech::do_main(argc, argv);
}
