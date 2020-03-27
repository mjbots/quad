// Copyright 2015-2020 Josh Pieper, jjp@pobox.com.
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

#include "simulator_window.h"

#include <boost/filesystem.hpp>

#include <dart/dynamics/BoxShape.hpp>
#include <dart/dynamics/BodyNode.hpp>
#include <dart/dynamics/FreeJoint.hpp>
#include <dart/dynamics/RevoluteJoint.hpp>
#include <dart/dynamics/WeldJoint.hpp>
#include <dart/gui/LoadGlut.hpp>

#include "mjlib/base/fail.h"
#include "mjlib/base/json5_read_archive.h"
#include "mjlib/base/limit.h"
#include "mjlib/base/pid.h"
#include "mjlib/io/now.h"
#include "mjlib/io/debug_deadline_service.h"

#include "mjlib/micro/pool_ptr.h"
#include "mjlib/multiplex/micro_server.h"
#include "mjlib/multiplex/micro_datagram_server.h"

#include "base/common.h"
#include "base/context_full.h"

#include "mech/moteus.h"
#include "mech/quadruped.h"

#include "simulator/make_robot.h"

namespace dd = dart::dynamics;
namespace ds = dart::simulation;

using mjlib::base::Limit;

namespace mjmech {
namespace simulator {

namespace {
class Servo : public mjlib::multiplex::MicroServer::Server,
              public mjlib::multiplex::MicroDatagramServer {
 public:
  Servo(dd::Joint* joint, double sign) : joint_(joint), sign_(sign) {
    server_.Start(this);
  }

  ~Servo() override {}

  uint32_t Write(mjlib::multiplex::MicroServer::Register reg,
                 const mjlib::multiplex::MicroServer::Value& value) override {
    switch (static_cast<mech::moteus::Register>(reg)) {
      case mech::moteus::kMode: {
        const auto new_mode_int = mech::moteus::ReadInt(value);
        if (new_mode_int >= static_cast<int>(mech::moteus::Mode::kNumModes)) {
          return 3;
        }
        staged_command_valid_ = true;
        staged_command_ = {};
        staged_command_.mode = static_cast<mech::moteus::Mode>(new_mode_int);
        return 0;
      }
      case mech::moteus::kCommandPosition: {
        staged_command_.position = sign_ * mech::moteus::ReadPosition(value) / 360.0;
        return 0;
      }
      case mech::moteus::kCommandVelocity: {
        staged_command_.velocity = sign_ * mech::moteus::ReadVelocity(value) / 360.0;
        return 0;
      }
      case mech::moteus::kCommandPositionMaxTorque: {
        staged_command_.max_torque_Nm = mech::moteus::ReadTorque(value);
        return 0;
      }
      case mech::moteus::kCommandStopPosition: {
        staged_command_.stop_position = sign_ * mech::moteus::ReadPosition(value) / 360.0;
        return 0;
      }
      case mech::moteus::kCommandTimeout: {
        staged_command_.timeout_s = mech::moteus::ReadTime(value);
        return 0;
      }
      case mech::moteus::kCommandFeedforwardTorque: {
        staged_command_.feedforward_Nm = sign_ * mech::moteus::ReadTorque(value);
        return 0;
      }
      case mech::moteus::kCommandKpScale: {
        staged_command_.kp_scale = mech::moteus::ReadPwm(value);
        return 0;
      }
      case mech::moteus::kCommandKdScale: {
        staged_command_.kd_scale = mech::moteus::ReadPwm(value);
        return 0;
      }
      default: {
        break;
      }
    }
    return 0;
  }

  mjlib::multiplex::MicroServer::ReadResult Read(
      mjlib::multiplex::MicroServer::Register reg,
      size_t type_int) const override {
    const auto type = static_cast<mech::moteus::RegisterTypes>(type_int);

    switch (static_cast<mech::moteus::Register>(reg)) {
      case mech::moteus::kMode: {
        return mech::moteus::WriteInt(static_cast<int8_t>(state_.mode), type);
      }
      case mech::moteus::kVoltage: {
        return mech::moteus::WriteVoltage(23.0, type);
      }
      case mech::moteus::kTemperature: {
        return mech::moteus::WriteTemperature(20.0, type);
      }
      case mech::moteus::kRezeroState: {
        return mech::moteus::WriteInt(1, type);
      }
      case mech::moteus::kFault: {
        return mech::moteus::WriteInt(0, type);
      }
      case mech::moteus::kRegisterMapVersion: {
        return mech::moteus::WriteInt(
            static_cast<int8_t>(mech::moteus::kCurrentRegisterMapVersion),
            type);
      }
      case mech::moteus::kPosition: {
        return mech::moteus::WritePosition(sign_ * 360.0 * position(), type);
      }
      case mech::moteus::kVelocity: {
        return mech::moteus::WriteVelocity(sign_ * 360.0 * velocity(), type);
      }
      case mech::moteus::kTorque: {
        return mech::moteus::WriteTorque(sign_ * current_torque_Nm_, type);
      }
      // case kQCurrent: {
      // }
      // case kDCurrent: {
      // }
      default: {
        return mech::moteus::WritePwm(0, type);
      }
    }
    return {};
  }

  void AsyncRead(Header* header,
                 const mjlib::base::string_span& read_buffer,
                 const mjlib::micro::SizeCallback& read_callback) override {
    BOOST_ASSERT(!read_header_);

    read_header_ = header;
    read_buffer_ = read_buffer;
    read_callback_ = read_callback;
  }

  void AsyncWrite(const Header& header,
                  const std::string_view& write_buffer,
                  const mjlib::micro::SizeCallback& write_callback) override {
    BOOST_ASSERT(!write_header_);

    write_header_ = header;
    write_buffer_ = write_buffer;
    write_callback_ = write_callback;
  }

  mjlib::multiplex::MicroDatagramServer::Properties
  properties() const override {
    return {};
  }

  void Request(const mjlib::multiplex::RegisterRequest& request,
               mjlib::multiplex::RegisterReply* reply,
               mjlib::base::error_code*) {
    BOOST_ASSERT(!!read_header_);

    // For simulation purposes, each servo thinks it is ID 1.
    read_header_->source = request.request_reply() ? 0x80 : 0x00;
    read_header_->destination = 1;
    read_header_->size = request.buffer().size();

    const auto to_read =
        std::min<size_t>(read_buffer_.size(), request.buffer().size());
    std::memcpy(read_buffer_.data(), request.buffer().data(), to_read);

    {
      auto read_copy = std::move(read_callback_);

      read_header_ = {};
      read_buffer_ = {};
      read_callback_ = {};

      read_copy(mjlib::micro::error_code(), to_read);
    }

    *reply = {};

    // This may have resulted in a write.

    if (write_header_) {

      mjlib::base::BufferReadStream stream{write_buffer_};
      *reply = mjlib::multiplex::ParseRegisterReply(stream);

      {
        auto write_copy = std::move(write_callback_);

        write_header_ = {};
        write_buffer_ = {};
        write_callback_ = {};

        write_copy(mjlib::micro::error_code(), stream.offset());
      }
    }

    Update();
  }

  void Run(double dt_s) {
    // In case nothing else sets it.
    current_torque_Nm_ = 0.0;

    switch (state_.mode) {
      case mech::moteus::Mode::kStopped: {
        joint_->setForce(0, 0.0);
        break;
      }
      case mech::moteus::Mode::kPosition: {
        RunPosition(dt_s);
        break;
      }
      case mech::moteus::Mode::kZeroVelocity: {
        RunZeroVelocity(dt_s);
        break;
      }
      default: {
        break;
      }
    }
  }

 private:
  double position() const {
    return joint_->getPosition(0) / (2.0 * M_PI);
  }

  double velocity() const {
    return joint_->getVelocity(0) / (2.0 * M_PI);
  }

  void RunZeroVelocity(double dt_s) {
    mjlib::base::PID::ApplyOptions apply_options;
    apply_options.kp_scale = 0.0;
    apply_options.kd_scale = 1.0;

    RunPositionCommon(dt_s, apply_options);
  }

  void RunPosition(double dt_s) {
    mjlib::base::PID::ApplyOptions apply_options;
    apply_options.kp_scale = command_.kp_scale;
    apply_options.kd_scale = command_.kd_scale;

    RunPositionCommon(dt_s, apply_options);
  }

  void RunPositionCommon(
      double dt_s,
      const mjlib::base::PID::ApplyOptions& apply_options) {
    if (!std::isnan(command_.position)) {
      state_.control_position = command_.position;
      command_.position = std::numeric_limits<float>::quiet_NaN();
    } else if (std::isnan(state_.control_position)) {
      state_.control_position = position();
    }

    auto velocity_command = command_.velocity;

    const auto old_position = state_.control_position;
    state_.control_position =
        Limit(state_.control_position + velocity_command * dt_s,
              position_min_, position_max_);
    if (!std::isnan(command_.stop_position)) {
      if ((state_.control_position -
           command_.stop_position) * velocity_command > 0.0) {
        // We are moving away from the stop position.  Force it to be there.
        state_.control_position = command_.stop_position;
      }
    }
    if (state_.control_position == old_position) {
      // We have hit a limit.
      velocity_command = 0.0;
    }

    const double unlimited_torque_Nm =
        pid_position_.Apply(position(), state_.control_position,
                            velocity(), velocity_command,
                            1.0 / dt_s,
                            apply_options) +
        command_.feedforward_Nm;

    const double limited_torque_Nm =
        Limit(unlimited_torque_Nm, -command_.max_torque_Nm,
              command_.max_torque_Nm);

    current_torque_Nm_ = limited_torque_Nm;
    joint_->setForce(0, limited_torque_Nm);
  }

  void Update() {
    if (staged_command_valid_) {
      // Update with the new command.
      staged_command_valid_ = false;
      command_ = staged_command_;

      // The simulation doesn't have a state machine yet.  We just
      // instantly switch to whatever we are commanded.
      state_.mode = command_.mode;

      if (std::isnan(command_.position) &&
          !std::isnan(command_.stop_position) &&
          !std::isnan(command_.velocity) &&
          command_.velocity != 0.0) {
        command_.velocity = std::abs(command_.velocity) *
            ((command_.stop_position > position()) ? 1.0 : -1.0);
      }
    }

    staged_command_ = {};
  }

  dd::Joint* const joint_;
  const double sign_;
  mjlib::micro::SizedPool<16384> pool_;
  mjlib::multiplex::MicroServer server_{&pool_, this, {}};

  Header* read_header_ = nullptr;
  mjlib::base::string_span read_buffer_;
  mjlib::micro::SizeCallback read_callback_;

  std::optional<Header> write_header_;
  std::string_view write_buffer_;
  mjlib::micro::SizeCallback write_callback_;

  struct State {
    mech::moteus::Mode mode = mech::moteus::Mode::kStopped;
    double control_position = std::numeric_limits<double>::quiet_NaN();

    mjlib::base::PID::State pid_position;
  };

  State state_;

  mjlib::base::PID::Config pid_position_config_ = []() {
      mjlib::base::PID::Config config;
      config.kp = 450.0;
      config.ki = 100.0;
      config.ilimit = 0.0;
      config.kd = 9.0f;
      config.sign = -1.0;
      return config;
  }();

  mjlib::base::PID pid_position_{
    &pid_position_config_, &state_.pid_position};

  struct Command {
    mech::moteus::Mode mode = mech::moteus::Mode::kStopped;

    double position = 0.0f;
    double velocity = 0.0f;

    double max_torque_Nm = 100.0f;
    double stop_position = std::numeric_limits<double>::quiet_NaN();
    double feedforward_Nm = 0.0f;

    double kp_scale = 1.0f;
    double kd_scale = 1.0f;

    double timeout_s = 0.0f;
  };

  Command staged_command_;
  bool staged_command_valid_ = false;

  Command command_;

  double current_torque_Nm_ = 0.0;

  const double position_min_ = -1.0;
  const double position_max_ = 1.0;
};

class SimMultiplex : public mjlib::multiplex::AsioClient {
 public:
  struct Options {
    template <typename Archive>
    void Serialize(Archive*) {}
  };

  SimMultiplex(boost::asio::executor executor, const Options&)
      : executor_(executor) {}
  ~SimMultiplex() override {}

  void AsyncRegister(
      const IdRequest& request, SingleReply* single_reply,
      mjlib::io::ErrorCallback callback) override {
    mjlib::base::error_code ec;
    DoRequest(request, single_reply, &ec);
    boost::asio::post(
        executor_,
        std::bind(std::move(callback), ec));
  }

  void AsyncRegisterMultiple(
      const std::vector<IdRequest>& requests, Reply* reply,
      mjlib::io::ErrorCallback callback) override {
    *reply = {};
    mjlib::base::error_code ec;
    for (const auto& id_request : requests) {
      reply->replies.push_back({});
      DoRequest(id_request, &reply->replies.back(), &ec);
    }
    boost::asio::post(
        executor_,
        std::bind(std::move(callback), ec));
  }

  mjlib::io::SharedStream MakeTunnel(
      uint8_t id, uint32_t channel, const TunnelOptions& options) override {
    return {};
  }

  void AsyncStart(mjlib::io::ErrorCallback callback) {
    boost::asio::post(
        executor_,
        std::bind(std::move(callback), mjlib::base::error_code()));
  }

  void AddServo(dd::Joint* joint, int id) {
    servos_[id] = std::make_unique<Servo>(joint, signs_.at(id));
  }

  void Run(double dt_s) {
    for (auto& pair : servos_) {
      pair.second->Run(dt_s);
    }
  }

 private:
  void DoRequest(const mjlib::multiplex::AsioClient::IdRequest& id_request,
                 mjlib::multiplex::AsioClient::SingleReply* reply,
                 mjlib::base::error_code* ec) {
    const auto id = id_request.id;
    const auto it = servos_.find(id);
    if (it == servos_.end()) {
      // We don't have this servo.
      *ec = mjlib::base::error_code::einval(
          fmt::format("unknown servo {}", id));
      return;
    }

    reply->id = id;
    it->second->Request(id_request.request, &reply->reply, ec);
  }

  boost::asio::executor executor_;
  std::map<int, std::unique_ptr<Servo>> servos_;

  std::map<int, double> signs_{
    {1, 1.0},
    {2, 1.0},
    {3, 1.0},
    {4, -1.0},
    {5, -1.0},
    {6, 1.0},
    {7, -1.0},
    {8, -1.0},
    {9, 1.0},
    {10, 1.0},
    {11, 1.0},
    {12, 1.0},
        };
};

class SimImu : public mech::ImuClient {
 public:
  struct Options {
    template <typename Archive>
    void Serialize(Archive*) {}
  };

  SimImu(boost::asio::executor executor, const Options&)
      : executor_(executor) {}

  ~SimImu() override {}

  void set_frame(dd::Frame* frame) {
    frame_ = frame;
  }

  void ReadImu(mech::AttitudeData* attitude,
               mjlib::io::ErrorCallback callback) override {
    if (frame_) {
      attitude->timestamp = mjlib::io::Now(executor_.context());

      // TODO(jpieper): Confirm the sign and magnitude of all these
      // things compared to the real thing.

      const Eigen::Isometry3d tf = frame_->getTransform();
      attitude->attitude = Sophus::SE3d(tf.matrix()).unit_quaternion();
      attitude->euler_deg = (180.0 / M_PI) * attitude->attitude.euler_rad();

      const Eigen::Vector3d rate_rps = frame_->getAngularVelocity();
      attitude->rate_dps.x() = base::Degrees(rate_rps[0]);
      attitude->rate_dps.y() = base::Degrees(rate_rps[1]);
      attitude->rate_dps.z() = -base::Degrees(rate_rps[2]);

      Eigen::Vector3d accel =
          frame_->getLinearAcceleration(Eigen::Vector3d(0, 0, 0));
      attitude->accel_mps2 = accel;
    } else {
      *attitude = {};
    }
    boost::asio::post(
        executor_,
        std::bind(std::move(callback), mjlib::base::error_code()));
  }

  void AsyncStart(mjlib::io::ErrorCallback callback) {
    boost::asio::post(
        executor_,
        std::bind(std::move(callback), mjlib::base::error_code()));
  }

  void set_data(const mech::AttitudeData& data) {
    data_ = data;
  }

 private:
  boost::asio::executor executor_;
  mech::AttitudeData data_;
  dd::Frame* frame_ = nullptr;
};
}

struct Options {
  bool start_disabled = false;
  std::string config = "configs/quada1.cfg";

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(start_disabled));
    a->Visit(MJ_NVP(config));
  }
};

class SimulatorWindow::Impl : public dart::gui::glut::SimWindow {
 public:
  static Impl* g_impl_;

  Impl(base::Context& context)
      : context_(context.context),
        executor_(context.executor),
        quadruped_(context) {
    g_impl_ = this;

    quadruped_.m()->multiplex_client->Register<SimMultiplex>("sim");
    quadruped_.m()->multiplex_client->set_default("sim");

    quadruped_.m()->imu_client->Register<SimImu>("sim");
    quadruped_.m()->imu_client->set_default("sim");

    floor_ = MakeFloor();
    {
      std::ifstream inf(options_.config);
      mjlib::base::system_error::throw_if(
          !inf.is_open(),
          fmt::format("could not open config file '{}'", options_.config));
      mjlib::base::Json5ReadArchive(inf).Accept(&quadruped_config_);
    }

    robot_ = MakeRobot(quadruped_config_);

    world_->addSkeleton(floor_);
    world_->addSkeleton(robot_);

    setWorld(world_);
  }

  void AsyncStart(mjlib::io::ErrorCallback callback) {
    // The GLUT timer will actually process our event loop, so it
    // needs to be going right away.
    StartGlutTimer();

    if (!options_.start_disabled) {
      // Send a space bar to get us simulating.
      SimWindow::keyboard(' ', 0, 0);
    }

    quadruped_.AsyncStart([this, callback=std::move(callback)](
                              const auto& ec) mutable {
        this->HandleStart(ec, std::move(callback));
      });
  }

  void HandleStart(const mjlib::base::error_code& ec,
                   mjlib::io::ErrorCallback callback) {
    if (ec) {
      boost::asio::post(
          executor_,
          std::bind(std::move(callback), ec));
      return;
    }

    imu_ = dynamic_cast<SimImu*>(quadruped_.m()->imu_client->selected());
    BOOST_ASSERT(imu_);
    multiplex_ = dynamic_cast<SimMultiplex*>(
        quadruped_.m()->multiplex_client->selected());
    BOOST_ASSERT(multiplex_);

    imu_->set_frame(robot_->getBodyNode("robot"));

    for (int leg = 0; leg < 4; leg++) {
      const auto& leg_config = quadruped_config_.legs.at(leg);

      std::string leg_prefix = fmt::format("leg{}", leg);
      auto add_servo = [&](auto name, auto id) {
        multiplex_->AddServo(
            robot_->getBodyNode(leg_prefix + name)->getParentJoint(), id);
      };
      add_servo("_shoulder", leg_config.ik.shoulder.id);
      add_servo("_femur", leg_config.ik.femur.id);
      add_servo("_tibia", leg_config.ik.tibia.id);
    }

    boost::asio::post(
        executor_,
        std::bind(std::move(callback), ec));
  }

  static void GlobalHandleGlutTimer(int) {
    g_impl_->HandleGlutTimer();
  }

  void StartGlutTimer() {
    glutTimerFunc(1, &GlobalHandleGlutTimer, 0);
  }

  void HandleGlutTimer() {
    context_.poll();
    context_.reset();
    StartGlutTimer();
  }

  void timeStepping() override {
    const double dt_s = world_->getTimeStep();
    multiplex_->Run(dt_s);

    SimWindow::timeStepping();
  }

  boost::asio::io_context& context_;
  boost::asio::executor executor_;

  Options options_;

  dd::SkeletonPtr floor_;
  dd::SkeletonPtr robot_;

  ds::WorldPtr world_ = std::make_shared<ds::World>();

  mech::QuadrupedConfig quadruped_config_;
  mech::Quadruped quadruped_;

  SimImu* imu_ = nullptr;
  SimMultiplex* multiplex_ = nullptr;
};

SimulatorWindow::Impl* SimulatorWindow::Impl::g_impl_ = nullptr;

SimulatorWindow::SimulatorWindow(base::Context& context)
    : impl_(std::make_unique<Impl>(context)) {}

SimulatorWindow::~SimulatorWindow() {}

clipp::group SimulatorWindow::program_options() {
  return clipp::group(
      mjlib::base::ClippArchive().Accept(&impl_->options_).release(),
      impl_->quadruped_.program_options());
}

void SimulatorWindow::AsyncStart(mjlib::io::ErrorCallback callback) {
  impl_->AsyncStart(std::move(callback));
}

void SimulatorWindow::InitWindow(int x, int y, const char* name) {
  impl_->initWindow(x, y, name);
}

}
}
