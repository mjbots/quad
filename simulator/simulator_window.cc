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

#include "mjlib/base/fail.h"
#include "mjlib/base/json5_read_archive.h"
#include "mjlib/io/now.h"
#include "mjlib/io/debug_deadline_service.h"

#include "base/common.h"
#include "base/context_full.h"

#include "mech/quadruped.h"

#include "simulator/make_robot.h"

namespace dd = dart::dynamics;
namespace ds = dart::simulation;

namespace mjmech {
namespace simulator {

namespace {
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
      const IdRequest&, SingleReply*, mjlib::io::ErrorCallback) override {
  }

  void AsyncRegisterMultiple(
      const std::vector<IdRequest>&, Reply*,
      mjlib::io::ErrorCallback) override {
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

 private:
  boost::asio::executor executor_;
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

  void ReadImu(mech::AttitudeData*,
               mjlib::io::ErrorCallback callback) override {
  }

  void AsyncStart(mjlib::io::ErrorCallback callback) {
    boost::asio::post(
        executor_,
        std::bind(std::move(callback), mjlib::base::error_code()));
  }

 private:
  boost::asio::executor executor_;
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
  Impl(base::Context& context)
      : context_(context.context),
        executor_(context.executor),
        quadruped_(context) {

    quadruped_.m()->multiplex_client->Register<SimMultiplex>("sim");
    quadruped_.m()->multiplex_client->set_default("sim");

    quadruped_.m()->imu_client->Register<SimImu>("sim");
    quadruped_.m()->imu_client->set_default("sim");

    floor_ = MakeFloor();
    mech::QuadrupedConfig config;
    {
      std::ifstream inf(options_.config);
      mjlib::base::system_error::throw_if(
          !inf.is_open(),
          fmt::format("could not open config file '{}'", options_.config));
      mjlib::base::Json5ReadArchive(inf).Accept(&config);
    }

    robot_ = MakeRobot(config);

    world_->addSkeleton(floor_);
    world_->addSkeleton(robot_);

    setWorld(world_);
  }

  void AsyncStart(mjlib::io::ErrorCallback callback) {
    if (!options_.start_disabled) {
      // Send a space bar to get us simulating.
      SimWindow::keyboard(' ', 0, 0);
    }

    quadruped_.AsyncStart(std::move(callback));
  }

  boost::asio::io_context& context_;
  boost::asio::executor executor_;

  Options options_;

  dd::SkeletonPtr floor_;
  dd::SkeletonPtr robot_;

  ds::WorldPtr world_ = std::make_shared<ds::World>();

  mech::Quadruped quadruped_;
};

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
