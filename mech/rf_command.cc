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

#include <boost/asio/io_context.hpp>

#include "mjlib/base/buffer_stream.h"
#include "mjlib/base/clipp.h"
#include "mjlib/base/clipp_archive.h"
#include "mjlib/io/now.h"
#include "mjlib/io/stream_factory.h"
#include "mjlib/telemetry/format.h"

#include "base/point3d.h"
#include "base/saturate.h"
#include "base/sophus.h"

#include "gl/gl_imgui.h"
#include "gl/window.h"

#include "mech/nrfusb_client.h"
#include "mech/quadruped_command.h"

namespace pl = std::placeholders;

namespace mjmech {
namespace mech {

constexpr double kMaxForwardVelocity_mm_s = 300.0;
constexpr double kMaxLateralVelocity_mm_s = 100.0;
constexpr double kMaxRotation_rad_s = (30.0 / 180.0) * M_PI;

constexpr double kMovementEpsilon_mm_s = 25.0;
constexpr double kMovementEpsilon_rad_s = (7.0 / 180.0) * M_PI;

namespace {
class SlotCommand {
 public:
  SlotCommand(mjlib::io::AsyncStream* stream)
      : executor_(stream->get_executor()),
        nrfusb_(stream) {
    StartRead();
  }

  using Slot = NrfusbClient::Slot;

  void Command(QuadrupedCommand::Mode mode,
               const Sophus::SE3d& pose_mm_RB,
               const base::Point3D& v_mm_s_R,
               const base::Point3D& w_LR) {
    {
      Slot slot0;
      slot0.priority = 0xffffffff;
      slot0.size = 1;
      slot0.data[0] = static_cast<uint8_t>(mode);
      nrfusb_.tx_slot(0, slot0);
    }

    {
      Slot slot2;
      slot2.priority = 0xffffffff;
      slot2.size = 6;
      mjlib::base::BufferWriteStream bstream({slot2.data, slot2.size});
      mjlib::telemetry::WriteStream tstream{bstream};
      tstream.Write(base::Saturate<int16_t>(v_mm_s_R.x()));
      tstream.Write(base::Saturate<int16_t>(v_mm_s_R.y()));
      tstream.Write(base::Saturate<int16_t>(32767.0 * w_LR.z() / (2 * M_PI)));
      nrfusb_.tx_slot(2, slot2);
    }
  }

  struct Data {
    QuadrupedCommand::Mode mode;
    int tx_count = 0;
    int rx_count = 0;
    base::Point3D v_mm_s_R;
    base::Point3D w_LB;
    double min_voltage = 0.0;
    double max_voltage = 0.0;
    double min_temp_C = 0.0;
    double max_temp_C = 0.0;
    int fault = 0;
  };

  const Data& data() const { return data_; }

 private:
  void StartRead() {
    bitfield_ = 0;
    nrfusb_.AsyncWaitForSlot(
        &bitfield_, std::bind(&SlotCommand::HandleRead, this, pl::_1));
  }

  void HandleRead(const mjlib::base::error_code& ec) {
    mjlib::base::FailIf(ec);

    const auto now = mjlib::io::Now(executor_.context());
    receive_times_.push_back(now);
    while (base::ConvertDurationToSeconds(now - receive_times_.front()) > 1.0) {
      receive_times_.pop_front();
    }

    data_.rx_count = receive_times_.size();

    for (int i = 0; i < 15; i++) {
      if ((bitfield_ & (1 << i)) == 0) { continue; }

      const auto slot = nrfusb_.rx_slot(i);
      if (i == 0) {
        data_.mode = static_cast<QuadrupedCommand::Mode>(slot.data[0]);
        data_.tx_count = slot.data[1];
      } else if (i == 1) {
        mjlib::base::BufferReadStream bs({slot.data, slot.size});
        mjlib::telemetry::ReadStream ts{bs};
        const double v_mm_s_R_x = *ts.Read<int16_t>();
        const double v_mm_s_R_y = *ts.Read<int16_t>();
        const double w_LB_z = *ts.Read<int16_t>();
        data_.v_mm_s_R = base::Point3D(v_mm_s_R_x, v_mm_s_R_y, 0.0);
        data_.w_LB = base::Point3D(0., 0., w_LB_z);
      } else if (i == 8) {
        data_.min_voltage = slot.data[0];
        data_.max_voltage = slot.data[1];
        data_.min_temp_C = slot.data[2];
        data_.max_temp_C = slot.data[3];
        data_.fault = slot.data[4];
      }
    }

    StartRead();
  }

  boost::asio::executor executor_;
  uint16_t bitfield_ = 0;
  NrfusbClient nrfusb_;
  Data data_;

  std::deque<boost::posix_time::ptime> receive_times_;
};

void DrawTelemetry(const SlotCommand* slot_command) {
  ImGui::Begin("Telemetry");

  if (slot_command) {
    const auto& d = slot_command->data();

    ImGui::Text("Mode: %s",
                QuadrupedCommand::ModeMapper().at(d.mode));
    ImGui::Text("tx/rx: %d/%d",
                d.tx_count, d.rx_count);
    ImGui::Text("cmd: (%4.0f, %4.0f, %4.0f)",
                d.v_mm_s_R.x(), d.v_mm_s_R.y(),
                d.w_LB.z());
    ImGui::Text("V: %.0f/%.0f", d.min_voltage, d.max_voltage);
    ImGui::Text("T: %.0f/%.0f", d.min_temp_C, d.max_temp_C);
    ImGui::Text("flt: %d", d.fault);

  } else {
    ImGui::Text("N/A");
  }

  ImGui::End();
}

void DrawGamepad(const GLFWgamepadstate& state) {
  ImGui::SetNextWindowPos({400, 50}, ImGuiCond_FirstUseEver);
  ImGui::Begin("Gamepad");

  ImGui::Text("A=%d B=%d X=%d Y=%d",
              state.buttons[GLFW_GAMEPAD_BUTTON_A],
              state.buttons[GLFW_GAMEPAD_BUTTON_B],
              state.buttons[GLFW_GAMEPAD_BUTTON_X],
              state.buttons[GLFW_GAMEPAD_BUTTON_Y]);
  ImGui::Text("DPAD U=%d R=%d D=%d L=%d",
              state.buttons[GLFW_GAMEPAD_BUTTON_DPAD_UP],
              state.buttons[GLFW_GAMEPAD_BUTTON_DPAD_RIGHT],
              state.buttons[GLFW_GAMEPAD_BUTTON_DPAD_DOWN],
              state.buttons[GLFW_GAMEPAD_BUTTON_DPAD_LEFT]);
  ImGui::Text("LEFT: %.3f %.3f",
              state.axes[GLFW_GAMEPAD_AXIS_LEFT_X],
              state.axes[GLFW_GAMEPAD_AXIS_LEFT_Y]);
  ImGui::Text("RIGHT: %.3f %.3f",
              state.axes[GLFW_GAMEPAD_AXIS_RIGHT_X],
              state.axes[GLFW_GAMEPAD_AXIS_RIGHT_Y]);

  ImGui::End();
}

enum GaitMode {
  kStop,
  kRest,
  kWalk,
  kJump,
  kZero,
  kNumGaitModes,
};

int do_main(int argc, char** argv) {
  mjlib::io::StreamFactory::Options stream;
  stream.type = mjlib::io::StreamFactory::Type::kSerial;
  stream.serial_port = "/dev/nrfusb";

  clipp::group group =
      mjlib::base::ClippArchive("stream.").Accept(&stream).release();
  mjlib::base::ClippParse(argc, argv, group);

  mjlib::io::SharedStream shared_stream;
  std::unique_ptr<SlotCommand> slot_command;

  boost::asio::io_context context;
  boost::asio::executor executor = context.get_executor();
  mjlib::io::StreamFactory stream_factory(executor);
  stream_factory.AsyncCreate(stream, [&](auto ec, auto shared_stream_in) {
      mjlib::base::FailIf(ec);
      shared_stream = shared_stream_in;  // so it sticks around
      slot_command = std::make_unique<SlotCommand>(shared_stream.get());
    });

  gl::Window window(1280, 720, "quad RF command");
  gl::GlImGui imgui(window);

  QuadrupedCommand::Mode command_mode = QuadrupedCommand::Mode::kStopped;
  int pending_gait_mode = kStop;

  std::vector<bool> old_gamepad_buttons;
  old_gamepad_buttons.resize(GLFW_GAMEPAD_BUTTON_LAST + 1);

  while (!window.should_close()) {
    context.poll(); context.reset();
    window.PollEvents();
    GLFWgamepadstate gamepad;
    glfwGetGamepadState(GLFW_JOYSTICK_1, &gamepad);

    std::vector<bool> gamepad_pressed;
    for (int i = 0; i <= GLFW_GAMEPAD_BUTTON_LAST; i++) {
      gamepad_pressed.push_back(gamepad.buttons[i] && !old_gamepad_buttons[i]);
      old_gamepad_buttons[i] = gamepad.buttons[i];
    }

    imgui.NewFrame();

    glClearColor(0.45f, 0.55f, 0.60f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);

    TRACE_GL_ERROR();

    DrawTelemetry(slot_command.get());
    DrawGamepad(gamepad);

    const bool gait_select_mode =
        gamepad.buttons[GLFW_GAMEPAD_BUTTON_Y];

    if (gait_select_mode) {
      if (gamepad_pressed[GLFW_GAMEPAD_BUTTON_DPAD_UP]) {
        pending_gait_mode =
            (pending_gait_mode + kNumGaitModes - 1) % kNumGaitModes;
      }
      if (gamepad_pressed[GLFW_GAMEPAD_BUTTON_DPAD_DOWN]) {
        pending_gait_mode =
            (pending_gait_mode + 1) % kNumGaitModes;
      }
    }

    {
      ImGui::Begin("Gait");
      const bool was_collapsed = ImGui::IsWindowCollapsed();

      ImGui::SetWindowCollapsed(!gait_select_mode);
      ImGui::SetWindowPos({800, 50}, ImGuiCond_FirstUseEver);

      ImGui::RadioButton("Stop", &pending_gait_mode, kStop);
      ImGui::RadioButton("Rest", &pending_gait_mode, kRest);
      ImGui::RadioButton("Walk", &pending_gait_mode, kWalk);
      ImGui::RadioButton("Jump", &pending_gait_mode, kJump);
      ImGui::RadioButton("Zero", &pending_gait_mode, kZero);

      ImGui::End();

      if (!gait_select_mode && !was_collapsed) {
        // Update our command.
        command_mode = [&]() {
          switch (pending_gait_mode) {
            case kStop: return QuadrupedCommand::Mode::kStopped;
            case kRest: return QuadrupedCommand::Mode::kRest;
            case kWalk: return QuadrupedCommand::Mode::kWalk;
            case kJump: return QuadrupedCommand::Mode::kJump;
            case kZero: return QuadrupedCommand::Mode::kZeroVelocity;
          }
          mjlib::base::AssertNotReached();
        }();
      }
    }

    base::Point3D v_mm_s_R;
    v_mm_s_R.x() = -kMaxForwardVelocity_mm_s * gamepad.axes[GLFW_GAMEPAD_AXIS_LEFT_Y];
    v_mm_s_R.y() = kMaxLateralVelocity_mm_s * gamepad.axes[GLFW_GAMEPAD_AXIS_LEFT_X];

    base::Point3D w_LR;
    w_LR.z() = kMaxRotation_rad_s * gamepad.axes[GLFW_GAMEPAD_AXIS_RIGHT_X];

    Sophus::SE3d pose_mm_RB;

    const QuadrupedCommand::Mode actual_command_mode = [&]() {
      const bool movement_commanded = (
          v_mm_s_R.norm() > kMovementEpsilon_mm_s ||
          w_LR.norm() > kMovementEpsilon_rad_s);
#if 0
      if (!movement_commanded &&
          (command_mode == QuadrupedCommand::Mode::kWalk ||
           command_mode == QuadrupedCommand::Mode::kJump)) {
        return QuadrupedCommand::Mode::kRest;
      }
#endif
      return command_mode;
    }();

    {
      ImGui::Begin("Command");
      ImGui::SetWindowPos({50, 400}, ImGuiCond_FirstUseEver);
      ImGui::Text("Mode  : %14s", QuadrupedCommand::ModeMapper().at(command_mode));
      ImGui::Text("Actual: %14s",
                  QuadrupedCommand::ModeMapper().at(actual_command_mode));
      ImGui::Text("cmd: (%4.0f, %4.0f, %6.3f)",
                  v_mm_s_R.x(),
                  v_mm_s_R.y(),
                  w_LR.z());
      ImGui::Text("pose x/y: (%3.0f, %3.0f)",
                  pose_mm_RB.translation().x(),
                  pose_mm_RB.translation().y());
    }

    if (slot_command) {
      slot_command->Command(actual_command_mode, pose_mm_RB, v_mm_s_R, w_LR);
    }

    imgui.Render();
    window.SwapBuffers();
  }
  return 0;
}
}

}
}

int main(int argc, char** argv) {
  return mjmech::mech::do_main(argc, argv);
}
