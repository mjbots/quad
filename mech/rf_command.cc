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

#include <sophus/se2.hpp>

#include "mjlib/base/buffer_stream.h"
#include "mjlib/base/clipp.h"
#include "mjlib/base/clipp_archive.h"
#include "mjlib/base/limit.h"
#include "mjlib/io/now.h"
#include "mjlib/io/stream_factory.h"
#include "mjlib/telemetry/format.h"

#include "base/interpolate.h"
#include "base/point3d.h"
#include "base/saturate.h"
#include "base/sophus.h"

#include "ffmpeg/codec.h"
#include "ffmpeg/file.h"
#include "ffmpeg/frame.h"
#include "ffmpeg/packet.h"
#include "ffmpeg/swscale.h"

#include "gl/flat_rgb_texture.h"
#include "gl/gl_imgui.h"
#include "gl/program.h"
#include "gl/shader.h"
#include "gl/vertex_array_object.h"
#include "gl/vertex_buffer_object.h"
#include "gl/window.h"

#include "mech/expo_map.h"
#include "mech/nrfusb_client.h"
#include "mech/quadruped_command.h"
#include "mech/turret_control.h"

namespace pl = std::placeholders;

namespace mjmech {
namespace mech {

const int kRemoteRobot = 0;
const int kRemoteTurret = 1;

constexpr double kMaxForwardVelocity_mm_s = 200.0;
constexpr double kMaxLateralVelocity_mm_s = 100.0;
constexpr double kMaxRotation_rad_s = (30.0 / 180.0) * M_PI;

constexpr double kMaxTurretPitch_dps = 50.0;
constexpr double kMaxTurretYaw_dps = 200.0;

constexpr double kMovementEpsilon_mm_s = 25.0;
constexpr double kMovementEpsilon_rad_s = (7.0 / 180.0) * M_PI;

namespace {
template <typename Container, typename Key>
typename Container::mapped_type get(const Container& c, const Key& key) {
  auto it = c.find(key);
  if (it == c.end()) {
    return typename Container::mapped_type{};
  }
  return it->second;
}

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
               const base::Point3D& w_LR,
               double jump_accel_mm_s2,
               TurretControl::Mode turret_mode,
               const base::Euler& turret_rate_dps,
               bool turret_track) {
    {
      Slot slot0;
      slot0.priority = 0xffffffff;
      mjlib::base::BufferWriteStream bs({slot0.data, 15});
      mjlib::telemetry::WriteStream ts{bs};

      ts.Write(static_cast<int8_t>(mode));
      if (mode == QuadrupedCommand::Mode::kJump) {
        slot0.size = 4;
        // For now, always repeat.
        ts.Write(static_cast<int8_t>(1));
        ts.Write(static_cast<uint16_t>(jump_accel_mm_s2));
      } else {
        slot0.size = 1;
      }
      nrfusb_.tx_slot(kRemoteRobot, 0, slot0);
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
      nrfusb_.tx_slot(kRemoteRobot, 2, slot2);
    }

    {
      Slot slot0;
      slot0.priority = 0xffffffff;
      slot0.size = 1;
      slot0.data[0] = static_cast<uint8_t>(turret_mode);
      nrfusb_.tx_slot(kRemoteTurret, 0, slot0);
    }

    {
      Slot slot1;
      slot1.priority = 0xffffffff;
      slot1.size = 5;
      mjlib::base::BufferWriteStream bstream({slot1.data, slot1.size});
      mjlib::telemetry::WriteStream tstream{bstream};
      tstream.Write(base::Saturate<int16_t>(32767.0 * turret_rate_dps.pitch / 400.0));
      tstream.Write(base::Saturate<int16_t>(32767.0 * turret_rate_dps.yaw / 400.0));
      tstream.Write(static_cast<int8_t>(turret_track ? 1 : 0));
      nrfusb_.tx_slot(kRemoteTurret, 1, slot1);
    }
  }

  struct Turret {
    TurretControl::Mode mode = TurretControl::Mode::kStop;
    int rx_count = 0;
    double imu_pitch_deg = 0.0;
    double imu_yaw_deg = 0.0;
    double imu_pitch_rate_dps = 0.0;
    double imu_yaw_rate_dps = 0.0;
    double servo_pitch_deg = 0.0;
    double servo_yaw_deg = 0.0;
    double min_voltage = 0.0;
    double max_voltage = 0.0;
    double min_temp_C = 0.0;
    double max_temp_C = 0.0;
    int fault = 0;
  };

  struct Data {
    QuadrupedCommand::Mode mode = QuadrupedCommand::Mode::kStopped;
    int tx_count = 0;
    int rx_count = 0;
    base::Point3D v_mm_s_R;
    base::Point3D w_LB;
    double min_voltage = 0.0;
    double max_voltage = 0.0;
    double min_temp_C = 0.0;
    double max_temp_C = 0.0;
    int fault = 0;

    Turret turret;
  };

  const Data& data() const { return data_; }

 private:
  void StartRead() {
    bitfield_ = 0;
    nrfusb_.AsyncWaitForSlot(
        &remote_, &bitfield_, std::bind(&SlotCommand::HandleRead, this, pl::_1));
  }

  void HandleRead(const mjlib::base::error_code& ec) {
    mjlib::base::FailIf(ec);

    ProcessRead();

    StartRead();
  }

  void ProcessRead() {
    if (remote_ == kRemoteRobot) {
      ProcessRobot();
    } else if (remote_ == kRemoteTurret) {
      ProcessTurret();
    }
  }

  void ProcessRobot() {
    const auto now = mjlib::io::Now(executor_.context());
    receive_times_.push_back(now);
    while (base::ConvertDurationToSeconds(now - receive_times_.front()) > 1.0) {
      receive_times_.pop_front();
    }

    data_.rx_count = receive_times_.size();

    for (int i = 0; i < 15; i++) {
      if ((bitfield_ & (1 << i)) == 0) { continue; }

      const auto slot = nrfusb_.rx_slot(remote_, i);
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
        data_.min_voltage = slot.data[0] * 0.25;
        data_.max_voltage = slot.data[1] * 0.25;
        data_.min_temp_C = slot.data[2];
        data_.max_temp_C = slot.data[3];
        data_.fault = slot.data[4];
      }
    }
  }

  void ProcessTurret() {
    auto& t = data_.turret;
    for (int i = 0; i < 15; i++) {
      if ((bitfield_ & (1 << i)) == 0) { continue; }

      const auto slot = nrfusb_.rx_slot(remote_, i);
      mjlib::base::BufferReadStream bs({slot.data, slot.size});
      mjlib::telemetry::ReadStream ts{bs};
      if (i == 0) {
        t.mode = static_cast<TurretControl::Mode>(slot.data[0]);
        t.rx_count = slot.data[1];
      } else if (i == 1) {
        t.imu_pitch_deg = *ts.Read<int16_t>() / 32767.0 * 180.0;
        t.imu_yaw_deg = *ts.Read<int16_t>() / 32767.0 * 180.0;
        t.imu_pitch_rate_dps = *ts.Read<int16_t>() / 32767.0 * 400.0;
        t.imu_yaw_rate_dps = *ts.Read<int16_t>() / 32767.0 * 400.0;
      } else if (i == 2) {
        t.servo_pitch_deg = *ts.Read<int16_t>() / 32767.0 * 180.0;
        t.servo_yaw_deg = *ts.Read<int16_t>() / 32767.0 * 180.0;
      } else if (i == 8) {
        t.min_voltage = slot.data[0] * 0.25;
        t.max_voltage = slot.data[1] * 0.25;
        t.min_temp_C = slot.data[2];
        t.max_temp_C = slot.data[3];
        t.fault = slot.data[4];
      }
    }
  }

  boost::asio::executor executor_;
  int remote_ = 0;
  uint16_t bitfield_ = 0;
  NrfusbClient nrfusb_;
  Data data_;

  std::deque<boost::posix_time::ptime> receive_times_;
};

void DrawTelemetry(const SlotCommand* slot_command, bool turret) {
  ImGui::Begin("Telemetry");

  if (slot_command) {
    const auto& d = slot_command->data();

    ImGui::Text("Mode: %s",
                get(QuadrupedCommand::ModeMapper(), d.mode));
    ImGui::Text("tx/rx: %d/%d",
                d.tx_count, d.rx_count);
    ImGui::Text("cmd: (%4.0f, %4.0f, %4.0f)",
                d.v_mm_s_R.x(), d.v_mm_s_R.y(),
                d.w_LB.z());
    ImGui::Text("V: %.2f/%.2f", d.min_voltage, d.max_voltage);
    ImGui::Text("T: %.0f/%.0f", d.min_temp_C, d.max_temp_C);
    ImGui::Text("flt: %d", d.fault);

  } else {
    ImGui::Text("N/A");
  }

  ImGui::End();

  if (turret) {
    ImGui::Begin("Turret");
    ImGui::SetWindowPos({50, 200}, ImGuiCond_FirstUseEver);
    if (slot_command) {
      const auto& t = slot_command->data().turret;
      ImGui::Text("Mode: %s",
                  get(TurretControl::ModeMapper(), t.mode));
      ImGui::Text("tx: %d", t.rx_count);
      ImGui::Text("pitch/yaw: (%6.1f, %6.1f)",
                  t.imu_pitch_deg, t.imu_yaw_deg);
      ImGui::Text("prate/yrate: (%4.0f, %4.0f)",
                  t.imu_pitch_rate_dps, t.imu_yaw_rate_dps);
      ImGui::Text("spitch/syaw: (%6.1f, %6.1f)",
                  t.servo_pitch_deg, t.servo_yaw_deg);
      ImGui::Text("V: %.2f/%.2f", t.min_voltage, t.max_voltage);
      ImGui::Text("T: %.0f/%.0f", t.min_temp_C, t.max_temp_C);
      ImGui::Text("flt: %d", t.fault);
    } else {
      ImGui::Text("N/A");
    }
    ImGui::End();
  }
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
  ImGui::Text("BUMP L=%d Y=%d",
              state.buttons[GLFW_GAMEPAD_BUTTON_LEFT_BUMPER],
              state.buttons[GLFW_GAMEPAD_BUTTON_RIGHT_BUMPER]);
  ImGui::Text("LEFT: %.3f %.3f",
              state.axes[GLFW_GAMEPAD_AXIS_LEFT_X],
              state.axes[GLFW_GAMEPAD_AXIS_LEFT_Y]);
  ImGui::Text("RIGHT: %.3f %.3f",
              state.axes[GLFW_GAMEPAD_AXIS_RIGHT_X],
              state.axes[GLFW_GAMEPAD_AXIS_RIGHT_Y]);
  ImGui::Text("TRIG: %.3f  %.3f",
              state.axes[GLFW_GAMEPAD_AXIS_LEFT_TRIGGER],
              state.axes[GLFW_GAMEPAD_AXIS_RIGHT_TRIGGER]);

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

void DrawGait(const GLFWgamepadstate& gamepad,
              const std::vector<bool>& gamepad_pressed,
              int* pending_gait_mode,
              QuadrupedCommand::Mode* command_mode) {
  const bool gait_select_mode =
      gamepad.buttons[GLFW_GAMEPAD_BUTTON_Y];

  if (gait_select_mode) {
    if (gamepad_pressed[GLFW_GAMEPAD_BUTTON_DPAD_UP]) {
      *pending_gait_mode =
          (*pending_gait_mode + kNumGaitModes - 1) % kNumGaitModes;
    }
    if (gamepad_pressed[GLFW_GAMEPAD_BUTTON_DPAD_DOWN]) {
      *pending_gait_mode =
          (*pending_gait_mode + 1) % kNumGaitModes;
    }
  }

  {
    ImGui::Begin("Gait");
    const bool was_collapsed = ImGui::IsWindowCollapsed();

    ImGui::SetWindowCollapsed(!gait_select_mode);
    ImGui::SetWindowPos({800, 50}, ImGuiCond_FirstUseEver);

    ImGui::RadioButton("Stop", pending_gait_mode, kStop);
    ImGui::RadioButton("Rest", pending_gait_mode, kRest);
    ImGui::RadioButton("Walk", pending_gait_mode, kWalk);
    ImGui::RadioButton("Jump", pending_gait_mode, kJump);
    ImGui::RadioButton("Zero", pending_gait_mode, kZero);

    ImGui::End();

    if (!gait_select_mode && !was_collapsed) {
      // Update our command.
      *command_mode = [&]() {
        switch (*pending_gait_mode) {
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
}

constexpr int kNumTurretModes = 2;

void DrawTurret(const GLFWgamepadstate& gamepad,
                const std::vector<bool>& gamepad_pressed,
                int* pending_turret_mode,
                TurretControl::Mode* turret_mode) {
  const bool turret_select_mode =
      gamepad.buttons[GLFW_GAMEPAD_BUTTON_X];

  if (turret_select_mode) {
    if (gamepad_pressed[GLFW_GAMEPAD_BUTTON_DPAD_UP]) {
      *pending_turret_mode =
          (*pending_turret_mode + kNumTurretModes - 1) % kNumTurretModes;
    }
    if (gamepad_pressed[GLFW_GAMEPAD_BUTTON_DPAD_DOWN]) {
      *pending_turret_mode =
          (*pending_turret_mode + 1) % kNumTurretModes;
    }
  }

  {
    ImGui::Begin("Turret Cmd");
    const bool was_collapsed = ImGui::IsWindowCollapsed();

    ImGui::SetWindowCollapsed(!turret_select_mode);
    ImGui::SetWindowPos({800, 200}, ImGuiCond_FirstUseEver);

    ImGui::RadioButton("Stop", pending_turret_mode, 0);
    ImGui::RadioButton("Active", pending_turret_mode, 1);

    ImGui::End();

    if (!turret_select_mode && !was_collapsed) {
      *turret_mode = [&]() {
        switch (*pending_turret_mode) {
          case 0: return TurretControl::Mode::kStop;
          case 1: return TurretControl::Mode::kActive;
        }
        mjlib::base::AssertNotReached();
      }();
    }
  }
}

class VideoRender {
 public:
  struct Options {
    double rotate_deg = 0.0;

    Options() {}
  };
  VideoRender(std::string_view filename, const Options& options = Options())
      : file_(
          filename,
          {
            { "input_format", "mjpeg" },
            { "framerate", "30" },
                },
          ffmpeg::File::Flags()
          .set_nonblock(true)
          .set_input_format(ffmpeg::InputFormat("v4l2"))),
        options_(options) {
    program_.use();

    vao_.bind();

    vertices_.bind(GL_ARRAY_BUFFER);

    std::vector<base::Point3D> points = {
      { -1.0, 1.0, 0.0, },
      { -1.0, -1.0, 0.0 },
      { 1.0, -1.0, 0.0 },
      { 1.0, 1.0, 0.0 },
    };

    for (auto& point : points) {
      point = rotate_.Rotate(point);
    }

    const auto& ps = points;
    auto f = [](auto v) { return static_cast<float>(v); };
    const float data[] = {
      // vertex (x, y, z) texture (u, v)
      f(ps[0].x()), f(ps[0].y()), f(ps[0].z()), 0.0f, 0.0f,
      f(ps[1].x()), f(ps[1].y()), f(ps[1].z()), 0.0f, 1.0f,
      f(ps[2].x()), f(ps[2].y()), f(ps[2].z()), 1.0f, 1.0f,
      f(ps[3].x()), f(ps[3].y()), f(ps[3].z()), 1.0f, 0.0f
    };
    vertices_.set_data_array(GL_ARRAY_BUFFER, data, GL_STATIC_DRAW);

    program_.VertexAttribPointer(
        program_.attribute("vertex"),
        3, GL_FLOAT, GL_FALSE, 20, 0);
    program_.VertexAttribPointer(
        program_.attribute("texCoord0"),
        2, GL_FLOAT, GL_FALSE, 20, 12);

    const uint8_t elements[] = {
      0, 1, 2,
      0, 2, 3,
    };
    elements_.set_data_array(
        GL_ELEMENT_ARRAY_BUFFER, elements, GL_STATIC_DRAW);
    vao_.unbind();

    program_.SetUniform(program_.uniform("frameTex"), 0);
    program_.SetUniform(program_.uniform("mvpMatrix"),
                        Ortho(-1, 1, -1, 1, -1, 1));
  }

  void SetViewport(const Eigen::Vector2i& window_size) {
    int x = 0;
    int y = 0;
    int display_w = window_size.x();
    int display_h = window_size.y();

    Eigen::Vector2i codec_size = Rotate(codec_.size());
    // Enforce an aspect ratio.
    const double desired_aspect_ratio =
        static_cast<double>(std::abs(codec_size.x())) /
        static_cast<double>(std::abs(codec_size.y()));
    const double actual_ratio =
        static_cast<double>(display_w) /
        static_cast<double>(display_h);
    if (actual_ratio > desired_aspect_ratio) {
      const int w = display_h * desired_aspect_ratio;
      const int remaining = display_w - w;
      x = remaining / 2;
      display_w = w;
    } else if (actual_ratio < desired_aspect_ratio) {
      const int h = display_w / desired_aspect_ratio;
      const int remaining = display_h - h;
      y = remaining / 2;
      display_h = h;
    }

    glViewport(x, y, display_w, display_h);
  }

  void Update() {
    UpdateVideo();
    Draw();
  }

  void UpdateVideo() {
    auto maybe_pref = file_.Read(&packet_);
    if (!maybe_pref) { return; }

    codec_.SendPacket(*maybe_pref);
    auto maybe_fref = codec_.GetFrame(&frame_);

    if (!maybe_fref) { return; }

    if (!swscale_) {
      swscale_.emplace(codec_, dest_frame_.size(), dest_frame_.format(),
                       ffmpeg::Swscale::kBicubic);
    }
    swscale_->Scale(*maybe_fref, dest_frame_ptr_);

    texture_.Store(dest_frame_ptr_->data[0]);
  }

  void Draw() {
    program_.use();
    vao_.bind();
    texture_.bind();
    glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_BYTE, 0);
    vao_.unbind();
  }

 private:
  Eigen::Vector2i Rotate(const Eigen::Vector2i value) {
    base::Point3D p(value.x(), value.y(), 0.0);
    const auto result = rotate_.Rotate(p);
    return {static_cast<int>(result.x()), static_cast<int>(result.y())};
  }

  static Eigen::Matrix4f Ortho(float left, float right, float bottom, float top,
                               float zNear, float zFar) {
    Eigen::Matrix4f result = Eigen::Matrix4f::Identity();
    result(0, 0) = 2.0f / (right - left);
    result(1, 1) = 2.0f / (top - bottom);
    result(2, 2) = 1.0f / (zFar - zNear);
    result(3, 0) = - (right + left) / (right - left);
    result(3, 1) = - (top + bottom) / (top - bottom);
    result(3, 2) = - zNear / (zFar - zNear);
    return result;
  }

  ffmpeg::File file_;
  const Options options_;
  base::Quaternion rotate_{
    base::Quaternion::FromEuler(0, 0, base::Radians(options_.rotate_deg))};
  ffmpeg::Stream stream_{file_.FindBestStream(ffmpeg::File::kVideo)};
  ffmpeg::Codec codec_{stream_};
  std::optional<ffmpeg::Swscale> swscale_;
  ffmpeg::Packet packet_;
  ffmpeg::Frame frame_;
  ffmpeg::Frame dest_frame_;
  ffmpeg::Frame::Ref dest_frame_ptr_{dest_frame_.Allocate(
        AV_PIX_FMT_RGB24, codec_.size(), 1)};

  static constexpr const char* kVertexShaderSource =
        "#version 150\n"
	"in vec3 vertex;\n"
	"in vec2 texCoord0;\n"
	"uniform mat4 mvpMatrix;\n"
	"out vec2 texCoord;\n"
	"void main() {\n"
	"	gl_Position = mvpMatrix * vec4(vertex, 1.0);\n"
	"	texCoord = texCoord0;\n"
	"}\n";

  static constexpr const char* kFragShaderSource =
        "#version 150\n"
	"uniform sampler2D frameTex;\n"
	"in vec2 texCoord;\n"
	"out vec4 fragColor;\n"
	"void main() {\n"
	"	fragColor = texture(frameTex, texCoord);\n"
	"}\n";

  gl::Shader vertex_shader_{kVertexShaderSource, GL_VERTEX_SHADER};
  gl::Shader fragment_shader_{kFragShaderSource, GL_FRAGMENT_SHADER};
  gl::Program program_{vertex_shader_, fragment_shader_};

  gl::VertexArrayObject vao_;
  gl::VertexBufferObject vertices_;
  gl::VertexBufferObject elements_;
  gl::FlatRgbTexture texture_{codec_.size()};
};

int do_main(int argc, char** argv) {
  std::string video = "/dev/video0";
  bool turret = false;
  double rotate_deg = 180.0;
  double jump_accel_mm_s2 = 2000.0;

  mjlib::io::StreamFactory::Options stream;
  stream.type = mjlib::io::StreamFactory::Type::kSerial;
  stream.serial_port = "/dev/nrfusb";

  clipp::group group = clipp::group(
      (clipp::option("v", "video") & clipp::value("", video)),
      (clipp::option("stuff")),
      (clipp::option("t", "turret").set(turret)),
      (clipp::option("r", "rotate") & clipp::value("", rotate_deg)),
      (clipp::option("jump-accel") & clipp::value("", jump_accel_mm_s2)),
      mjlib::base::ClippArchive("stream.").Accept(&stream).release()
  );

  mjlib::base::ClippParse(argc, argv, group);

  mjlib::io::SharedStream shared_stream;
  std::unique_ptr<SlotCommand> slot_command;

  boost::asio::io_context context;
  boost::asio::executor executor = context.get_executor();
  mjlib::io::StreamFactory stream_factory{executor};
  stream_factory.AsyncCreate(stream, [&](auto ec, auto shared_stream_in) {
      mjlib::base::FailIf(ec);
      shared_stream = shared_stream_in;  // so it sticks around
      slot_command = std::make_unique<SlotCommand>(shared_stream.get());
    });

  gl::Window window(1280, 720, "quad RF command");
  gl::GlImGui imgui(window);

  std::optional<VideoRender> video_render;
  try {
    VideoRender::Options video_options;
    video_options.rotate_deg = rotate_deg;
    video_render.emplace(video, video_options);
  } catch (mjlib::base::system_error& se) {
    if (std::string(se.what()).find("No such file or directory") ==
        std::string::npos) {
      throw;
    }
  }

  QuadrupedCommand::Mode command_mode = QuadrupedCommand::Mode::kStopped;
  int pending_gait_mode = kStop;
  TurretControl::Mode turret_mode = TurretControl::Mode::kStop;
  int pending_turret_mode = 0;

  std::vector<bool> old_gamepad_buttons;
  old_gamepad_buttons.resize(GLFW_GAMEPAD_BUTTON_LAST + 1);

  ExpoMap expo{[]() {
      ExpoMap::Options options;
      options.deadband = 0.15;
      options.slow_range = 0.40;
      options.slow_value = 0.10;
      return options;
    }()};

  ExpoMap turret_walk_expo{[]() {
      ExpoMap::Options options;
      options.deadband = 0.0;
      options.slow_range = 0.30;
      options.slow_value = 0.50;
      return options;
    }()};

  ExpoMap lateral_walk_expo{[]() {
      ExpoMap::Options options;
      options.deadband = 0.15;
      options.slow_range = 0.30;
      options.slow_value = 0.15;
      return options;
    }()};

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

    if (video_render) {
      video_render->SetViewport(window.framebuffer_size());
    }
    glClearColor(0.45f, 0.55f, 0.60f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);

    TRACE_GL_ERROR();

    if (video_render) {
      video_render->Update();
    }

    TRACE_GL_ERROR();

    DrawTelemetry(slot_command.get(), turret);
    DrawGamepad(gamepad);

    DrawGait(gamepad, gamepad_pressed, &pending_gait_mode, &command_mode);
    DrawTurret(gamepad, gamepad_pressed, &pending_turret_mode, &turret_mode);

    base::Point3D v_mm_s_R;
    base::Point3D w_LR;
    base::Euler turret_rate_dps;
    bool turret_track = false;
    if (!turret) {
      v_mm_s_R.x() = -kMaxForwardVelocity_mm_s *
          gamepad.axes[GLFW_GAMEPAD_AXIS_LEFT_Y];
      v_mm_s_R.y() = kMaxLateralVelocity_mm_s *
          lateral_walk_expo(gamepad.axes[GLFW_GAMEPAD_AXIS_LEFT_X]);

      w_LR.z() = kMaxRotation_rad_s * gamepad.axes[GLFW_GAMEPAD_AXIS_RIGHT_X];
    } else {
      const Eigen::Vector2d cmd_turret = Eigen::Vector2d(
          lateral_walk_expo(gamepad.axes[GLFW_GAMEPAD_AXIS_LEFT_X]),
          -gamepad.axes[GLFW_GAMEPAD_AXIS_LEFT_Y]);

      const double turret_rad =
          base::WrapNegPiToPi(
              base::Radians(-slot_command->data().turret.servo_yaw_deg));
      const Sophus::SE2d pose_robot_turret = Sophus::SE2d(turret_rad, {});

      // Rotate this to be relative to the robot instead of the turret.
      const Eigen::Vector2d cmd_robot = pose_robot_turret * cmd_turret;

      // Now we use some heuristics to make things drive better.  If
      // we are trying to mostly turn, then we scale back the forward
      // velocity so that we get our turn done before moving.
      const double turn_ratio = cmd_robot.x() / cmd_robot.norm();
      const double kTurnThreshold = 0.5;
      const double forward_scale =
          mjlib::base::Limit(
              base::Interpolate(
                  1.0, 0.0,
                  (turn_ratio - kTurnThreshold) /
                  (1.0 - kTurnThreshold)),
              0.0, 1.0);

      v_mm_s_R.x() = kMaxForwardVelocity_mm_s * forward_scale * cmd_robot.y();
      // We pick the sign of our rotation to get closest to a forward
      // or backward configuration as possible.
      const double rotation_sign = 1.0;
          // (std::abs(turret_rad) > 0.5 * M_PI) ?
          // -1.0 : 1.0;
      w_LR.z() =
          std::copysign(1.0, v_mm_s_R.x()) * rotation_sign *
          kMaxRotation_rad_s * turret_walk_expo(cmd_robot.x());

      // Finally, do the turret rates.
      turret_rate_dps.pitch = -kMaxTurretPitch_dps *
          expo(gamepad.axes[GLFW_GAMEPAD_AXIS_RIGHT_Y]);
      turret_rate_dps.yaw = kMaxTurretYaw_dps *
          expo(gamepad.axes[GLFW_GAMEPAD_AXIS_RIGHT_X]);
      turret_track = gamepad.buttons[GLFW_GAMEPAD_BUTTON_LEFT_BUMPER] != 0;
    }

    Sophus::SE3d pose_mm_RB;

    const QuadrupedCommand::Mode actual_command_mode = [&]() {
      const bool movement_commanded = (
          v_mm_s_R.norm() > kMovementEpsilon_mm_s ||
          w_LR.norm() > kMovementEpsilon_rad_s);
      if (!movement_commanded &&
          (command_mode == QuadrupedCommand::Mode::kWalk ||
           command_mode == QuadrupedCommand::Mode::kJump)) {
        return QuadrupedCommand::Mode::kRest;
      }
      return command_mode;
    }();

    {
      ImGui::Begin("Command");
      ImGui::SetWindowPos({50, 400}, ImGuiCond_FirstUseEver);
      ImGui::Text("Mode  : %14s", get(QuadrupedCommand::ModeMapper(), command_mode));
      ImGui::Text("Actual: %14s",
                  get(QuadrupedCommand::ModeMapper(), actual_command_mode));
      ImGui::Text("cmd: (%4.0f, %4.0f, %6.3f)",
                  v_mm_s_R.x(),
                  v_mm_s_R.y(),
                  w_LR.z());
      ImGui::Text("pose x/y: (%3.0f, %3.0f)",
                  pose_mm_RB.translation().x(),
                  pose_mm_RB.translation().y());
      ImGui::End();
    }

    if (slot_command) {
      slot_command->Command(
          actual_command_mode, pose_mm_RB, v_mm_s_R, w_LR,
          jump_accel_mm_s2,
          turret_mode, turret_rate_dps, turret_track);
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
