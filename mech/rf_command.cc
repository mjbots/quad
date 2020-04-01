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
#include "mjlib/io/stream_factory.h"
#include "mjlib/telemetry/format.h"

#include "base/point3d.h"
#include "base/saturate.h"
#include "base/sophus.h"

#include "gl/gl_imgui.h"
#include "gl/window.h"

#include "mech/nrfusb_client.h"
#include "mech/quadruped_command.h"

namespace mjmech {
namespace mech {

namespace {
class SlotCommand {
 public:
  SlotCommand(mjlib::io::AsyncStream* stream)
      : nrfusb_(stream) {}

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

 private:
  NrfusbClient nrfusb_;
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

  bool show_demo_window = true;

  while (!window.should_close()) {
    context.poll(); context.reset();
    window.PollEvents();

    imgui.NewFrame();

    glClearColor(0.45f, 0.55f, 0.60f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);

    TRACE_GL_ERROR();

    ImGui::ShowDemoWindow(&show_demo_window);

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
