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

#include "ffmpeg/file.h"

#include <fstream>

#include <boost/test/auto_unit_test.hpp>

#include "tools/cpp/runfiles/runfiles.h"

#include "ffmpeg/codec.h"
#include "ffmpeg/swscale.h"

using namespace mjmech::ffmpeg;
using bazel::tools::cpp::runfiles::Runfiles;

BOOST_AUTO_TEST_CASE(FileTest) {
  std::string error;
  std::unique_ptr<Runfiles> runfiles{Runfiles::CreateForTest(&error)};

  const std::string path = "ffmpeg/test/data/sample_log.mp4";

  const std::string qualified_path = [&]() {
    if (!!runfiles) {
      return runfiles->Rlocation("com_github_mjbots_mech/" + path);
    }
    return path;
  }();

  File dut{qualified_path};

  auto stream = dut.FindBestStream(File::kVideo);
  BOOST_TEST(stream.codec_parameters()->width == 1920);

  auto codec = Codec(stream);
  std::optional<Swscale> swscale;

  Packet packet;
  Frame frame;
  Frame dest_frame;
  auto dest_ptr = dest_frame.Allocate(AV_PIX_FMT_RGB24, {640, 480}, 1);
  {
    auto maybe_pref = dut.Read(&packet);
    if (!!maybe_pref) {
      codec.SendPacket(*maybe_pref);
      auto maybe_fref = codec.GetFrame(&frame);

      if (!!maybe_fref) {
        if (!swscale) {
          swscale.emplace(codec, dest_frame.size(),
                          dest_frame.format(), Swscale::kBicubic);
        }
        swscale->Scale(*maybe_fref, dest_ptr);
      }
    }
  }
}
