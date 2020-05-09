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


// TODO:

// * Dragging the time bar is totally unresponsive
// * Plots
// * Video
// * 3D mech

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/lexical_cast.hpp>

#include <fmt/format.h>

#include "gl/gl_imgui.h"

#include "mjlib/base/buffer_stream.h"
#include "mjlib/base/clipp.h"
#include "mjlib/base/time_conversions.h"
#include "mjlib/telemetry/file_reader.h"

using mjlib::telemetry::FileReader;
using Element = mjlib::telemetry::BinarySchemaParser::Element;
using Format = mjlib::telemetry::Format;
using FileReader = mjlib::telemetry::FileReader;

namespace mjmech {
namespace util {
namespace {

class TreeView {
 public:
  TreeView(FileReader* reader) : reader_(reader) {
  }

  void Update(boost::posix_time::ptime timestamp) {
    if (last_timestamp_.is_not_a_date_time() ||
        timestamp < last_timestamp_ ||
        (timestamp - last_timestamp_) > boost::posix_time::seconds(1)) {
      Seek(timestamp);
    } else {
      Step(timestamp);
    }
    last_timestamp_ = timestamp;

    Render();
  }

  void Render() {
    ImGui::SetNextWindowPos(ImVec2(0, 0), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(400, 620), ImGuiCond_FirstUseEver);
    gl::ImGuiWindow file_window("Data");

    std::vector<const FileReader::Record*> records;
    for (const auto& pair : data_) { records.push_back(pair.first); }
    std::sort(records.begin(), records.end(), [](auto lhs, auto rhs) {
        return lhs->name < rhs->name;
      });

    for (auto record : records) {
      const auto data = data_.at(record);
      if (data.empty()) {
        // No data... for now we won't even let you expand it.
        if (ImGui::TreeNodeEx(record, ImGuiTreeNodeFlags_Leaf,
                              "%s", record->name.c_str())) {
          ImGui::TreePop();
        }
      } else {
        ImGui::Columns(2, nullptr, true);
        // ImGui::SetColumnWidth(0, ImGui::GetWindowContentRegionWidth() - 150);
        mjlib::base::BufferReadStream stream{data};
        VisitElement(record->schema->root(), stream);
        ImGui::Columns(1);
      }
    }
  }

  void VisitElement(const Element* element, mjlib::base::ReadStream& stream,
                    const char* name_override = nullptr) {
    using FT = Format::Type;

    // Union types we just forward through to the appropriate typed
    // child.
    if (element->type == FT::kUnion) {
      const auto index = element->ReadUnionIndex(stream);
      VisitElement(element->children[index], stream);
      return;
    }

    const bool children = !element->children.empty() || !element->fields.empty();
    int flags = 0;
    if (!children) {
      flags |= ImGuiTreeNodeFlags_Leaf;
    }
    const bool expanded = ImGui::TreeNodeEx(
        element, flags, "%s",
        name_override ? name_override : element->name.c_str());
    ImGui::NextColumn();

    // Read the scalar data to display.
    const auto value = [&]() -> std::string {
      switch (element->type) {
        case FT::kBoolean: {
          return element->ReadBoolean(stream) ? "true" : "false";
        }
        case FT::kFixedInt:
        case FT::kVarint: {
          return fmt::format("{}", element->ReadIntLike(stream));
        }
        case FT::kFixedUInt:
        case FT::kVaruint: {
          return fmt::format("{}", element->ReadUIntLike(stream));
        }
        case FT::kFloat32:
        case FT::kFloat64: {
          return fmt::format("{}", element->ReadFloatLike(stream));
        }
        case FT::kBytes: {
          return fmt::format("b'{}'", element->ReadString(stream));
        }
        case FT::kString: {
          return element->ReadString(stream);
        }
        case FT::kTimestamp:
        case FT::kDuration: {
          // TODO: Optionally (or always) display calendar time.
          return fmt::format(
              "{:.3f}", element->ReadIntLike(stream) / 1000000.0);
        }
        case FT::kEnum: {
          const auto code = element->children.front()->ReadUIntLike(stream);
          const auto it = element->enum_items.find(code);
          if (it != element->enum_items.end()) { return it->second; }
          return fmt::format("{}", code);
        }
        case FT::kFinal:
        case FT::kNull:
        case FT::kObject:
        case FT::kArray:
        case FT::kMap:
        case FT::kUnion: {
          break;
        }
      }
      return "";
    }();

    ImGui::Text("%s", value.c_str());
    ImGui::NextColumn();

    if (expanded) {
      switch (element->type) {
        case FT::kObject: {
          for (const auto& field : element->fields) {
            VisitElement(field.element, stream);
          }
          break;
        }
        case FT::kArray: {
          const auto nelements = element->ReadArraySize(stream);
          for (uint64_t i = 0; i < nelements; i++) {
            ImGui::PushID(i);
            VisitElement(element->children.front(), stream,
                         fmt::format("{}", i).c_str());
            ImGui::PopID();
          }
          break;
        }
        case FT::kMap: {
          // TODO
          element->Ignore(stream);
          break;
        }
        case FT::kUnion: {
          mjlib::base::AssertNotReached();
        }
        default: {
          break;
        }
      }
      ImGui::TreePop();
    } else {
      // We still need to skip any children to keep our stream
      // consistent.
      switch (element->type) {
        case FT::kObject:
        case FT::kArray:
        case FT::kMap: {
          element->Ignore(stream);
          break;
        }
        case FT::kUnion: {
          mjlib::base::AssertNotReached();
        }
        default: {
          break;
        }
      }
    }
  }

  void Seek(boost::posix_time::ptime timestamp) {
    data_ = {};
    for (auto record : reader_->records()) { data_[record] = ""; }
    last_index_ = {};

    const auto records = reader_->Seek(timestamp);
    for (const auto& pair : records) {
      const auto item = (*reader_->items([&]() {
          FileReader::ItemsOptions options;
          options.start = pair.second;
          return options;
        }()).begin());
      if (item.index > last_index_) { last_index_ = item.index; }
      data_.insert(std::make_pair(pair.first, item.data));
    }
  }

  void Step(boost::posix_time::ptime timestamp) {
    // We are some small distance into the future from our last
    // operation.  Step until we get there.
    auto items = reader_->items([&]() {
        FileReader::ItemsOptions options;
        options.start = last_index_;
        return options;
      }());
    for (const auto& item : items) {
      if (item.timestamp > timestamp) {
        // We're done!
        break;
      }
      if (item.index > last_index_) { last_index_ = item.index; }
      data_[item.record] = item.data;
    }
  }


  FileReader* const reader_;
  boost::posix_time::ptime last_timestamp_;
  FileReader::Index last_index_ = {};
  std::map<const FileReader::Record*, std::string> data_;
};

class Timeline {
 public:
  Timeline(FileReader* reader) {
    {
      const auto final_item = reader->final_item();
      auto items = reader->items([&]() {
          FileReader::ItemsOptions options;
          options.start = final_item;
          return options;
        }());
      end_ = (*items.begin()).timestamp;
    }
    {
      auto items = reader->items();
      start_ = (*items.begin()).timestamp;
    }
    float_range_ = mjlib::base::ConvertDurationToSeconds(end_ - start_);
  }

  void Update() {
    ImGui::SetNextWindowSize(ImVec2(800, 100), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowPos(ImVec2(0, 620), ImGuiCond_FirstUseEver);
    gl::ImGuiWindow playback("Playback");
    ImGui::SetNextItemWidth(-1);
    ImGui::SliderFloat("Time", &float_time_, 0, float_range_);
    if (ImGui::RadioButton("RR", mode_ == kFastRewind)) { mode_ = kFastRewind; }
    ImGui::SameLine();
    if (ImGui::RadioButton("StepR", false)) {
      float_time_ -= step_;
      mode_ = kStop;
    }
    ImGui::SameLine();
    if (ImGui::RadioButton("Rewind", mode_ == kRewind)) { mode_ = kRewind; }
    ImGui::SameLine();
    if (ImGui::RadioButton("Stop", mode_ == kStop)) { mode_ = kStop; }
    ImGui::SameLine();
    if (ImGui::RadioButton("Play", mode_ == kPlay)) { mode_ = kPlay; }
    ImGui::SameLine();
    if (ImGui::RadioButton("StepF", false)) {
      float_time_ += step_;
      mode_ = kStop;
    }
    ImGui::SameLine();
    if (ImGui::RadioButton("FF", mode_ == kFastForward)) { mode_ = kFastForward; }
    ImGui::SetNextItemWidth(150);
    ImGui::InputFloat("Step", &step_, 0.001, 0.01);
    ImGui::SameLine(0, 20.0);
    ImGui::SetNextItemWidth(150);
    ImGui::InputFloat("Speed", &fast_speed_, 0.1, 1.0);
    ImGui::SameLine(0, 30.0);
    ImGui::Text("Clock: %s", boost::lexical_cast<std::string>(current()).c_str());

    const auto now = boost::posix_time::microsec_clock::universal_time();
    const double dt_s = last_update_.is_not_a_date_time() ? 0.0 :
        mjlib::base::ConvertDurationToSeconds(now - last_update_);
    last_update_ = now;

    switch (mode_) {
      case kFastRewind: {
        float_time_ -= fast_speed_ * dt_s;
        break;
      }
      case kRewind: {
        float_time_ -= dt_s;
        break;
      }
      case kStop: {
        break;
      }
      case kPlay: {
        float_time_ += dt_s;
        break;
      }
      case kFastForward: {
        float_time_ += fast_speed_ * dt_s;
        break;
      }
    }
  }

  boost::posix_time::ptime current() const {
    return start_ + mjlib::base::ConvertSecondsToDuration(float_time_);
  }

 private:
  enum Mode {
    kFastRewind,
    kRewind,
    kStop,
    kPlay,
    kFastForward,
  };

  Mode mode_ = kStop;

  boost::posix_time::ptime start_;
  boost::posix_time::ptime end_;

  float float_range_ = 0.0;
  float float_time_ = 0.0;
  float step_ = 0.1;
  float fast_speed_ = 5.0;

  boost::posix_time::ptime last_update_;
};
}

int do_main(int argc, char** argv) {
  std::string log_filename;

  auto group = clipp::group(
      clipp::value("log file", log_filename)
                            );

  mjlib::base::ClippParse(argc, argv, group);

  mjlib::telemetry::FileReader file_reader(log_filename);
  const auto records = file_reader.records();

  gl::Window window(1280, 720, "tplot2");
  gl::GlImGui imgui(window);

  ImGui::GetIO().ConfigFlags |=
      ImGuiConfigFlags_DockingEnable;

  Timeline timeline{&file_reader};
  TreeView tree_view{&file_reader};

  while (!window.should_close()) {
    window.PollEvents();
    imgui.NewFrame();

    glClearColor(0.45f, 0.55f, 0.60f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);

    timeline.Update();
    tree_view.Update(timeline.current());

    ImGui::ShowDemoWindow();

    imgui.Render();
    window.SwapBuffers();
  }

  return 0;
}
}
}

int main(int argc, char** argv) {
  return mjmech::util::do_main(argc, argv);
}
