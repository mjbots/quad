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
// * 3D mech
//  * need view reset
//  * need toggles to render different things like actual/commanded
//  * render lines and arrows in 3d
//  * render velocities and forces
//  * render a grid with units in 3d
//  * it would be nice to start with legs down
//  * should render an arrow on the top and sides of the mech so I
//    know which way is forward
//  * it is mirrored left/right
//  * light should be on the actual top (negative Z), or behind the
//    camera
//  * optionally rotate by roll/pitch, double optionally by yaw
//  * plot time trajectories over a time window, or perhaps just
//    resettable trailers

// * Plots
//  * I get crazy artifacts when non-first plots are entirely off screen
//  * save window size in imgui.ini
// * Save/restore plot configuration
// * Derived/scripted fields

#include <string>
#include <variant>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/lexical_cast.hpp>

#include <fmt/format.h>

#include <implot.h>

#include "base/aspect_ratio.h"

#include "ffmpeg/codec.h"
#include "ffmpeg/file.h"
#include "ffmpeg/ffmpeg.h"
#include "ffmpeg/frame.h"
#include "ffmpeg/packet.h"
#include "ffmpeg/swscale.h"

#include "gl/flat_rgb_texture.h"
#include "gl/framebuffer.h"
#include "gl/gl_imgui.h"
#include "gl/perspective_camera.h"
#include "gl/program.h"
#include "gl/renderbuffer.h"
#include "gl/shader.h"
#include "gl/trackball.h"
#include "gl/vertex_array_object.h"
#include "gl/vertex_buffer_object.h"

#include "mjlib/base/buffer_stream.h"
#include "mjlib/base/clipp.h"
#include "mjlib/base/time_conversions.h"
#include "mjlib/base/tokenizer.h"
#include "mjlib/telemetry/file_reader.h"
#include "mjlib/telemetry/mapped_binary_reader.h"

#include "mech/quadruped_control.h"

using mjlib::telemetry::FileReader;
using Element = mjlib::telemetry::BinarySchemaParser::Element;
using Format = mjlib::telemetry::Format;
using FileReader = mjlib::telemetry::FileReader;
using FT = Format::Type;

namespace mjmech {
namespace util {
namespace {

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

class TreeView {
 public:
  TreeView(FileReader* reader) : reader_(reader) {
  }

  std::optional<std::string> data(const std::string& name) {
    for (const auto& pair : data_) {
      if (pair.first->name == name) { return pair.second; }
    }
    return {};
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

  struct Parent {
    const Parent* parent = nullptr;
    const Element* element = nullptr;
    int64_t array_index = -1;

    std::string RenderToken(const Element* child) const {
      const std::string this_token =
          array_index >= 0 ? fmt::format("{}", array_index) : child->name;
      return (parent ? (parent->RenderToken(element) + ".") : "") + this_token;
    }
  };

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
        mjlib::base::BufferReadStream stream{data};
        Parent parent;
        VisitElement(record->schema->root(), stream, &parent);
        ImGui::Columns(1);
      }
    }
  }

  void VisitElement(const Element* element,
                    mjlib::base::ReadStream& stream,
                    const Parent* parent,
                    const char* name_override = nullptr) {
    // Union types we just forward through to the appropriate typed
    // child.
    if (element->type == FT::kUnion) {
      const auto index = element->ReadUnionIndex(stream);
      VisitElement(element->children[index], stream, parent);
      return;
    }

    const bool children =
        (!element->children.empty() || !element->fields.empty()) &&
        (element->type != FT::kEnum);

    int flags = 0;
    if (!children) {
      flags |= ImGuiTreeNodeFlags_Leaf;
    }
    const bool expanded = ImGui::TreeNodeEx(
        element, flags, "%s",
        name_override ? name_override : element->name.c_str());

    if (!children && ImGui::BeginDragDropSource()) {
      const std::string token = parent->RenderToken(element);
      ImGui::SetDragDropPayload("DND_TLOG", token.data(), token.size());
      ImGui::TextUnformatted(token.c_str());
      ImGui::EndDragDropSource();
    }

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
        case FT::kFixedArray:
        case FT::kMap:
        case FT::kUnion: {
          break;
        }
      }
      return "";
    }();

    ImGui::Text("%s", value.c_str());
    ImGui::NextColumn();

    auto do_array = [&](uint64_t nelements) {
      for (uint64_t i = 0; i < nelements; i++) {
        ImGui::PushID(i);
        Parent new_parent;
        new_parent.parent = parent;
        new_parent.element = element;
        new_parent.array_index = i;
        VisitElement(element->children.front(), stream, &new_parent,
                     fmt::format("{}", i).c_str());
        ImGui::PopID();
      }
    };

    if (expanded) {
      switch (element->type) {
        case FT::kObject: {
          for (const auto& field : element->fields) {
            Parent new_parent;
            new_parent.parent = parent;
            new_parent.element = element;
            VisitElement(field.element, stream, &new_parent);
          }
          break;
        }
        case FT::kArray: {
          const auto nelements = element->ReadArraySize(stream);
          do_array(nelements);
          break;
        }
        case FT::kFixedArray: {
          do_array(element->array_size);
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
        case FT::kFixedArray:
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
      data_[pair.first] = item.data;
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

class ValueRetrieve {
 public:
  ValueRetrieve(const Element* root,
                boost::posix_time::ptime log_start,
                const std::string& name)
      : log_start_(log_start) {
    if (name.empty()) {
      is_timestamp_ = true;
      valid_ = true;
      return;
    }

    mjlib::base::Tokenizer tokenizer(name, ".");
    auto* element = root;
    auto next = tokenizer.next();
    valid_ = [&]() {
      while (true) {
        switch (element->type) {
          case FT::kFinal:
          case FT::kNull:
          case FT::kBoolean:
          case FT::kFixedInt:
          case FT::kFixedUInt:
          case FT::kVarint:
          case FT::kVaruint:
          case FT::kFloat32:
          case FT::kFloat64:
          case FT::kBytes:
          case FT::kString:
          case FT::kEnum:
          case FT::kTimestamp:
          case FT::kDuration: {
            // This won't be used.
            chain_.push_back(static_cast<const Element*>(nullptr));

            // We're done.
            return true;
          }
          case FT::kObject: {
            // Loop to find the next name.
            const bool success = [&]() {
              for (const auto& field : element->fields) {
                if (field.name == next) {
                  // This is it.
                  chain_.push_back(field.element);
                  element = field.element;
                  return true;
                }
              }
              // Wow, we couldn't find the name.  That's a problem.
              return false;
            }();
            if (!success) { return false; }
            break;
          }
          case FT::kArray:
          case FT::kFixedArray: {
            chain_.push_back(static_cast<uint64_t>(
                                 std::stoull(std::string(next))));
            element = element->children.front();
            break;
          }
          case FT::kMap: {
            mjlib::base::AssertNotReached();
          }
          case FT::kUnion: {
            // It is unclear what to do here in the general case.
            // We'll hard-code for the optional case where the first
            // element is null.
            MJ_ASSERT(element->children.size() == 2 &&
                      element->children.front()->type == FT::kNull);
            element = element->children[1];
            // We don't want to consume any of our text string here.
            continue;
          }
        }
        next = tokenizer.next();
      }
    }();
  }

  bool valid() const {
    return valid_;
  }

  float operator()(const FileReader::Item& item) const {
    if (is_timestamp_) {
      return mjlib::base::ConvertDurationToSeconds(item.timestamp - log_start_);
    }

    mjlib::base::BufferReadStream stream{item.data};

    const Element* element = item.record->schema->root();
    auto it = chain_.begin();
    while (it != chain_.end()) {
      const auto& link = *it;
      switch (element->type) {
        case FT::kFinal:
        case FT::kNull: {
          return std::numeric_limits<float>::quiet_NaN();
        }
        case FT::kBoolean: {
          return element->ReadBoolean(stream) ? 1.0f : 0.0f;
        }
        case FT::kFixedInt:
        case FT::kVarint: {
          return element->ReadIntLike(stream);
        }
        case FT::kFixedUInt:
        case FT::kVaruint: {
          return element->ReadUIntLike(stream);
        }
        case FT::kFloat32:
        case FT::kFloat64: {
          return element->ReadFloatLike(stream);
        }
        case FT::kBytes:
        case FT::kString: {
          return 0.0f;  // We can't do this yet.
        }
        case FT::kDuration: {
          return element->ReadIntLike(stream) / 1000000.0;
        }
        case FT::kTimestamp: {
          return mjlib::base::ConvertDurationToSeconds(
              mjlib::base::ConvertEpochMicrosecondsToPtime(
                  element->ReadIntLike(stream)) - log_start_);
        }
        case FT::kEnum: {
          return element->children.front()->ReadUIntLike(stream);
        }
        case FT::kUnion: {
          const auto union_index = element->ReadUnionIndex(stream);
          element = element->children[union_index];
          continue;
        }
        case FT::kObject: {
          const auto* desired_child = std::get<const Element*>(link);
          for (const auto& field : element->fields) {
            if (field.element == desired_child) {
              element = field.element;
              break;
            }
            field.element->Ignore(stream);
          }
          break;
        }
        case FT::kArray:
        case FT::kFixedArray: {
          const uint64_t size =
              (element->type == FT::kArray) ?
              element->ReadArraySize(stream) : element->array_size;
          const uint64_t array_index = std::get<uint64_t>(link);

          if (array_index >= size) {
            // No need to even try.
            return 0.0f;
          }

          const auto* child = element->children.front();
          if (child->maybe_fixed_size >= 0) {
            stream.ignore(array_index * child->maybe_fixed_size);
          } else {
            for (uint64_t i = 0; i < array_index; i++) {
              element->children.front()->Ignore(stream);
            }
          }

          element = element->children.front();
          break;
        }
        case FT::kMap: {
          // TODO
          return 0.0f;
        }
      }
      ++it;
    }
    mjlib::base::AssertNotReached();
  }

 private:
  // Either an element to move down, or an array index.
  using Link = std::variant<const Element*, uint64_t>;
  using Chain = std::vector<Link>;

  boost::posix_time::ptime log_start_;
  bool valid_ = false;
  bool is_timestamp_ = false;
  Chain chain_;
};

class PlotRetrieve {
 public:
  PlotRetrieve(FileReader* reader, boost::posix_time::ptime log_start,
               const std::string& x_token, const std::string& y_token)
      : root_(FindRoot(reader, x_token, y_token)),
        log_start_(log_start) {
  }

  bool valid() const {
    return root_.valid && x_.valid() && y_.valid();
  }

  float x(const FileReader::Item& item) const {
    return x_(item);
  }

  float y(const FileReader::Item& item) const {
    return y_(item);
  }

  FileReader::ItemsOptions items() const {
    FileReader::ItemsOptions result;
    result.records.push_back(root_.record);
    return result;
  }

 private:
  struct Root {
    const Element* root;
    std::string record;
    std::string x_name;
    std::string y_name;
    bool valid = false;
  } root_;

  static Root FindRoot(
      FileReader* reader,
      const std::string& x_token, const std::string y_token) {
    // If a token is empty, that means use the timestamp field.  If
    // neither are empty, then we need to make sure they refer to the
    // same channel.
    mjlib::base::Tokenizer x_tokenizer(x_token, ".");
    mjlib::base::Tokenizer y_tokenizer(y_token, ".");

    Root result;
    if (!x_token.empty() && !y_token.empty()) {
      const auto x_record = x_tokenizer.next();
      const auto y_record = y_tokenizer.next();
      if (x_record != y_record) {
        // Can't do this.
        return {};
      }
      result.record = x_record;
    } else if (x_token.empty() && y_token.empty()) {
      // Nothing?
      return {};
    } else if (!x_token.empty()) {
      result.record = x_tokenizer.next();
    } else {
      result.record = y_tokenizer.next();
    }

    result.root = reader->record(result.record)->schema->root();
    result.x_name = x_tokenizer.remaining();
    result.y_name = y_tokenizer.remaining();
    result.valid = true;
    return result;
  }

  boost::posix_time::ptime log_start_;

  ValueRetrieve x_{root_.root, log_start_, root_.x_name};
  ValueRetrieve y_{root_.root, log_start_, root_.y_name};
};

class PlotView {
 public:
  PlotView(FileReader* reader, boost::posix_time::ptime log_start)
      : reader_(reader),
        log_start_(log_start) {
  }

  void Update(boost::posix_time::ptime timestamp) {
    ImGui::SetNextWindowPos(ImVec2(400, 0), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(800, 620), ImGuiCond_FirstUseEver);
    gl::ImGuiWindow file_window("Plot");

    if (fit_plot_) {
      const auto& p = **fit_plot_;
      float xmin = std::numeric_limits<float>::infinity();
      float xmax = -std::numeric_limits<float>::infinity();
      for (const auto& plot : plots_) {
        xmin = std::min(xmin, plot.min_x);
        xmax = std::max(xmax, plot.max_x);
      }
      ImGui::SetNextPlotLimitsX(xmin, xmax, ImGuiCond_Always);
      ImGui::SetNextPlotLimitsY(p.min_y, p.max_y, ImGuiCond_Always, p.axis);
      fit_plot_ = {};
    }
    const int extra_flags = [&]() {
      int result = 0;
      for (const auto& plot : plots_) {
        if (plot.axis == 1) {
          result |= ImPlotFlags_YAxis2;
        } else if (plot.axis == 2) {
          result |= ImPlotFlags_YAxis3;
        }
      }
      return result;
    }();
    if (ImGui::BeginPlot("Plot", "time", nullptr, ImVec2(-1, -25),
                         ImPlotFlags_Default | extra_flags)) {
      for (const auto& plot : plots_) {
        ImGui::SetPlotYAxis(plot.axis);
        for (const auto& pair : plot.float_styles) {
          ImGui::PushPlotStyleVar(pair.first, pair.second);
        }
        for (const auto& pair : plot.int_styles) {
          ImGui::PushPlotStyleVar(pair.first, pair.second);
        }
        ImGui::Plot(plot.legend.c_str(), plot.xvals.data(), plot.yvals.data(),
                    plot.xvals.size());
        ImGui::PopPlotStyleVar(plot.float_styles.size() + plot.int_styles.size());

        const auto it = std::lower_bound(
            plot.timestamps.begin(), plot.timestamps.end(), timestamp);
        if (it != plot.timestamps.end()) {
          const auto index = it - plot.timestamps.begin();
          ImGui::PushPlotStyleVar(ImPlotStyleVar_Marker, ImMarker_Diamond);
          ImGui::Plot((plot.legend + "_mrk").c_str(),
                      plot.xvals.data() + index, plot.yvals.data() + index,
                      1);
          ImGui::PopPlotStyleVar();
        }
      }

      ImGui::EndPlot();
    }

    if (ImGui::BeginDragDropTarget()) {
      const auto* payload = ImGui::AcceptDragDropPayload("DND_TLOG");
      if (payload) {
        std::string token(static_cast<char*>(payload->Data), payload->DataSize);
        AddPlot("", token);
      }
      ImGui::EndDragDropTarget();
    }

    ImGui::PushItemWidth(60);
    ImGui::Combo("Axis", &current_axis_, kAxisNames, IM_ARRAYSIZE(kAxisNames));
    ImGui::PopItemWidth();
    ImGui::SameLine(0, 10.0);
    if (ImGui::Button("Properties")) {
      ImGui::OpenPopup("Plot Properties");
    }
    ImGui::SameLine(0, 20.0);
    if (ImGui::Button("Remove")) {
      plots_.erase(plots_.begin() + current_plot_index_);
      if (current_plot_index_ > 0 && current_plot_index_ >= plots_.size()) {
        current_plot_index_--;
      }
    }
    ImGui::SameLine(0, 10);
    if (ImGui::BeginCombo("Plots", current_plot_name().c_str())) {
      for (size_t i = 0; i < plots_.size(); i++) {
        if (ImGui::Selectable(
                plots_[i].legend.c_str(), i == current_plot_index_)) {
          current_plot_index_ = i;
        }
      }
      ImGui::EndCombo();
    }

    if (ImGui::BeginPopup("Plot Properties")) {
      if (current_plot_index_ < plots_.size()) {
        auto& plot = plots_[current_plot_index_];

        ImGui::Text("%s", current_plot_name().c_str());
        float step1 = 1.0f;
        ImGui::InputScalar("Width", ImGuiDataType_Float, &plot.float_styles[ImPlotStyleVar_LineWeight], &step1);
        ImGui::InputScalar("Marker Size", ImGuiDataType_Float, &plot.float_styles[ImPlotStyleVar_MarkerSize], &step1);
        constexpr const char* marker_types[] = {
            "none",
            "circle",
            "square",
            "diamond",
            "up",
            "down",
            "left",
            "right",
            "cross",
            "plus",
            "asterisk",
        };

        ImGui::Combo("Marker", &plot.marker_type, marker_types, IM_ARRAYSIZE(marker_types));
        plot.int_styles[ImPlotStyleVar_Marker] = 1 << plot.marker_type;
        ImGui::EndPopup();
      }
    }
  }

 private:
  std::string current_plot_name() const {
    if (current_plot_index_ >= plots_.size()) {
      return "";
    }
    return plots_[current_plot_index_].legend;
  }

  std::string MakeLegend(const std::string& x, const std::string& y) {
    if (!x.empty() && !y.empty()) {
      return fmt::format("{} vs {}", y, x);
    } else if (x.empty()) {
      return y;
    }
    return fmt::format("time vs {}", x);
  }

  void AddPlot(const std::string& x_token, const std::string& y_token) {
    PlotRetrieve getter(reader_, log_start_, x_token, y_token);
    if (!getter.valid()) {
      return;
    }

    plots_.push_back({});
    auto& plot = plots_.back();
    plot.x_token = x_token;
    plot.y_token = y_token;

    plot.legend = MakeLegend(x_token, y_token);

    for (auto item : reader_->items(getter.items())) {
      plot.timestamps.push_back(item.timestamp);
      plot.xvals.push_back(getter.x(item));
      plot.yvals.push_back(getter.y(item));
    }

    if (plot.xvals.empty()) {
      plots_.pop_back();
      return;
    }

    plot.min_x = *std::min_element(plot.xvals.begin(), plot.xvals.end());
    plot.max_x = *std::max_element(plot.xvals.begin(), plot.xvals.end());
    plot.min_y = *std::min_element(plot.yvals.begin(), plot.yvals.end());
    plot.max_y = *std::max_element(plot.yvals.begin(), plot.yvals.end());
    if (plot.max_y <= plot.min_y) {
      plot.max_y = plot.min_y + 1.0f;
    }
    if (plot.max_x <= plot.min_x) {
      plot.max_x = plot.max_x + 1.0f;
    }

    plot.axis = current_axis_;

    // If this is the only plot on this axis, then re-fit things.
    if (1 == std::count_if(
            plots_.begin(), plots_.end(),
            [&](const auto& plt) { return plt.axis == current_axis_; })) {
      fit_plot_ = &plot;
    }
  }

  FileReader* const reader_;
  boost::posix_time::ptime log_start_;

  struct Plot {
    std::string x_token;
    std::string y_token;

    std::string legend;

    std::vector<boost::posix_time::ptime> timestamps;
    std::vector<float> xvals;
    std::vector<float> yvals;

    float min_x = {};
    float max_x = {};
    float min_y = {};
    float max_y = {};

    std::map<int, float> float_styles {
      {ImPlotStyleVar_LineWeight, 1.0f },
      {ImPlotStyleVar_MarkerSize, 5.0f },
    };
    int marker_type = 0;
    std::map<int, int> int_styles {
      {ImPlotStyleVar_Marker, ImMarker_None},
    };

    int axis = 0;
  };

  static inline constexpr const char * kAxisNames[] = {
    "Left",
    "Right",
    "Aux",
  };

  std::vector<Plot> plots_;
  std::optional<Plot*> fit_plot_;
  size_t current_plot_index_ = 0;
  int current_axis_ = 0;
};

class Video {
 public:
  Video(boost::posix_time::ptime log_start, std::string_view filename,
        double time_offset_s)
      : log_start_(log_start),
        time_offset_s_(time_offset_s),
        file_(filename),
        stream_(file_.FindBestStream(ffmpeg::File::kVideo)),
        codec_(stream_) {
    // Read until we get the first frame, so we know what the first
    // timestamp is.
    while (true) {
      auto maybe_packet_ref = file_.Read(&packet_);
      if (!maybe_packet_ref) {
        mjlib::base::system_error::einval(
            "Could not find frames in video");
      }
      if ((*maybe_packet_ref)->stream_index !=
          stream_.av_stream()->index) { continue; }

      codec_.SendPacket(*maybe_packet_ref);

      auto maybe_frame_ref = codec_.GetFrame(&frame_);
      if (maybe_frame_ref) {
        start_pts_ = (*maybe_frame_ref)->pts;
        break;
      }
    }
  }

  void Update(boost::posix_time::ptime timestamp) {
    gl::ImGuiWindow video("Video");

    if (video) {
      if (last_timestamp_.is_not_a_date_time() ||
          timestamp < last_timestamp_ ||
          (timestamp - last_timestamp_) > boost::posix_time::seconds(1)) {
        Seek(timestamp);
      } else if (timestamp >= last_video_timestamp_) {
        Step(timestamp);
      }

      last_timestamp_ = timestamp;

      const auto ws = ImGui::GetContentRegionAvail();
      const auto p = base::MaintainAspectRatio(codec_.size(), {ws.x, ws.y});
      ImGui::SameLine(p.min().x());
      ImGui::Image(reinterpret_cast<ImTextureID>(texture_.id()),
                   ImVec2(p.sizes().x(), p.sizes().y()));
    }
  }

  void Step(boost::posix_time::ptime timestamp) {
    ReadUntil(timestamp);
  }

  void Seek(boost::posix_time::ptime timestamp) {
    last_video_timestamp_ = {};

    const auto delta_s = mjlib::base::ConvertDurationToSeconds(
        timestamp - log_start_) - time_offset_s_;
    const int pts = delta_s * time_base_.den / time_base_.num;
    const int delta_pts = 1.0 * time_base_.den / time_base_.num;
    ffmpeg::File::SeekOptions seek_options;
    seek_options.any = false;
    file_.Seek(stream_, pts - delta_pts, pts, pts + delta_pts, seek_options);

    ReadUntil(timestamp);
  }

  void ReadUntil(boost::posix_time::ptime timestamp) {
    while (true) {
      auto maybe_pref = file_.Read(&packet_);
      if (!maybe_pref) {
        // EOF?
        return;
      }

      if ((*maybe_pref)->stream_index !=
          stream_.av_stream()->index) { continue; }

      codec_.SendPacket(*maybe_pref);

      auto maybe_fref = codec_.GetFrame(&frame_);
      if (!maybe_fref) { continue; }

      if (!swscale_) {
        swscale_.emplace(codec_, dest_frame_.size(), dest_frame_.format(),
                         ffmpeg::Swscale::kBicubic);
      }

      swscale_->Scale(*maybe_fref, dest_frame_ptr_);

      texture_.Store(dest_frame_ptr_->data[0]);

      last_video_timestamp_ =
          log_start_ +
          mjlib::base::ConvertSecondsToDuration(
              time_offset_s_ +
              static_cast<double>((*maybe_fref)->pts) *
              time_base_.num / time_base_.den);

      if (last_video_timestamp_ >= timestamp) {
        break;
      }
    }
  }

  boost::posix_time::ptime log_start_;
  double time_offset_s_ = 0.0;
  ffmpeg::File file_;
  ffmpeg::Stream stream_;
  ffmpeg::Codec codec_;
  std::optional<ffmpeg::Swscale> swscale_;
  ffmpeg::Packet packet_;
  ffmpeg::Frame frame_;
  ffmpeg::Frame dest_frame_;
  ffmpeg::Frame::Ref dest_frame_ptr_{dest_frame_.Allocate(
        AV_PIX_FMT_RGB24, codec_.size(), 1)};

  gl::FlatRgbTexture texture_{codec_.size()};

  AVRational time_base_ = stream_.av_stream()->time_base;
  int64_t start_pts_ = 0;

  boost::posix_time::ptime last_timestamp_;
  boost::posix_time::ptime last_video_timestamp_;
};

class SphereModel {
 public:
  SphereModel(int levels = 1) {
    for (auto tindex : tindicies_) {
      Subdivide(vdata_[tindex[0]],
                vdata_[tindex[1]],
                vdata_[tindex[2]],
                levels);
    }
  }

  struct Triangle {
    Eigen::Vector3f p1;
    Eigen::Vector3f p2;
    Eigen::Vector3f p3;
  };

  std::vector<Triangle> operator()(const Eigen::Vector3f& center, float radius) {
    std::vector<Triangle> result;
    for (const auto& triangle : unit_) {
      result.push_back(
          {triangle.p1 * radius + center,
                triangle.p2 * radius + center,
                triangle.p3 * radius + center});
    }
    return result;
  }

 private:
  void Subdivide(const Eigen::Vector3f& v1,
                 const Eigen::Vector3f& v2,
                 const Eigen::Vector3f& v3,
                 int depth) {
    if (depth == 0) {
      unit_.push_back({v1, v2, v3});
      return;
    }

    // Calculate midpoints of each side.
    const Eigen::Vector3f v12 = (0.5 * (v1 + v2)).normalized();
    const Eigen::Vector3f v23 = (0.5 * (v2 + v3)).normalized();
    const Eigen::Vector3f v13 = (0.5 * (v1 + v3)).normalized();

    const int next_depth = depth - 1;
    Subdivide(v1, v12, v13, next_depth);
    Subdivide(v2, v23, v12, next_depth);
    Subdivide(v3, v13, v23, next_depth);
    Subdivide(v12, v23, v13, next_depth);
  }

  std::vector<Triangle> unit_;

  const float X = 0.525731112119133696f;
  const float Z = 0.850650808352039932f;

  const Eigen::Vector3f vdata_[12] = {
    {-X, 0.0f, Z}, {X, 0.0f, Z}, {-X, 0.0f, -Z}, {X, 0.0f, -Z},
    {0.0f, Z, X}, {0.0f, Z, -X}, {0.0f, -Z, X}, {0.0f, -Z, -X},
    {Z, X, 0.0f}, {-Z, X, 0.0f}, {Z, -X, 0.0f}, {-Z, -X, 0.0f}
  };

  const Eigen::Vector3i tindicies_[20] = {
    {1,4,0}, {4,9,0}, {4,5,9}, {8,5,4}, {1,8,4},
    {1,10,8}, {10,3,8}, {8,3,5}, {3,2,5}, {3,7,2},
    {3,10,7}, {10,6,7}, {6,11,7}, {6,0,11}, {6,1,0},
    {10,1,6}, {11,0,9}, {2,11,9}, {5,2,9}, {11,2,7}
  };
};

class MechRender {
 public:
  MechRender(FileReader* reader, TreeView* tree_view)
      : reader_(reader->record("qc_status")->schema->root()),
        tree_view_(tree_view) {
    program_.use();
    vao_.bind();

    vertices_.bind(GL_ARRAY_BUFFER);

    program_.VertexAttribPointer(
        program_.attribute("inVertex"), 3, GL_FLOAT, GL_FALSE, 48, 0);
    program_.VertexAttribPointer(
        program_.attribute("inNormal"), 3, GL_FLOAT, GL_FALSE, 48, 12);
    program_.VertexAttribPointer(
        program_.attribute("inUv"), 2, GL_FLOAT, GL_FALSE, 48, 24);
    program_.VertexAttribPointer(
        program_.attribute("inColor"), 4, GL_FLOAT, GL_FALSE, 48, 32);

    vao_.unbind();

    program_.SetUniform(program_.uniform("lightPos"),
                        Eigen::Vector3f({-1000, 0, 3000}));
    program_.SetUniform(program_.uniform("currentTexture"), 0);
    program_.SetUniform(program_.uniform("modelMatrix"),
                        Eigen::Matrix4f(Eigen::Matrix4f::Identity()));
    program_.SetUniform(program_.uniform("projMatrix"),
                        camera_.matrix());

    // For now, our rendering texture will consist of a single white
    // pixel, which will just let us use the passed in color.
    const uint8_t white[4] = {255, 255, 255, 255};
    texture_.Store(white);
  }

  void Render() {
    const auto maybe_qc_status = tree_view_->data("qc_status");
    if (maybe_qc_status) {
      DrawMech(reader_.Read(*maybe_qc_status));
    }

    AddTriangle({-1, 1, 0}, {1, 1, 0}, {0, -1, 0}, {1, 0, 0, 0});
  }

  void DrawMech(const mech::QuadrupedControl::Status& qs) {
    AddBox({0, 0, 0},
           {230, 0, 0},
           {0, 240, 0},
           {0, 0, 125},
           {1.0, 0, 0, 1.0});

    for (const auto& leg_B : qs.state.legs_B) {
      AddBall(leg_B.position_mm.cast<float>(), 20, Eigen::Vector4f(0, 1, 0, 1));
    }
  }

  void AddBall(const Eigen::Vector3f& center,
               float radius,
               const Eigen::Vector4f& rgba) {
    for (const auto& t : sphere_(center, radius)) {
      AddTriangle(t.p1, t.p2, t.p3, rgba);
    }
  }

  void AddBox(const Eigen::Vector3f& center,
              const Eigen::Vector3f& length,
              const Eigen::Vector3f& width,
              const Eigen::Vector3f& height,
              const Eigen::Vector4f& rgba) {
    const Eigen::Vector3f hl = 0.5 * length;
    const Eigen::Vector3f hw = 0.5 * width;
    const Eigen::Vector3f hh = 0.5 * height;
    // Bottom
    AddQuad(center - hh - hw - hl,
            center - hh - hw + hl,
            center - hh + hw + hl,
            center - hh + hw - hl,
            rgba);
    // Top
    AddQuad(center + hh + hw - hl,
            center + hh + hw + hl,
            center + hh - hw + hl,
            center + hh - hw - hl,
            rgba);
    // Back
    AddQuad(center - hl - hh - hw,
            center - hl - hh + hw,
            center - hl + hh + hw,
            center - hl + hh - hw,
            rgba);
    // Front
    AddQuad(center + hl + hh - hw,
            center + hl + hh + hw,
            center + hl - hh + hw,
            center + hl - hh - hw,
            rgba);
    // Left
    AddQuad(center - hw - hh - hl,
            center - hw - hh + hl,
            center - hw + hh + hl,
            center - hw + hh - hl,
            rgba);
    // Right
    AddQuad(center + hw + hh - hl,
            center + hw + hh + hl,
            center + hw - hh + hl,
            center + hw - hh - hl,
            rgba);
  }

  void AddQuad(const Eigen::Vector3f& p1,
               const Eigen::Vector3f& p2,
               const Eigen::Vector3f& p3,
               const Eigen::Vector3f& p4,
               const Eigen::Vector4f& rgba) {
    const Eigen::Vector2f uv{0, 0};
    const Eigen::Vector3f normal = (p3 - p1).cross(p2 - p1).normalized();
    auto index1 = AddVertex(p1, normal, uv, rgba);
    auto index2 = AddVertex(p2, normal, uv, rgba);
    auto index3 = AddVertex(p3, normal, uv, rgba);
    auto index4 = AddVertex(p4, normal, uv, rgba);
    indices_.push_back(index1);
    indices_.push_back(index2);
    indices_.push_back(index3);

    indices_.push_back(index3);
    indices_.push_back(index4);
    indices_.push_back(index1);
  }

  void AddTriangle(const Eigen::Vector3f& p1,
                   const Eigen::Vector3f& p2,
                   const Eigen::Vector3f& p3,
                   const Eigen::Vector4f& rgba) {
    const Eigen::Vector3f normal = (p3 - p1).cross(p2 - p1);
    auto index1 = AddVertex(p1, normal, {0, 0}, rgba);
    auto index2 = AddVertex(p2, normal, {0, 0}, rgba);
    auto index3 = AddVertex(p3, normal, {0, 0}, rgba);
    indices_.push_back(index1);
    indices_.push_back(index2);
    indices_.push_back(index3);
  }

  uint32_t AddVertex(const Eigen::Vector3f& p1,
                     const Eigen::Vector3f& normal,
                     const Eigen::Vector2f& uv,
                     const Eigen::Vector4f& rgba) {
    const auto i = gpu_data_.size();
    gpu_data_.resize(i + 12);
    gpu_data_[i + 0] = p1.x();
    gpu_data_[i + 1] = p1.y();
    gpu_data_[i + 2] = p1.z();
    gpu_data_[i + 3] = normal.x();
    gpu_data_[i + 4] = normal.y();
    gpu_data_[i + 5] = normal.z();
    gpu_data_[i + 6] = uv.x();
    gpu_data_[i + 7] = uv.y();
    gpu_data_[i + 8] = rgba(0);
    gpu_data_[i + 9] = rgba(1);
    gpu_data_[i + 10] = rgba(2);
    gpu_data_[i + 11] = rgba(3);
    return i / 12;
  }

  void Update() {
    gpu_data_.clear();
    indices_.clear();

    Render();

    {
      gl::Framebuffer::Bind binder(framebuffer_);
      glViewport(0, 0, size_.x(), size_.y());
      glEnable(GL_DEPTH_TEST);
      glClearColor(0.45f, 0.55f, 0.60f, 1.0f);
      glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
      program_.use();
      program_.SetUniform(program_.uniform("viewMatrix"),
                          trackball_.matrix());

      vao_.bind();

      vertices_.set_vector(GL_ARRAY_BUFFER, gpu_data_, GL_STATIC_DRAW);
      elements_.set_vector(
          GL_ELEMENT_ARRAY_BUFFER, indices_, GL_STATIC_DRAW);

      texture_.bind();
      glDrawElements(GL_TRIANGLES, indices_.size(), GL_UNSIGNED_INT, 0);
      vao_.unbind();
    }

    gl::ImGuiWindow render("Render");

    const auto ws = ImGui::GetContentRegionAvail();
    const auto p = base::MaintainAspectRatio(size_, {ws.x, ws.y});
    ImGui::BeginChild("##ignored", ws);

    const auto& IO = ImGui::GetIO();
    auto window_pos = ImGui::GetWindowPos();
    auto mouse_pos = ImGui::GetMousePos();
    auto pos_pixel = ImVec2(mouse_pos.x - window_pos.x,
                            mouse_pos.y - window_pos.y);
    Eigen::Vector2f pos_norm = Eigen::Vector2f(
        std::max(-1.0f, std::min(2.0f, pos_pixel.x / p.sizes().x())),
        std::max(-1.0f, std::min(2.0f, pos_pixel.y / p.sizes().y())));
    if (ImGui::IsWindowHovered()) {
      for (int i = 0; i < 3; i++) {
        if (IO.MouseClicked[i]) {
          trackball_.MouseDown(pos_norm, i);
        }
      }
    }
    if (ImGui::IsWindowHovered() || trackball_.active()) {
      trackball_.MouseMove(pos_norm);
    }
    {
      for (int i = 0; i < 3; i++) {
        if (IO.MouseReleased[i]) {
          trackball_.MouseUp(pos_norm);
        }
      }
    }

    ImGui::ImageButton(reinterpret_cast<ImTextureID>(
                           imgui_texture_.id()),
                       ImVec2(p.sizes().x(), p.sizes().y()),
                       ImVec2(0, 0),
                       ImVec2(1, 1),
                       0);

    ImGui::EndChild();
  }

  SphereModel sphere_;

  mjlib::telemetry::MappedBinaryReader<mech::QuadrupedControl::Status> reader_;
  TreeView* const tree_view_;

  Eigen::Vector2i size_{1024, 768};

  gl::PerspectiveCamera camera_{[&]() {
      gl::PerspectiveCamera::Options options;
      options.aspect = static_cast<double>(size_.x()) /
                       static_cast<double>(size_.y());
      options.near = 100;
      options.far = 10000;
      return options;
    }()};

  gl::Trackball trackball_{{0.f, 0.f, 1000.f}, {0.f, 0.f, 0.f}};

  gl::Framebuffer framebuffer_;
  gl::FlatRgbTexture imgui_texture_{size_};
  gl::Renderbuffer renderbuffer_;
  const bool attach_ = [&]() {
    framebuffer_.attach(imgui_texture_.texture(), renderbuffer_);
    return true;
  }();

  gl::Shader vertex_shader_{kVertexShaderSource, GL_VERTEX_SHADER};
  gl::Shader fragment_shader_{kFragShaderSource, GL_FRAGMENT_SHADER};

  static constexpr const char* kVertexShaderSource =
      "#version 330 core\n"
      "in vec3 inVertex;\n"
      "in vec3 inNormal;\n"
      "in vec2 inUv;\n"
      "in vec4 inColor;\n"
      "uniform mat4 projMatrix;\n"
      "uniform mat4 viewMatrix;\n"
      "uniform mat4 modelMatrix;\n"
      "out vec2 fragUv;\n"
      "out vec4 fragColor;\n"
      "out vec3 fragNormal;\n"
      "out vec3 fragPos;\n"
      "void main(){\n"
      "  fragUv = inUv;\n"
      "  fragColor = inColor;\n"
      "  fragNormal = inNormal;\n"
      "  fragPos = vec3(viewMatrix * modelMatrix * vec4(inVertex.xyz, 1.0));\n"
      "  gl_Position = projMatrix * viewMatrix * modelMatrix * vec4(inVertex.xyz, 1);\n"
      "}\n"
      ;

  static constexpr const char* kFragShaderSource =
      "#version 330 core\n"
      "in vec2 fragUv;\n"
      "in vec4 fragColor;\n"
      "in vec3 fragNormal;\n"
      "in vec3 fragPos;\n"
      "uniform vec3 lightPos;\n"
      "uniform sampler2D currentTexture;\n"
      "void main() {\n"
      "  vec3 lightDir = normalize(lightPos - fragPos);\n"
      "  float ambient = 0.3;\n"
      "  float diff = max(dot(fragNormal, lightDir), 0);\n"
      "  vec4 lightModel = vec4((diff + ambient) * vec3(1.0, 1.0, 1.0), 1.0);\n"
      "  gl_FragColor = lightModel * fragColor * texture(currentTexture, fragUv);\n"
      "}\n"
      ;

  gl::Program program_{vertex_shader_, fragment_shader_};

  gl::FlatRgbTexture texture_{Eigen::Vector2i(1, 1), GL_RGBA};
  gl::VertexArrayObject vao_;
  gl::VertexBufferObject vertices_;
  gl::VertexBufferObject elements_;

  std::vector<float> gpu_data_;
  std::vector<uint32_t> indices_;
};
}

int do_main(int argc, char** argv) {
  ffmpeg::Ffmpeg::Register();

  std::string log_filename;
  std::string video_filename;
  double video_time_offset_s = 0.0;

  auto group = clipp::group(
      clipp::value("log file", log_filename),
      clipp::option("v", "video") & clipp::value("video", video_filename),
      clipp::option("voffset") & clipp::value("OFF", video_time_offset_s)
  );

  mjlib::base::ClippParse(argc, argv, group);

  mjlib::telemetry::FileReader file_reader(log_filename);
  const auto records = file_reader.records();

  gl::Window window(1280, 720, "tplot2");
  gl::GlImGui imgui(window);

  ImGui::GetIO().ConfigFlags |=
      ImGuiConfigFlags_DockingEnable;

  auto log_start = (*file_reader.items().begin()).timestamp;
  Timeline timeline{&file_reader};
  TreeView tree_view{&file_reader};
  PlotView plot_view{&file_reader, log_start};
  std::optional<Video> video;
  MechRender mech_render{&file_reader, &tree_view};

  if (!video_filename.empty()) {
    video.emplace(log_start, video_filename, video_time_offset_s);
  }

  while (!window.should_close()) {
    window.PollEvents();
    imgui.NewFrame();

    glClearColor(0.45f, 0.55f, 0.60f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);

    timeline.Update();
    const auto current = timeline.current();
    tree_view.Update(current);
    plot_view.Update(current);
    if (video) {
      video->Update(current);
    }
    mech_render.Update();

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
