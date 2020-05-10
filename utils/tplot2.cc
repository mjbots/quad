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
// * Plots
//  * I get crazy artifacts when non-first plots are entirely off screen
//  * multiple y axes
// * Video
// * 3D mech
// * Save/restore plot configuration
// * Derived/scripted fields

#include <string>
#include <variant>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/lexical_cast.hpp>

#include <fmt/format.h>

#include <implot.h>

#include "gl/gl_imgui.h"

#include "mjlib/base/buffer_stream.h"
#include "mjlib/base/clipp.h"
#include "mjlib/base/time_conversions.h"
#include "mjlib/base/tokenizer.h"
#include "mjlib/telemetry/file_reader.h"

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
            Parent new_parent;
            new_parent.parent = parent;
            new_parent.element = element;
            VisitElement(field.element, stream, &new_parent);
          }
          break;
        }
        case FT::kArray: {
          const auto nelements = element->ReadArraySize(stream);
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
          case FT::kArray: {
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
        case FT::kArray: {
          const uint64_t size = element->ReadArraySize(stream);
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
  PlotView(FileReader* reader)
      : reader_(reader),
        log_start_((*reader->items().begin()).timestamp) {
  }

  void Update(boost::posix_time::ptime timestamp) {
    ImGui::SetNextWindowPos(ImVec2(400, 0), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(800, 620), ImGuiCond_FirstUseEver);
    gl::ImGuiWindow file_window("Plot");

    if (fit_contents_) {
      MJ_ASSERT(!plots_.empty());
      const auto& p = plots_.front();
      ImGui::SetNextPlotRange(p.min_x, p.max_x, p.min_y, p.max_y, ImGuiCond_Always);
      fit_contents_ = false;
    }
    if (ImGui::BeginPlot("Plot", "time", nullptr, ImVec2(-1, -25),
                         ImPlotFlags_Default)) {  // | ImPlotFlags_Y2Axis
      for (const auto& plot : plots_) {
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

    if (ImGui::BeginCombo("Plots", current_plot_name().c_str())) {
      for (size_t i = 0; i < plots_.size(); i++) {
        if (ImGui::Selectable(
                plots_[i].legend.c_str(), i == current_plot_index_)) {
          current_plot_index_ = i;
        }
      }
      ImGui::EndCombo();
    }
    ImGui::SameLine(0, 20.0);
    if (ImGui::Button("Remove")) {
      plots_.erase(plots_.begin() + current_plot_index_);
      if (current_plot_index_ > 0 && current_plot_index_ >= plots_.size()) {
        current_plot_index_--;
      }
    }
    ImGui::SameLine(0, 10.0);
    if (ImGui::Button("Properties")) {
      ImGui::OpenPopup("Plot Properties");
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

    fit_contents_ = plots_.size() == 1;
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
  };

  std::vector<Plot> plots_;
  bool fit_contents_ = false;
  size_t current_plot_index_ = 0;
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
  PlotView plot_view{&file_reader};

  while (!window.should_close()) {
    window.PollEvents();
    imgui.NewFrame();

    glClearColor(0.45f, 0.55f, 0.60f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);

    timeline.Update();
    tree_view.Update(timeline.current());
    plot_view.Update(timeline.current());

    ImGui::ShowDemoWindow();
    ImGui::ShowImPlotDemoWindow();

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
