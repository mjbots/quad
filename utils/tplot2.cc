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
//  * be able to plot sum total of down ground reaction force
//     * eventually also for things that are close to the ground
//  * render feet shadows on ground to give an idea of height off
//  * render a grid on ground (with units)
//  * render text velocities/forces near the arrows
//  * it would be nice to start with legs down
//  * plot time trajectories over a time window, or perhaps just
//    resettable trailers
//  * make line rendering be anti-aliased and support line width
//  * show exaggerated pitch and roll
//  * show some indication of foot slip
// * Video
//  * after rewinding, video sometimes doesn't start playing for a
//    good while
//  * pan / zoom
//  * hw accelerated decoding or color xform
// * multiple render/plot/tree widgets
// * Save/restore tree view expansion state and scroll location
// * Derived/scripted fields
// * search/filtering in tree widget


#include <fstream>
#include <string>
#include <variant>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/lexical_cast.hpp>

#include <fmt/format.h>

#include <implot.h>

#include "base/aspect_ratio.h"

#include "mjlib/base/buffer_stream.h"
#include "mjlib/base/clipp.h"
#include "mjlib/base/json5_read_archive.h"
#include "mjlib/base/json5_write_archive.h"
#include "mjlib/base/time_conversions.h"
#include "mjlib/base/tokenizer.h"
#include "mjlib/telemetry/file_reader.h"
#include "mjlib/telemetry/mapped_binary_reader.h"

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
#include "gl/simple_line_render_list.h"
#include "gl/simple_texture_render_list.h"
#include "gl/trackball.h"
#include "gl/vertex_array_object.h"
#include "gl/vertex_buffer_object.h"

#include "mech/attitude_data.h"
#include "mech/quadruped_control.h"

#include "utils/quadruped_tplot2.h"
#include "utils/tree_view.h"

using mjlib::telemetry::FileReader;
using Element = mjlib::telemetry::BinarySchemaParser::Element;
using Format = mjlib::telemetry::Format;
using FileReader = mjlib::telemetry::FileReader;
using FT = Format::Type;

namespace mjmech {
namespace utils {
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
  float step_ = 0.01;
  float fast_speed_ = 0.1;

  boost::posix_time::ptime last_update_;
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

  double operator()(const FileReader::Item& item) const {
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
          return std::numeric_limits<double>::quiet_NaN();
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

  double x(const FileReader::Item& item) const {
    return x_(item);
  }

  double y(const FileReader::Item& item) const {
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
  struct State {
    struct Plot {
      std::string x_token;
      std::string y_token;
      bool deriv = false;
      int axis = 0;

      template <typename Archive>
      void Serialize(Archive* a) {
        a->Visit(MJ_NVP(x_token));
        a->Visit(MJ_NVP(y_token));
        a->Visit(MJ_NVP(deriv));
        a->Visit(MJ_NVP(axis));
      }
    };

    std::vector<Plot> plots;

    struct Axis {
      double min = base::kNaN;
      double max = base::kNaN;

      template <typename Archive>
      void Serialize(Archive* a) {
        a->Visit(MJ_NVP(min));
        a->Visit(MJ_NVP(max));
      }
    };

    Axis x_axis;
    std::array<Axis, 3> y_axis = {};

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(plots));
      a->Visit(MJ_NVP(x_axis));
      a->Visit(MJ_NVP(y_axis));
    }
  };

  PlotView(FileReader* reader,
           TreeView* tree_view,
           boost::posix_time::ptime log_start,
           const State& initial)
      : reader_(reader),
        tree_view_(tree_view),
        log_start_(log_start) {
    for (const auto& plot : initial.plots) {
      current_axis_ = plot.axis;
      if (plot.deriv) {
        AddDerivPlot(plot.y_token);
      } else {
        AddLogPlot(plot.x_token, plot.y_token);
      }
    }

    if (std::isfinite(initial.x_axis.min) &&
        std::isfinite(initial.x_axis.max)) {
      ImPlot::SetNextPlotLimitsX(
          initial.x_axis.min, initial.x_axis.max, ImGuiCond_Always);
    }
    for (size_t i = 0; i < initial.y_axis.size(); i++) {
      const auto& y = initial.y_axis[i];
      if (std::isfinite(y.min) && std::isfinite(y.max)) {
        ImPlot::SetNextPlotLimitsY(y.min, y.max, ImGuiCond_Always, i);
      }
    }

    fit_plot_ = {};
  }

  State state() {
    State result;
    for (const auto& plot : plots_) {
      State::Plot out;
      out.x_token = plot.x_token;
      out.y_token = plot.y_token;
      out.deriv = plot.deriv;
      out.axis = plot.axis;
      result.plots.push_back(out);
    }

    {
      const auto x = x_limits_;
      result.x_axis.min = x.X.Min;
      result.x_axis.max = x.X.Max;
    }
    for (int i = 0; i < result.y_axis.size(); i++) {
      const auto y = y_limits_[i];
      result.y_axis[i].min = y.Y.Min;
      result.y_axis[i].max = y.Y.Max;
    }

    return result;
  }

  void Update(boost::posix_time::ptime timestamp) {
    ImGui::SetNextWindowPos(ImVec2(400, 0), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(800, 620), ImGuiCond_FirstUseEver);
    gl::ImGuiWindow file_window("Plot");

    if (fit_plot_) {
      const auto& p = **fit_plot_;
      double xmin = std::numeric_limits<float>::infinity();
      double xmax = -std::numeric_limits<float>::infinity();
      for (const auto& plot : plots_) {
        xmin = std::min(xmin, plot.min_x);
        xmax = std::max(xmax, plot.max_x);
      }
      ImPlot::SetNextPlotLimitsX(xmin, xmax, ImGuiCond_Always);
      ImPlot::SetNextPlotLimitsY(p.min_y, p.max_y, ImGuiCond_Always, p.axis);
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
    if (ImPlot::BeginPlot("Plot", "time", nullptr, ImVec2(-1, -25),
                         ImPlotFlags_Default | extra_flags)) {
      for (const auto& plot : plots_) {
        ImPlot::SetPlotYAxis(plot.axis);
        for (const auto& pair : plot.float_styles) {
          ImPlot::PushStyleVar(pair.first, pair.second);
        }
        for (const auto& pair : plot.int_styles) {
          ImPlot::PushStyleVar(pair.first, pair.second);
        }
        ImPlot::PlotLine(
            plot.legend.c_str(),
            plot.xvals.data(), plot.yvals.data(),
            plot.xvals.size());
        ImPlot::PopStyleVar(plot.float_styles.size() + plot.int_styles.size());

        const auto it = std::upper_bound(
            plot.timestamps.begin(), plot.timestamps.end(), timestamp);
        if (it != plot.timestamps.end()) {
          const auto index = std::max<int>(
              0, static_cast<int>(it - plot.timestamps.begin()) - 1);
          ImPlot::PushStyleVar(ImPlotStyleVar_Marker, ImPlotMarker_Diamond);
          ImPlot::PlotLine(
              ("##" + plot.legend + "_mrk").c_str(),
              plot.xvals.data() + index, plot.yvals.data() + index,
              1);
          ImPlot::PopStyleVar();
        }
      }

      x_limits_ = ImPlot::GetPlotLimits(0);
      for (int i = 0; i < y_limits_.size(); i++) {
        y_limits_[i] = ImPlot::GetPlotLimits(i);
      }
      ImPlot::EndPlot();
    }

    if (ImGui::BeginDragDropTarget()) {
      const auto* tlog_payload = ImGui::AcceptDragDropPayload("DND_TLOG");
      if (tlog_payload) {
        std::string token(static_cast<char*>(tlog_payload->Data),
                          tlog_payload->DataSize);
        AddLogPlot("", token);
      }
      const auto* deriv_payload = ImGui::AcceptDragDropPayload("DND_DERIV");
      if (deriv_payload) {
        std::string token(static_cast<char*>(deriv_payload->Data),
                          deriv_payload->DataSize);
        AddDerivPlot(token);
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
  struct Plot {
    std::string legend;

    std::vector<boost::posix_time::ptime> timestamps;
    std::vector<double> xvals;
    std::vector<double> yvals;

    double min_x = {};
    double max_x = {};
    double min_y = {};
    double max_y = {};

    std::map<int, float> float_styles {
      {ImPlotStyleVar_LineWeight, 1.0f },
      {ImPlotStyleVar_MarkerSize, 5.0f },
    };
    int marker_type = 0;
    std::map<int, int> int_styles {
      {ImPlotStyleVar_Marker, ImPlotMarker_None},
    };

    int axis = 0;

    bool deriv = false;
    std::string x_token;
    std::string y_token;
  };

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

  void AddLogPlot(const std::string& x_token, const std::string& y_token) {
    PlotRetrieve getter(reader_, log_start_, x_token, y_token);
    if (!getter.valid()) {
      return;
    }

    plots_.push_back({});
    auto& plot = plots_.back();
    plot.deriv = false;
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

    FinishPlot(&plot);
  }

  void AddDerivPlot(const std::string& token) {
    plots_.push_back({});
    auto& plot = plots_.back();

    plot.deriv = true;
    plot.y_token = token;

    plot.legend = MakeLegend("", token);

    auto data = tree_view_->ExtractDeriv(token);
    plot.timestamps = std::move(data.timestamps);
    plot.xvals = std::move(data.xvals);
    plot.yvals = std::move(data.yvals);

    if (plot.xvals.empty()) {
      plots_.pop_back();
      return;
    }

    FinishPlot(&plot);
  }

  void FinishPlot(Plot* plot) {
    // Switch all non-finite numbers to NaN.
    for (auto& value : plot->xvals) {
      if (!std::isfinite(value)) {
        value = std::numeric_limits<double>::quiet_NaN();
      }
    }
    for (auto& value : plot->yvals) {
      if (!std::isfinite(value)) {
        value = std::numeric_limits<double>::quiet_NaN();
      }
    }

    plot->min_x = *std::min_element(plot->xvals.begin(), plot->xvals.end());
    plot->max_x = *std::max_element(plot->xvals.begin(), plot->xvals.end());
    plot->min_y = *std::min_element(plot->yvals.begin(), plot->yvals.end());
    plot->max_y = *std::max_element(plot->yvals.begin(), plot->yvals.end());
    if (plot->max_y <= plot->min_y) {
      plot->max_y = plot->min_y + 1.0f;
    }
    if (plot->max_x <= plot->min_x) {
      plot->max_x = plot->max_x + 1.0f;
    }

    plot->axis = current_axis_;

    // If this is the only plot on this axis, then re-fit things.
    if (1 == std::count_if(
            plots_.begin(), plots_.end(),
            [&](const auto& plt) { return plt.axis == current_axis_; })) {
      fit_plot_ = plot;
    }

    current_plot_index_ = plots_.size() - 1;
  }

  FileReader* const reader_;
  TreeView* const tree_view_;
  boost::posix_time::ptime log_start_;

  static inline constexpr const char * kAxisNames[] = {
    "Left",
    "Right",
    "Aux",
  };

  std::vector<Plot> plots_;
  std::optional<Plot*> fit_plot_;
  size_t current_plot_index_ = 0;
  int current_axis_ = 0;

  ImPlotLimits x_limits_ = {};
  std::array<ImPlotLimits, 3> y_limits_ = {};
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
    ReadUntil(timestamp, false);
  }

  void Seek(boost::posix_time::ptime timestamp) {
    last_video_timestamp_ = {};

    const auto delta_s = mjlib::base::ConvertDurationToSeconds(
        timestamp - log_start_) - time_offset_s_;
    const int pts = std::max<int>(
        0, delta_s * time_base_.den / time_base_.num);
    ffmpeg::File::SeekOptions seek_options;
    seek_options.backward = true;
    file_.Seek(stream_, pts, seek_options);

    ReadUntil(timestamp, true);
  }

  void ReadUntil(boost::posix_time::ptime timestamp, bool discard_first) {
    int discard_count = discard_first ? 1 : 0;
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

      if (discard_count) {
        discard_count--;
        continue;
      }

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
  struct State {
    bool leg_actual = true;
    bool leg_command = true;
    bool leg_force = false;
    bool attitude = true;
    bool ground = true;
    bool support = true;
    bool target = true;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(leg_actual));
      a->Visit(MJ_NVP(leg_command));
      a->Visit(MJ_NVP(leg_force));
      a->Visit(MJ_NVP(attitude));
      a->Visit(MJ_NVP(ground));
      a->Visit(MJ_NVP(support));
      a->Visit(MJ_NVP(target));
    }
  };

  MechRender(FileReader* reader, TreeView* tree_view, State state)
      : reader_(reader->record("qc_status")->schema->root()),
        control_reader_(reader->record("qc_control")->schema->root()),
        imu_reader_(reader->record("imu")->schema->root()),
        tree_view_(tree_view),
        state_(state) {
    triangle_.SetProjMatrix(camera_.matrix());
    triangle_.SetLightPos({-1000, 0, -3000});

    lines_.SetProjMatrix(camera_.matrix());

    // For now, our rendering texture will consist of a single white
    // pixel, which will just let us use the passed in color.
    const uint8_t white[4] = {255, 255, 255, 255};
    texture_.Store(white);
  }

  State state() const {
    return state_;
  }

  void Render() {
    const auto maybe_qc_status = tree_view_->data("qc_status");
    const auto maybe_qc_control = tree_view_->data("qc_control");
    const auto maybe_imu = tree_view_->data("imu");
    if (maybe_qc_status && maybe_qc_control && maybe_imu &&
        !maybe_qc_status->empty() &&
        !maybe_qc_control->empty() &&
        !maybe_imu->empty()) {
      DrawMech(reader_.Read(*maybe_qc_status),
               control_reader_.Read(*maybe_qc_control),
               imu_reader_.Read(*maybe_imu));
    }
  }

  Eigen::Matrix3d AttitudeMatrix(const base::Quaternion& attitude) const {
    return attitude.matrix();
  }

  void DrawMech(const mech::QuadrupedControl::Status& qs,
                const mech::QuadrupedControl::ControlLog& qc,
                const mech::AttitudeData& attitude) {
    if (state_.ground) {
      DrawGround(qs, attitude);
    }

    if (state_.attitude) {
      transform_ = Eigen::Matrix4f::Identity();
      transform_.topLeftCorner<3, 3>() =
          AttitudeMatrix(attitude.attitude).cast<float>();
    } else {
      transform_ = Eigen::Matrix4f::Identity();
    }
    triangle_.SetTransform(transform_);
    lines_.SetTransform(transform_);

    AddBox({0, 0, 0},
           {0.230, 0, 0},
           {0, 0.240, 0},
           {0, 0, 0.125},
           {1.0, 0, 0, 1.0});

    if (state_.leg_actual) {
      for (const auto& leg_B : qs.state.legs_B) {
        AddBall(leg_B.position.cast<float>(),
                0.010, Eigen::Vector4f(0, 1, 0, 1));
        if (!state_.leg_force) {
          lines_.AddSegment(
              leg_B.position.cast<float>(),
              (leg_B.position +
               kVelocityDrawScale * leg_B.velocity).cast<float>(),
              Eigen::Vector4f(0, 1, 0, 1));
        } else {
          lines_.AddSegment(
              leg_B.position.cast<float>(),
              (leg_B.position +
               kForceDrawScale * leg_B.force_N).cast<float>(),
              Eigen::Vector4f(0, 1, 0, 1));
        }
      }
    }

    if (state_.leg_command) {
      for (const auto& leg_B : qc.legs_B) {
        AddBall(leg_B.position.cast<float>(),
                0.008, Eigen::Vector4f(0, 0, 1, 1));
        if (!state_.leg_force) {
          lines_.AddSegment(
              leg_B.position.cast<float>(),
              (leg_B.position +
               kVelocityDrawScale * leg_B.velocity).cast<float>(),
              Eigen::Vector4f(0, 0, 1, 1));
        } else {
          lines_.AddSegment(
              leg_B.position.cast<float>(),
              (leg_B.position +
               kForceDrawScale * leg_B.force_N).cast<float>(),
              Eigen::Vector4f(0, 0, 1, 1));
        }
      }
    }

    if (state_.target) {
      for (const auto& leg : qs.state.walk.legs) {
        Eigen::Vector3d target_B =
            qs.state.robot.frame_RB.pose.inverse() * leg.target_R;
        AddBall(target_B.cast<float>(),
                0.006, Eigen::Vector4f(0, 1, 1, 1));
      }
    }

    if (state_.support && !qs.state.legs_B.empty() && !qc.legs_B.empty()) {
      std::vector<base::Point3D> points;
      for (int i : {0, 1, 3, 2}) {
        const auto& leg_B = qc.legs_B[i];
        if (leg_B.stance != 0) {
          points.push_back(qs.state.legs_B[i].position);
        }
      }
      for (size_t i = 0; i < points.size(); i++) {
        lines_.AddSegment(
            points[i].cast<float>(),
            points[(i + 1) % points.size()].cast<float>(),
            Eigen::Vector4f(1, 0, 0, 1));
      }
      if (points.size() == 4) {
        // Render putative "next" supports.
        lines_.AddSegment(
            points[0].cast<float>(), points[2].cast<float>(),
            Eigen::Vector4f(1, 0.2, 0, 1));
        lines_.AddSegment(
            points[1].cast<float>(), points[3].cast<float>(),
            Eigen::Vector4f(1, 0, 0.2, 1));
      }
    }

    transform_ = Eigen::Matrix4f::Identity();
    triangle_.SetTransform(transform_);
    lines_.SetTransform(transform_);
  }

  void DrawGround(const mech::QuadrupedControl::Status& qs,
                  const mech::AttitudeData& attitude) {
    Eigen::Matrix3d tf_LB = AttitudeMatrix(attitude.attitude);

    // Stick the ground perpendicular to gravity at the location of
    // the lowest leg.
    double max_z_L = 0.0;

    for (const auto& leg_B : qs.state.legs_B) {
      Eigen::Vector3d position_L = tf_LB * leg_B.position;
      max_z_L = std::max(max_z_L, position_L.z());
    }

    const double l = kGroundSize;
    Eigen::Vector3f normal = Eigen::Vector3f(0, 0, -1);
    Eigen::Vector2f uv(0, 0);
    Eigen::Vector4f rgba(0.3, 0.3, 0.3, 1.0);

    if (!state_.attitude) {
      // We are rendering into the B frame.
      transform_ = Eigen::Matrix4f::Identity();
      transform_.topLeftCorner<3, 3>() = tf_LB.inverse().cast<float>();
      triangle_.SetTransform(transform_);
      lines_.SetTransform(transform_);
    }

    // Render our CoM projected onto the ground.
    AddBall(Eigen::Vector3f(0, 0, max_z_L), 0.006, Eigen::Vector4f(1, 0, 0, 1));

    auto ic = triangle_.AddVertex(Eigen::Vector3f(0, 0, max_z_L), normal, uv, rgba);
    for (int i = 0; i < 16; i++) {
      const double t1 = 2 * M_PI * (static_cast<double>(i) / 16);
      Eigen::Vector3f p1_L(l * std::cos(t1), l * std::sin(t1), max_z_L);

      // This could be more optimal and re-use edge indices as well.
      const double t2 = 2 * M_PI * (static_cast<double>((i + 1) % 16) / 16);
      Eigen::Vector3f p2_L(l * std::cos(t2), l * std::sin(t2), max_z_L);

      auto i1 = triangle_.AddVertex(p1_L, normal, uv, rgba);
      auto i2 = triangle_.AddVertex(p2_L, normal, uv, rgba);

      triangle_.AddTriangle(i1, i2, ic);
    }
  }

  void AddBall(const Eigen::Vector3f& center,
               float radius,
               const Eigen::Vector4f& rgba) {
    const Eigen::Vector2f uv(0.f, 0.f);
    for (const auto& t : sphere_(center, radius)) {
      triangle_.AddTriangle(t.p3, uv, t.p2, uv, t.p1, uv, rgba);
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
    const Eigen::Vector2f uv(0.f, 0.f);
    // Bottom
    triangle_.AddQuad(center - hh - hw - hl, uv,
                      center - hh - hw + hl, uv,
                      center - hh + hw + hl, uv,
                      center - hh + hw - hl, uv,
                      rgba);
    // Top
    triangle_.AddQuad(center + hh + hw - hl, uv,
                      center + hh + hw + hl, uv,
                      center + hh - hw + hl, uv,
                      center + hh - hw - hl, uv,
                      rgba);
    // Back
    triangle_.AddQuad(center - hl - hh - hw, uv,
                      center - hl - hh + hw, uv,
                      center - hl + hh + hw, uv,
                      center - hl + hh - hw, uv,
                      Eigen::Vector4f(0.f, 0.f, 1.f, 1.f));
    // Front
    triangle_.AddQuad(center + hl + hh - hw, uv,
                      center + hl + hh + hw, uv,
                      center + hl - hh + hw, uv,
                      center + hl - hh - hw, uv,
                      Eigen::Vector4f(0.f, 1.f, 0.f, 1.f));
    // Left
    triangle_.AddQuad(center - hw - hh - hl, uv,
                      center - hw - hh + hl, uv,
                      center - hw + hh + hl, uv,
                      center - hw + hh - hl, uv,
                      rgba);
    // Right
    triangle_.AddQuad(center + hw + hh - hl, uv,
                      center + hw + hh + hl, uv,
                      center + hw - hh + hl, uv,
                      center + hw - hh - hl, uv,
                      rgba);
  }

  void Update() {
    triangle_.Reset();
    lines_.Reset();

    Render();

    {
      gl::Framebuffer::Bind binder(framebuffer_);
      glViewport(0, 0, size_.x(), size_.y());
      glEnable(GL_DEPTH_TEST);
      glClearColor(0.45f, 0.55f, 0.60f, 1.0f);
      glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

      // TRIANGLES
      triangle_.Upload();
      triangle_.SetViewMatrix(trackball_.matrix());
      triangle_.SetModelMatrix(model_matrix_);
      triangle_.Render();

      // LINES
      lines_.Upload();
      lines_.SetViewMatrix(trackball_.matrix());
      lines_.SetModelMatrix(model_matrix_);
      lines_.Render();
    }

    gl::ImGuiWindow render("Render");

    const auto ws = ImGui::GetContentRegionAvail();
    const auto p = base::MaintainAspectRatio(size_, {ws.x, ws.y});
    ImGui::BeginChild("##ignored", ws);
    ImGui::Columns(2);
    ImGui::SetColumnWidth(0, ws.x - 100);

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

    ImGui::NextColumn();
    if (ImGui::Button("reset view")) {
      trackball_ = MakeTrackball();
    }
    ImGui::Checkbox("actual", &state_.leg_actual);
    ImGui::Checkbox("command", &state_.leg_command);
    ImGui::Checkbox("force", &state_.leg_force);
    ImGui::Checkbox("attitude", &state_.attitude);
    ImGui::Checkbox("ground", &state_.ground);
    ImGui::Checkbox("support", &state_.support);
    ImGui::Checkbox("target", &state_.target);

    ImGui::EndChild();
  }

  gl::Trackball MakeTrackball() {
    return gl::Trackball{{0.f, 0.f, 1.000f}, {0.f, 0.f, 0.f}};
  }

  SphereModel sphere_;

  mjlib::telemetry::MappedBinaryReader<mech::QuadrupedControl::Status> reader_;
  mjlib::telemetry::MappedBinaryReader<
    mech::QuadrupedControl::ControlLog> control_reader_;
  mjlib::telemetry::MappedBinaryReader<mech::AttitudeData> imu_reader_;
  TreeView* const tree_view_;

  Eigen::Vector2i size_{1024, 768};

  Eigen::Matrix4f model_matrix_{Eigen::Matrix4f::Identity()};
  Eigen::Matrix4f transform_{Eigen::Matrix4f::Identity()};
  gl::PerspectiveCamera camera_{[&]() {
      gl::PerspectiveCamera::Options options;
      options.aspect = static_cast<double>(size_.x()) /
                       static_cast<double>(size_.y());
      options.near = 0.100;
      options.far = 10.000;
      return options;
    }()};

  gl::Trackball trackball_ = MakeTrackball();

  gl::Framebuffer framebuffer_;
  gl::FlatRgbTexture imgui_texture_{size_};
  gl::Renderbuffer renderbuffer_;
  const bool attach_ = [&]() {
    framebuffer_.attach(imgui_texture_.texture(), renderbuffer_);
    return true;
  }();

  gl::FlatRgbTexture texture_{Eigen::Vector2i(1, 1), GL_RGBA};
  gl::SimpleTextureRenderList triangle_{&texture_.texture()};
  gl::SimpleLineRenderList lines_;

  State state_;

  const double kVelocityDrawScale = 0.1;
  const double kForceDrawScale = 2.0;
  const double kGroundSize = 0.500;
};

constexpr const char* kIniFileName = "tplot2.ini";

class Tplot2 {
 public:
  struct Options {
    std::string log_filename;
    std::string config_filename;
    std::string video_filename;
    double video_time_offset_s = 0.0;
  };

  static Options Parse(int argc, char** argv) {
    Options result;

    auto group = clipp::group(
        clipp::value("log file", result.log_filename),
        clipp::option("c", "config") &
        clipp::value("CONFIG", result.config_filename),
        clipp::option("v", "video") &
        clipp::value("video", result.video_filename),
        clipp::option("voffset") &
        clipp::value("OFF", result.video_time_offset_s)
    );

    mjlib::base::ClippParse(argc, argv, group);

    return result;
  }

  Tplot2(int argc, char** argv)
      : options_(Parse(argc, argv)) {
    ImGui::GetIO().ConfigFlags |=
        ImGuiConfigFlags_DockingEnable;

  }

  void Run() {
    while (!window_.should_close()) {
      CheckSave();

      window_.PollEvents();
      imgui_.NewFrame();

      glClearColor(0.45f, 0.55f, 0.60f, 1.0f);
      glClear(GL_COLOR_BUFFER_BIT);

      timeline_.Update();
      const auto current = timeline_.current();
      tree_view_.Update(current);
      plot_view_.Update(current);
      if (video_) {
        video_->Update(current);
      }
      if (mech_render_) {
        mech_render_->Update();
      }

      imgui_.Render();
      window_.SwapBuffers();
    }

    CheckSave(true);
  }

 private:
  void CheckSave(bool force = false) {
    auto& io = ImGui::GetIO();
    if (!io.WantSaveIniSettings && !force) { return; }

    size_t imgui_ini_size = 0;
    const char* imgui_ini = ImGui::SaveIniSettingsToMemory(&imgui_ini_size);
    io.WantSaveIniSettings = false;

    State to_save = initial_save_;
    to_save.imgui = std::string(imgui_ini, imgui_ini_size);
    to_save.window.size = window_.size();
    to_save.window.pos = window_.pos();
    to_save.plot = plot_view_.state();
    if (mech_) {
      to_save.mech = mech_render_->state();
    }

    std::ofstream of(kIniFileName);
    mjlib::base::Json5WriteArchive(of).Accept(&to_save);
  }

  struct State {
    std::string imgui;

    struct Window {
      Eigen::Vector2i size = { 1280, 720 };
      Eigen::Vector2i pos = { -1, -1 };

      template <typename Archive>
      void Serialize(Archive* a) {
        a->Visit(MJ_NVP(size));
        a->Visit(MJ_NVP(pos));
      }
    };

    Window window;
    MechRender::State mech;
    PlotView::State plot;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(imgui));
      a->Visit(MJ_NVP(window));
      a->Visit(MJ_NVP(mech));
      a->Visit(MJ_NVP(plot));
    }
  };

  const Options options_;

  const State initial_save_ = [&]() {
    std::ifstream in(options_.config_filename.empty() ?
                     kIniFileName : options_.config_filename.c_str());
    if (!in.is_open()) { return State(); }

    State state;
    try {
      mjlib::base::Json5ReadArchive(in).Accept(&state);
    } catch (mjlib::base::system_error& se) {
      std::cout << "Error reading ini, ignoring: " << se.what();
    }
    return state;
  }();

  const bool ffmpeg_register_ =
      []() { ffmpeg::Ffmpeg::Register(); return true; }();
  mjlib::telemetry::FileReader file_reader_{options_.log_filename};
  const bool mech_ = (file_reader_.record("qc_status") != nullptr);
  gl::Window window_{
    initial_save_.window.size.x(), initial_save_.window.size.y(),
        "tplot2"};
  const bool setpos_ = [&]() {
    if (initial_save_.window.pos != Eigen::Vector2i(-1, -1)) {
      window_.set_pos(initial_save_.window.pos);
    }
    return true;
  }();
  gl::GlImGui imgui_{window_,  []() {
      gl::GlImGui::Options options;
      options.persist_settings = false;
      return options;
    }()};

  const bool load_imgui_ = [&]() {
    if (initial_save_.imgui.empty()) { return false; }

    ImGui::LoadIniSettingsFromMemory(
        initial_save_.imgui.data(),
        initial_save_.imgui.size());
    return true;
  }();

  const boost::posix_time::ptime log_start_ =
      (*file_reader_.items().begin()).timestamp;
  Timeline timeline_{&file_reader_};
  TreeView tree_view_{&file_reader_, log_start_};
  PlotView plot_view_{&file_reader_, &tree_view_, log_start_, initial_save_.plot};

  std::optional<Video> video_;
  std::optional<MechRender> mech_render_;
  const bool mech_register_ = [&]() {
    if (mech_) {
      AddQuadrupedDerived(&file_reader_, &tree_view_);
      mech_render_.emplace(&file_reader_, &tree_view_, initial_save_.mech);
    }

    return true;
  }();

  const bool video_register_ = [&]() {
    if (!options_.video_filename.empty()) {
      video_.emplace(log_start_,
                     options_.video_filename,
                     options_.video_time_offset_s);
    }

    return true;
  }();
};
}

int do_main(int argc, char** argv) {
  Tplot2 tplot2(argc, argv);
  tplot2.Run();

  return 0;
}
}
}

int main(int argc, char** argv) {
  return mjmech::utils::do_main(argc, argv);
}
