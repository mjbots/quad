// Copyright 2016-2019 Josh Pieper, jjp@pobox.com.
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

#include "target_tracker.h"

#include <mutex>

#include <gst/gst.h>

#include <fmt/format.h>

#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include "mjlib/io/now.h"

#include "base/logging.h"
#include "base/telemetry_registry.h"

namespace mjmech {
namespace mech {

class TargetTracker::Impl : public CameraFrameConsumer {
 public:
  Impl(TargetTracker* parent, base::Context& context)
      : parent_(parent),
        executor_(context.executor) {
    context.telemetry_registry->Register("target_data", &data_signal_);

    cv::setNumThreads(1);
  }

  ~Impl() override {}

  void AsyncStart(mjlib::io::ErrorCallback handler) {
    log_.debug("AsyncStart");
    enabled_ = true;

    boost::asio::post(
        executor_,
        std::bind(std::move(handler), mjlib::base::error_code()));
  }

  // The CameraFrameConsumer interface.
  void ConsumeRawSample(GstSample* sample) override {
    log_.debug("ConsumeRawSample");

    // We are in a background thread.
    if (!enabled_) { return; }

    GstCaps* caps = gst_sample_get_caps(sample);
    GstStructure* str = gst_caps_get_structure(caps, 0);

    // Apparently we should have a defined capabilities structure at
    // this point.
    BOOST_VERIFY(GST_CAPS_IS_SIMPLE(caps));

    if (log_.isDebugEnabled()) {
      char* caps_str = gst_caps_to_string(caps);
      log_.debug("target caps: %s", caps_str);
      g_free(caps_str);
    }

    // TODO jpieper: Assert that the format is I420.

    GstBuffer* buf = gst_sample_get_buffer(sample);
    BOOST_VERIFY(buf != nullptr);

    const size_t len = gst_buffer_get_size(buf);
    GstMemory* mem = gst_buffer_get_all_memory(buf);
    GstMapInfo info;
    std::memset(&info, 0, sizeof(info));
    const bool ok = gst_memory_map(mem, &info, GST_MAP_READ);
    BOOST_VERIFY(ok);
    BOOST_VERIFY(info.size == len);

    gint width = 0;
    gint height = 0;
    BOOST_VERIFY(gst_structure_get_int(str, "width", &width));
    BOOST_VERIFY(gst_structure_get_int(str, "height", &height));

    cv::Mat mat(height, width, CV_8UC1, info.data);

    HandleImage(mat);

    gst_memory_unmap(mem, &info);
    gst_memory_unref(mem);
  }

  void HandleImage(const cv::Mat& mat) {
    // We are in a background thread.
    log_.debug(fmt::format("got frame {:d}", frame_count_));

    auto data = [&]() {
      std::lock_guard<std::mutex> guard(mutex_);
      auto copy = data_;
      copy.timestamp = mjlib::io::Now(this->executor_.context());
      return copy;
    }();


    const auto& p = parent_->parameters_;
    const auto center = cv::Point2f(mat.cols / 2, mat.rows / 2);
    cv::Rect crop_area = cv::Rect(
        mat.cols / 2 - p.region_width / 2,
        mat.rows / 2 - p.region_height / 2,
        p.region_width,
        p.region_height) &
        cv::Rect(0, 0, mat.cols, mat.rows);
    cv::Mat subset = mat(crop_area);

    std::vector<int> marker_ids;
    std::vector<std::vector<cv::Point2f>> marker_corners;

    cv::aruco::detectMarkers(
        subset, aruco_dictionary_,
        marker_corners, marker_ids,
        aruco_parameters_);

    if (marker_ids.empty()) {
      data.target = std::nullopt;
    } else {
      // For now, just pick the one with a first corner closest to the center.
      auto it = std::min_element(
          marker_corners.begin(), marker_corners.end(),
          [&](const auto& lhs_corners, const auto& rhs_corners) {
            return (cv::norm(lhs_corners.front() - center) <
                    cv::norm(rhs_corners.front() - center));
          });
      BOOST_VERIFY(it != marker_corners.end());

      TargetTrackerData::Target target;
      base::Point3D total;
      for (const auto& corner : *it) {
        target.corners.emplace_back(corner.x + crop_area.x,
                                    corner.y + crop_area.y, 0);
        total += target.corners.back();
      }

      target.center = total * (1.0 / it->size());
      data.target = target;
    }

    {
      std::lock_guard<std::mutex> guard(mutex_);
      data_ = data;
    }

    boost::asio::post(
        executor_,
        std::bind(&Impl::Update, this));
  }

  void Update() {
    std::lock_guard<std::mutex> guard(mutex_);
    data_signal_(&data_);
  }

  TargetTracker* const parent_;
  boost::asio::executor executor_;
  base::LogRef log_ = base::GetLogInstance("TargetTracker");

  bool enabled_ = false;

  int frame_count_ = 0;

  std::mutex mutex_;
  TargetTrackerData data_;
  TargetTrackerDataSignal data_signal_;

  cv::Ptr<cv::aruco::Dictionary> aruco_dictionary_ =
      cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
  cv::Ptr<cv::aruco::DetectorParameters> aruco_parameters_ =
      cv::aruco::DetectorParameters::create();
};

TargetTracker::TargetTracker(base::Context& context)
    : impl_(new Impl(this, context)) {}

TargetTracker::~TargetTracker() {}

void TargetTracker::AsyncStart(mjlib::io::ErrorCallback handler) {
  impl_->AsyncStart(std::move(handler));
}

std::weak_ptr<CameraFrameConsumer> TargetTracker::get_frame_consumer() {
  return impl_;
}

const TargetTrackerData& TargetTracker::data() const {
  return impl_->data_;
}

TargetTrackerDataSignal* TargetTracker::data_signal() {
  return &impl_->data_signal_;
}

}
}
