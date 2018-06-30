// Copyright 2016 Josh Pieper, jjp@pobox.com.  All rights reserved.
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

/// @file
///
/// All OpenCV operations are currently commented out as it is hard to
/// get to work on the joule and xenial simultaneously at the moment.

#include "target_tracker.h"

#include <mutex>

#include <gst/gst.h>

#include "base/logging.h"

namespace mjmech {
namespace mech {

// namespace {
// const int kMaxFeatures = 50;
// const int kMinFeatures = 10;
// const double kQualityLevel = 0.01;
// const double kMinDistance = 3.0;
// }

class TargetTracker::Impl : public CameraFrameConsumer {
 public:
  Impl(TargetTracker* parent, base::Context& context)
      : parent_(parent),
        service_(context.service) {}

  ~Impl() override {}

  void AsyncStart(base::ErrorHandler handler) {
    enabled_ = true;
    service_.post(std::bind(handler, base::ErrorCode()));
  }

  void StartTracking(const base::Point3D& point) {
    std::lock_guard<std::mutex> guard(mutex_);
    data_.state = TargetTrackerData::kStarting;
    data_.features.clear();
    data_.initial = point;
  }

  void StopTracking() {
    std::lock_guard<std::mutex> guard(mutex_);
    data_.state = TargetTrackerData::kIdle;
    data_.features.clear();
  }

  // The CameraFrameConsumer interface.
  void ConsumeRawSample(GstSample* sample) override {
    if (!enabled_) { return; }

    GstCaps* caps = gst_sample_get_caps(sample);
    GstStructure* str = gst_caps_get_structure(caps, 0);

    // Apparently we should have a defined capabilities structure at
    // this point.
    BOOST_ASSERT(GST_CAPS_IS_SIMPLE(caps));

    if (log_.isDebugEnabled()) {
      char* caps_str = gst_caps_to_string(caps);
      log_.debug("target caps: %s", caps_str);
      g_free(caps_str);
    }

    // TODO jpieper: Assert that the format is I420.

    GstBuffer* buf = gst_sample_get_buffer(sample);
    BOOST_ASSERT(buf != nullptr);

    const size_t len = gst_buffer_get_size(buf);
    GstMemory* mem = gst_buffer_get_all_memory(buf);
    GstMapInfo info;
    std::memset(&info, 0, sizeof(info));
    const bool ok = gst_memory_map(mem, &info, GST_MAP_READ);
    BOOST_ASSERT(ok);
    BOOST_ASSERT(info.size == len);

    gint width = 0;
    gint height = 0;
    BOOST_ASSERT(gst_structure_get_int(str, "width", &width));
    BOOST_ASSERT(gst_structure_get_int(str, "height", &height));

    // cv::Mat mat(height, width, CV_8UC1, info.data);

    // HandleImage(mat);

    gst_memory_unmap(mem, &info);
    gst_memory_unref(mem);
  }

  // void HandleImage(const cv::Mat& mat) {
  //   auto data = [&]() {
  //     std::lock_guard<std::mutex> guard(mutex_);
  //     return data_;
  //   }();

  //   switch (data.state) {
  //     case TargetTrackerData::kIdle: { return; }
  //     case TargetTrackerData::kStarting: {
  //       HandleStartTracking(mat, &data);
  //       data.state = TargetTrackerData::kTracking;
  //       break;
  //     }
  //     case TargetTrackerData::kTracking: {
  //       HandleTracking(mat, &data);
  //       break;
  //     }
  //   }

  //   {
  //     std::lock_guard<std::mutex> guard(mutex_);
  //     data_ = data;
  //   }

  //   service_.post(std::bind(&Impl::Update, this));
  // }

  void Update() {
    std::lock_guard<std::mutex> guard(mutex_);
    data_signal_(&data_);
  }

  // cv::Rect GetRect(const base::Point3D& point,
  //                  const cv::Size& size) const {
  //   const auto& p = parent_->parameters_;
  //   const double x1 = std::max(0.0, point.x - 0.5 * p.region_width);
  //   const double y1 = std::max(0.0, point.y - 0.5 * p.region_height);

  //   const double w = std::min(1.0 * p.region_width, size.width - x1);
  //   const double h = std::min(1.0 * p.region_height, size.height - y1);

  //   return cv::Rect(x1, y1, w, h);
  // }

  // void HandleStartTracking(const cv::Mat& mat, TargetTrackerData* data) {
  //   old_image_ = mat.clone();
  //   const auto rect = GetRect(data->initial, mat.size());
  //   cv::Mat roi(old_image_, rect);
  //   std::vector<cv::Point2f> features;
  //   cv::goodFeaturesToTrack(roi, features,
  //                           kMaxFeatures, kQualityLevel, kMinDistance);
  //   data->features.clear();
  //   for (const auto& p: features) {
  //     data->features.push_back({p.x + rect.x, p.y + rect.y, 0.0});
  //   }

  //   data->current = data->initial;
  // }

  // void HandleTracking(const cv::Mat& mat, TargetTrackerData* data) {
  //   // TODO jpieper: Add margin around this.
  //   const auto rect = GetRect(data->current, mat.size());
  //   cv::Mat old_roi(old_image_, rect);
  //   cv::Mat roi(mat, rect);

  //   std::vector<cv::Point2f> old_features;
  //   for (const auto& p: data->features) {
  //     old_features.push_back(cv::Point2f(p.x - rect.x, p.y - rect.y));
  //   }
  //   std::vector<cv::Point2f> new_features;

  //   std::vector<unsigned char> status;
  //   std::vector<double> err;
  //   cv::calcOpticalFlowPyrLK(old_image_, mat, old_features, new_features,
  //                            status, err);

  //   BOOST_ASSERT(status.size() == new_features.size());
  //   data->features.clear();
  //   base::Point3D total;
  //   for (std::size_t i = 0; i < new_features.size(); i++) {
  //     if (status[i] == 0) { continue; }
  //     data->features.push_back(
  //         {new_features[i].x + rect.x,
  //          new_features[i].y + rect.y, 0.0});
  //     total += data->features.back();
  //   }

  //   data->current = total.scaled(1.0 / data->features.size());

  //   if (data->features.size() < kMinFeatures) {
  //     data->state = TargetTrackerData::kIdle;
  //   }

  //   old_image_ = mat.clone();
  // }


  TargetTracker* const parent_;
  boost::asio::io_service& service_;
  base::LogRef log_ = base::GetLogInstance("TargetTracker");

  bool enabled_ = false;

  std::mutex mutex_;
  TargetTrackerData data_;
  TargetTrackerDataSignal data_signal_;

  // cv::Mat old_image_;
  // std::vector<cv::Point2f> features_;
};

TargetTracker::TargetTracker(base::Context& context)
    : impl_(new Impl(this, context)) {}

TargetTracker::~TargetTracker() {}

void TargetTracker::AsyncStart(base::ErrorHandler handler) {
  impl_->AsyncStart(handler);
}

std::weak_ptr<CameraFrameConsumer> TargetTracker::get_frame_consumer() {
  return impl_;
}

void TargetTracker::StartTracking(const base::Point3D& point) {
  impl_->StartTracking(point);
}

void TargetTracker::StopTracking() {
  impl_->StopTracking();
}

const TargetTrackerData& TargetTracker::data() const {
  return impl_->data_;
}

TargetTrackerDataSignal* TargetTracker::data_signal() {
  return &impl_->data_signal_;
}

}
}
