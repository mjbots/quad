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

#include "target_tracker.h"

#include <gst/gst.h>

#include <opencv2/opencv.hpp>

#include "base/logging.h"

namespace mjmech {
namespace mech {

class TargetTracker::Impl : public CameraFrameConsumer {
 public:
  Impl(TargetTracker* parent, base::Context& context)
      : parent_(parent),
        service_(context.service) {}

  void AsyncStart(base::ErrorHandler handler) {
    enabled_ = true;
    service_.post(std::bind(handler, base::ErrorCode()));
  }

  void StartTracking(const base::Point3D&) {}

  void StopTracking() {}

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

    cv::Mat mat(height, width, CV_8UC1, info.data);

    // TODO jpieper: Implement me.
  }

  TargetTracker* const parent_;
  boost::asio::io_service& service_;
  base::LogRef log_ = base::GetLogInstance("TargetTracker");

  bool enabled_ = false;

  TargetTrackerData data_;
  TargetTrackerDataSignal data_signal_;
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
