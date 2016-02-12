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

namespace mjmech {
namespace mech {

class TargetTracker::Impl : public CameraFrameConsumer {
 public:
  Impl(TargetTracker* parent, base::Context& context)
      : parent_(parent),
        service_(context.service) {}

  void AsyncStart(base::ErrorHandler handler) {
    service_.post(std::bind(handler, base::ErrorCode()));
  }

  void StartTracking(const base::Point3D&) {}

  void StopTracking() {}

  TargetTracker* const parent_;
  boost::asio::io_service& service_;

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
