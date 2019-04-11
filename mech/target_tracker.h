// Copyright 2016-2019 Josh Pieper, jjp@pobox.com.  All rights reserved.
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

#pragma once

#include "base/point3d.h"

#include "camera_driver.h"
#include "target_tracker_data.h"

namespace mjmech {
namespace mech  {

class TargetTracker : boost::noncopyable {
 public:
  TargetTracker(base::Context& context);
  ~TargetTracker();

  void AsyncStart(mjlib::io::ErrorCallback);

  struct Parameters {
    int region_width = 420;
    int region_height = 420;

    template <typename Archive>
    void Serialize(Archive* a)  {
      a->Visit(MJ_NVP(region_width));
      a->Visit(MJ_NVP(region_height));
    }
  };

  Parameters* parameters() { return &parameters_; }

  // Get a frameconsumer interface. pointer has same lifetime
  // as this object.
  std::weak_ptr<CameraFrameConsumer> get_frame_consumer();

  const TargetTrackerData& data() const;
  TargetTrackerDataSignal* data_signal();

 private:
  Parameters parameters_;

  class Impl;
  std::shared_ptr<Impl> impl_;
};

}
}
