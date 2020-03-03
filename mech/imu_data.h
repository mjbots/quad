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

#include "mjlib/base/visitor.h"

#include "base/point3d.h"

namespace mjmech {
namespace mech {

struct ImuData {
  boost::posix_time::ptime timestamp;

  base::Point3D rate_deg_s;
  base::Point3D accel_mps2;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(timestamp));
    a->Visit(MJ_NVP(rate_deg_s));
    a->Visit(MJ_NVP(accel_mps2));
  }
};

}
}
