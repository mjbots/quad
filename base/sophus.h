// Copyright 2019 Josh Pieper, jjp@pobox.com.
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

#include <sophus/se3.hpp>

#include "mjlib/base/eigen.h"
#include "mjlib/base/visitor.h"

namespace mjlib {
namespace base {

template <typename Scalar>
struct ExternalSerializer<Sophus::SO3<Scalar>> {
  struct Wrapper {
    Wrapper(Sophus::SO3<Scalar>* wrapped) : wrapped_(wrapped) {}

    template <typename Archive>
    void Serialize(Archive* a) {
      auto* data = wrapped_->data();
      a->Visit(mjlib::base::MakeNameValuePair(&data[0], "x"));
      a->Visit(mjlib::base::MakeNameValuePair(&data[1], "y"));
      a->Visit(mjlib::base::MakeNameValuePair(&data[2], "z"));
      a->Visit(mjlib::base::MakeNameValuePair(&data[3], "w"));
    }

    Sophus::SO3<Scalar>* wrapped_;
  };

  template <typename PairReceiver>
  void Serialize(Sophus::SO3<Scalar>* value, PairReceiver receiver) {
    Wrapper wrapper(value);
    receiver(mjlib::base::MakeNameValuePair(&wrapper, ""));
  }
};

template <typename Scalar>
struct ExternalSerializer<Sophus::SE3<Scalar>> {
  struct Wrapper {
    Wrapper(Sophus::SE3<Scalar>* wrapped) : wrapped_(wrapped) {}

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(mjlib::base::MakeNameValuePair(
                   &wrapped_->so3(), "so3"));
      a->Visit(mjlib::base::MakeNameValuePair(
                   &wrapped_->translation(), "translation"));
    }

    Sophus::SE3<Scalar>* wrapped_;
  };

  template <typename PairReceiver>
  void Serialize(Sophus::SE3<Scalar>* value, PairReceiver receiver) {
    Wrapper wrapper(value);
    receiver(mjlib::base::MakeNameValuePair(&wrapper, ""));
  }
};

}
}
