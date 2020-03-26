// Copyright 2014-2015 Josh Pieper, jjp@pobox.com.
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

#include "base/tf.h"

#include <boost/test/auto_unit_test.hpp>

namespace {
using namespace mjmech::base;

void CheckVectorsClose(const Point3D& p1,
                       const Point3D& p2) {
  BOOST_CHECK_SMALL(p1.x() - p2.x(), 1e-3);
  BOOST_CHECK_SMALL(p1.y() - p2.y(), 1e-3);
  BOOST_CHECK_SMALL(p1.z() - p2.z(), 1e-3);
}

void CheckMapping(const Frame& frame,
                  Point3D p1,
                  Point3D p2) {
  CheckVectorsClose(frame.MapToParent(p1), p2);
  CheckVectorsClose(frame.MapFromParent(p2), p1);
}

void CheckFrames(const Frame& f1, Point3D f1p,
                 const Frame& f2, Point3D f2p) {
  CheckVectorsClose(f1.MapToFrame  (&f2, f1p), f2p);
  CheckVectorsClose(f2.MapFromFrame(&f1, f1p), f2p);
  CheckVectorsClose(f2.MapToFrame  (&f1, f2p), f1p);
  CheckVectorsClose(f1.MapFromFrame(&f2, f2p), f1p);
}
}

BOOST_AUTO_TEST_CASE(TestSimpleTransforms) {
  {
    Frame frame({10, 0, 0}, Quaternion());

    CheckMapping(frame, {0, 0, 0}, {10, 0, 0});
    CheckMapping(frame, {1, 0, 0}, {11, 0, 0});
    CheckMapping(frame, {1, 2, 0}, {11, 2, 0});
    CheckMapping(frame, {1, 2, 3}, {11, 2, 3});
  }

  {
    Frame frame({0, 10, 0},
                Quaternion::FromEuler(0, 0, 0.5 * M_PI));

    CheckMapping(frame, {0, 0, 0}, {0, 10, 0});
    CheckMapping(frame, {1, 0, 0}, {0, 9, 0});
    CheckMapping(frame, {-1, 0, -2}, {0, 11, -2});
  }

  {
    Frame frame({0, 0, 3},
                Quaternion::FromEuler(-0.5 * M_PI, 0, 0));
    CheckMapping(frame, {0, 0, 0}, {0, 0, 3});
    CheckMapping(frame, {0, 0, 1}, {-1, 0, 3});
  }
}

BOOST_AUTO_TEST_CASE(TestFrameChains) {
  Frame root({0, 0, 0}, Quaternion());

  Frame child1({10, 2, 0}, Quaternion(), &root);
  Frame child2({-3, -5, 0}, Quaternion(), &root);

  CheckMapping(child1, {0, 0, 0}, {10, 2, 0});
  CheckFrames(child1, {0, 0, 0}, root, {10, 2, 0});
  CheckFrames(child2, {0, 0, 0}, root, {-3, -5, 0});
  CheckFrames(child1, {0, 0, 0}, child2, {13, 7, 0});

  Frame subchild1({1, 2, 0}, Quaternion(), &child1);
  CheckMapping(subchild1, {0, 0, 0}, {1, 2, 0});
  CheckFrames(subchild1, {0, 0, 0}, child1, {1, 2, 0});
  CheckFrames(subchild1, {0, 0, 0}, root, {11, 4, 0});
  CheckFrames(subchild1, {0, 0, 0}, child2, {14, 9, 0});
}
