// Copyright 2014-2015 Josh Pieper, jjp@pobox.com.  All rights reserved.
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

#include "base/quaternion.h"

#include <boost/test/auto_unit_test.hpp>

#include "base/euler.h"

namespace {
using namespace mjmech;
typedef base::Euler Euler;
typedef base::Quaternion Quaternion;
typedef base::Point3D Point3D;

void CheckVectorClose(const Point3D& lhs,
                      double x, double y, double z) {
  BOOST_CHECK_SMALL(lhs.x() - x, 1e-2);
  BOOST_CHECK_SMALL(lhs.y() - y, 1e-2);
  BOOST_CHECK_SMALL(lhs.z() - z, 1e-2);
}
}

BOOST_AUTO_TEST_CASE(BasicQuaternion) {
  Point3D v(10., 0., 0);

  v = Quaternion::FromEuler(0, 0, M_PI_2).Rotate(v);
  CheckVectorClose(v, 0, -10, 0);

  v = Quaternion::FromEuler(0, 0, -M_PI_2).Rotate(v);
  CheckVectorClose(v, 10, 0, 0);

  v = Quaternion::FromEuler(0, M_PI_2, 0).Rotate(v);
  CheckVectorClose(v, 10, 0, 0);

  v = Quaternion::FromEuler(M_PI_2, 0, 0).Rotate(v);
  CheckVectorClose(v, 0, 0, -10);

  v = Quaternion::FromEuler(0, 0, M_PI_2).Rotate(v);
  CheckVectorClose(v, 0, 0, -10);

  v = Quaternion::FromEuler(0, M_PI_2, 0).Rotate(v);
  CheckVectorClose(v, 0, 10, 0);

  v = Quaternion::FromEuler(M_PI_2, 0, 0).Rotate(v);
  CheckVectorClose(v, 0, 10, 0);

  v = Quaternion::FromEuler(0, 0, M_PI_2).Rotate(v);
  CheckVectorClose(v, 10, 0, 0);
}

namespace {
void CheckEuler(Euler euler_rad,
                double roll_rad,
                double pitch_rad,
                double yaw_rad) {
  BOOST_CHECK_SMALL(euler_rad.roll - roll_rad, 1e-5);
  BOOST_CHECK_SMALL(euler_rad.pitch - pitch_rad, 1e-5);
  BOOST_CHECK_SMALL(euler_rad.yaw - yaw_rad, 1e-5);
}
}

BOOST_AUTO_TEST_CASE(QuaternionEulerAndBack) {
  struct TestCase {
    double roll_deg;
    double pitch_deg;
    double yaw_deg;
  };

  TestCase tests[] = {
    {45, 0, 0},
    {0, 45, 0},
    {0, 0, 45},
    {0, 90, 0},
    {0, 90, 20},
    {0, -90, 0},
    {0, -90, -10},
    {0, -90, 30},
    {10, 20, 30},
    {-30, 10, 20},
  };

  for (const auto& x: tests) {
    Euler result_rad = Quaternion::FromEuler(
        x.roll_deg / 180 * M_PI,
        x.pitch_deg / 180 * M_PI,
        x.yaw_deg / 180 * M_PI).euler_rad();
    CheckEuler(result_rad,
               x.roll_deg / 180 * M_PI,
               x.pitch_deg / 180 *M_PI,
               x.yaw_deg / 180 * M_PI);
  }
}

BOOST_AUTO_TEST_CASE(QuaternionMultiply1) {
  Quaternion x90 = Quaternion::FromEuler(0, M_PI_2, 0);
  Quaternion xn90 = Quaternion::FromEuler(0, -M_PI_2, 0);
  Quaternion y90 = Quaternion::FromEuler(M_PI_2, 0, 0);

  Quaternion result = xn90 * y90 * x90;
  Point3D vector(0, 1, 0);
  vector = result.Rotate(vector);
  CheckVectorClose(vector, 1, 0, 0);

  Quaternion initial = Quaternion::FromEuler(0, 0, 0.5 * M_PI_2);
  initial = Quaternion::FromEuler(0, 0, 0.5 * M_PI_2) * initial;
  CheckEuler(initial.euler_rad(), 0, 0, M_PI_2);

  initial = Quaternion::FromEuler(0, 10 / 180. * M_PI, 0) * initial;
  vector = initial.Rotate(vector);
  CheckVectorClose(vector, 0, -0.9848078, -0.17364818);
  CheckEuler(initial.euler_rad(), 10 / 180.0 * M_PI, 0, M_PI_2);
}
