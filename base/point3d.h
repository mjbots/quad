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

#pragma once

#include <boost/format.hpp>

#include "common.h"
#include "visitor.h"

namespace mjmech {
namespace base {
struct Point3D {
  double x;
  double y;
  double z;

  Point3D() : x(), y(), z() {}
  Point3D(double x, double y, double z) : x(x), y(y), z(z) {}

  double operator[](size_t index) const {
    switch (index) {
      case 0: return x;
      case 1: return y;
      case 2: return z;
    }
    BOOST_ASSERT(false);
  }

  std::string str() const {
    return (boost::format("<Point3D x/y/z %f/%f/%f") % x % y % z).str();
  }

  Point3D operator+(const Point3D& other) const {
    return Point3D{x + other.x, y + other.y, z + other.z};
  }

  Point3D operator-(const Point3D& other) const {
    return Point3D{x - other.x, y - other.y, z - other.z};
  }

  Point3D& operator+=(const Point3D& other) {
    *this = *this + other;
    return *this;
  }

  Point3D& operator-=(const Point3D& other) {
    *this = *this - other;
    return *this;
  }

  bool operator==(const Point3D& other) const {
    return x == other.x && y == other.y && z == other.z;
  }

  bool operator!=(const Point3D& other) const {
    return !(*this == other);
  }

  double length() const {
    return std::sqrt(length_squared());
  }

  double length_squared() const {
    return x * x + y * y + z * z;
  }

  Point3D scaled(double value) const {
    return Point3D{x * value, y * value, z * value};
  }

  double heading_deg() const {
    return Degrees(WrapNegPiToPi(0.5 * M_PI - std::atan2(y, x)));
  }

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(x));
    a->Visit(MJ_NVP(y));
    a->Visit(MJ_NVP(z));
  }
};
}
}
