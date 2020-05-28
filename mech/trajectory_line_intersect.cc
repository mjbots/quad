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

#include "mech/trajectory_line_intersect.h"

#include "base/common.h"

namespace mjmech {
namespace mech {

namespace {
double StraightLine(const Eigen::Vector2d& velocity,
                    const Eigen::Vector2d& p1,
                    const Eigen::Vector2d& p2) {
  // From https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection
  const double x1 = 0;
  const double y1 = 0;
  const double x2 = velocity.x();
  const double y2 = velocity.y();

  const double x3 = p1.x();
  const double y3 = p1.y();
  const double x4 = p2.x();
  const double y4 = p2.y();

  const double denom = ((x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4));
  if (denom == 0.0) {
    return std::numeric_limits<double>::infinity();
  }

  const double t =
      ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / denom;

  // 't' is in terms of the length of x1,y1 -> x2,y2 which is our
  // velocity.  Thus a t of 1 is 1 unit of time.
  return t;
}

double CurvedPath(const Eigen::Vector2d& velocity,
                  double omega,
                  const Eigen::Vector2d& p1_in,
                  const Eigen::Vector2d& p2_in) {
  // From: https://mathworld.wolfram.com/Circle-LineIntersection.html
  const double radius = velocity.norm() / omega;
  const Eigen::Vector2d center =
      (1.0 / omega) * ((Eigen::Matrix2d() << 0, -1, 1, 0).finished() * velocity);
  const Eigen::Vector2d p1 = p1_in - center;
  const Eigen::Vector2d p2 = p2_in - center;
  const double dx = p2.x() - p1.x();
  const double dy = p2.y() - p1.y();
  const double dr2 = dx * dx + dy * dy;

  const double D = p1.x() * p2.y() - p2.x() * p1.y();
  const double discriminant2 = radius * radius * dr2 - D * D;
  if (discriminant2 < 0.0) {
    return std::numeric_limits<double>::infinity();
  }

  auto calct = [&](const Eigen::Vector2d& p) {
    Eigen::Vector2d delta = p - center;
    const double theta0 = std::atan2(-center.y(), -center.x());
    const double theta = std::atan2(delta.y(), delta.x());
    return base::WrapNegPiToPi(theta - theta0) * radius / velocity.norm();
  };

  if (discriminant2 == 0.0) {
    const double x = D * dy / dr2 + center.x();
    const double y = -D * dx / dr2 + center.y();

    return calct({x, y});
  }
  const double discriminant = std::sqrt(discriminant2);

  auto sign = [](double value) {
    return value < 0.0 ? -1 : 1.0;
  };

  // There are two intersections.  Pick the one closest to our starting point.
  Eigen::Vector2d s1(
      (D * dy + sign(dy) * dx * discriminant) / dr2 + center.x(),
      (-D * dx + std::abs(dy) * discriminant) / dr2 + center.y());
  Eigen::Vector2d s2(
      (D * dy - sign(dy) * dx * discriminant) / dr2 + center.x(),
      (-D * dx - std::abs(dy) * discriminant) / dr2 + center.y());
  const double t1 = calct(s1);
  const double t2 = calct(s2);
  return (std::abs(t1) < std::abs(t2)) ? t1 : t2;
}
}

double TrajectoryLineIntersectTime(const Eigen::Vector2d& velocity,
                                   double omega,
                                   const Eigen::Vector2d& p1,
                                   const Eigen::Vector2d& p2) {
  // We handle the straight line case differently from the curved path
  // case.
  if (std::abs(omega) < 1e-6) {
    return StraightLine(velocity, p1, p2);
  } else {
    return CurvedPath(velocity, omega, p1, p2);
  }
}

}
}
