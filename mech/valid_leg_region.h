// Copyright 2019-2020 Josh Pieper, jjp@pobox.com.
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

#include <Eigen/Geometry>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>

#include "mech/ik.h"
#include "mech/trajectory_line_intersect.h"

namespace mjmech {
namespace mech {

class ValidLegRegion {
 public:
  static constexpr double kXStep = 0.01;  // 10mm
  static constexpr double kYStep = 0.01;  // 10mm

  using Slice = std::pair<Eigen::Vector3d, Eigen::Vector3d>;
  using Plane = std::vector<Slice>;
  using Point = boost::geometry::model::d2::point_xy<double>;
  using Polygon = boost::geometry::model::polygon<Point>;

  ValidLegRegion(const IkSolver& ik,
                 const base::Point3D& idle_G,
                 double lift_height) {
    // Our strategy is to move in small increments forward and
    // backward in X.  At each X value, we scan +-Y to find the
    // bounds.  These bounds are used to trace out a rough polygon.
    // This is repeated at the idle_R.z (the stand up height), and
    // again at the walking height.  After that, we try to find a
    // maximal aligned bounding box that fits within.

    std::vector<Polygon> poly_G;

    for (double z : { 0.0, lift_height }) {
      base::Point3D p_G = idle_G;
      p_G.z() = idle_G.z() - z;
      poly_G.push_back(SearchPlane(ik, p_G));
    }

    std::vector<Polygon> merged_G;
    boost::geometry::intersection(poly_G.front(), poly_G.back(), merged_G);
    if (merged_G.empty()) {
      return;
    }

    // TODO: Shrink this so we get margin.

    boost::geometry::simplify(merged_G.front(), bounds_G_, kXStep);
  }

  /// For a point at the given location, moving at the given velocity
  /// and that velocity rotating at the given omega, determine when it
  /// will leave the bounding region.
  ///
  /// Returns a negative value if it is outside the bounding region,
  /// and infinity (possibly negative) if it will never cross the
  /// bounding region.
  double TimeToLeave_G(const Eigen::Vector2d& point_G,
                       const Eigen::Vector2d& velocity,
                       double omega) const {
    const bool within = boost::geometry::within(
        Point(point_G.x(), point_G.y()), bounds_G_);

    // If we are already outside, then just report negative infinity.
    if (!within) {
      return !std::numeric_limits<double>::infinity();
    }

    if (velocity.norm() == 0.0 && omega == 0.0) {
      Point p_G{point_G.x(), point_G.y()};
      return std::numeric_limits<double>::infinity();
    }

    // Find the smallest non-negative value.
    namespace bg = boost::geometry;
    std::optional<double> smallest_time_s;
    bg::for_each_segment(bounds_G_, [&](const auto& segment) {
        // Transform each segment to be relative to point_G.
        const Eigen::Vector2d p1 =
            Eigen::Vector2d(bg::get<0, 0>(segment),
                            bg::get<0, 1>(segment)) - point_G;
        const Eigen::Vector2d p2 =
            Eigen::Vector2d(bg::get<1, 0>(segment),
                            bg::get<1, 1>(segment)) - point_G;
        const double this_s =
            TrajectoryLineIntersectTime(velocity, omega, p1, p2);
        if (this_s >= 0.0 &&
            (!smallest_time_s || this_s < *smallest_time_s)) {
          smallest_time_s = this_s;
        }
      });

    return smallest_time_s.value();
  }

 private:

  Polygon SearchPlane(const IkSolver& ik, const base::Point3D& start_G) const {
    Plane plane;

    for (double xdir : {-1.0, 1.0}) {
      base::Point3D cur_G = start_G;
      while (true) {
        cur_G.x() += kXStep * xdir;
        auto maybe_slice = FindSlice(ik, cur_G);
        if (!maybe_slice) {
          break;
        }
        plane.push_back(*maybe_slice);
      }
    }

    std::sort(plane.begin(), plane.end(),
              [](const auto& lhs, const auto& rhs) {
                return lhs.first.x() < rhs.first.x();
              });

    Polygon result;
    // We generate the polygon by walking up the right side, then down
    // the left side.
    for (const auto& slice : plane) {
      Point p{slice.second.x(), slice.second.y()};
      boost::geometry::append(result, p);
    }

    // Then back down the right side.
    for (auto it = plane.rbegin(); it != plane.rend(); ++it) {
      Point p{it->first.x(), it->first.y()};
      boost::geometry::append(result, p);
    }

    return result;
  }

  std::optional<Slice> FindSlice(
      const IkSolver& ik, const base::Point3D& start_G) const {
    auto scan_y_G = [&](double ydir) {
      base::Point3D cur_G = start_G;
      std::optional<base::Point3D> old_G;
      while (true) {
        IkSolver::Effector effector_G;
        effector_G.pose = cur_G;
        const auto maybe_result = ik.Inverse(effector_G, {});
        if (!maybe_result) {
          // We reached a point where the solution is no longer
          // valid.  Return the old point.
          return old_G;
        }
        old_G = cur_G;
        cur_G.y() += kYStep * ydir;
      }
    };

    auto maybe_first = scan_y_G(-1);
    auto maybe_second = scan_y_G(1);
    if (!maybe_first || !maybe_second) {
      return {};
    }
    return Slice{*maybe_first, *maybe_second};
  }

  Polygon bounds_G_;
};

}
}
