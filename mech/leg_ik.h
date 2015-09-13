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

#include "base/common.h"
#include "base/error_code.h"
#include "base/tf.h"
#include "base/visitor.h"

namespace legtool {
struct JointAngles {
  struct Joint {
    int ident = 0;
    double angle_deg = 0;

    Joint() {}
    Joint(int ident, double angle_deg) : ident(ident), angle_deg(angle_deg) {}
    bool operator==(const Joint& rhs) {
      return ident == rhs.ident && angle_deg == rhs.angle_deg;
    }
  };

  std::vector<Joint> joints;

  static JointAngles Invalid() { return JointAngles(); }
  bool Valid() const { return !joints.empty(); }

  double GetLargestChangeDeg(const JointAngles& other) const {
    BOOST_ASSERT(joints.size() == other.joints.size());
    double result = 0.0;
    for (size_t i = 0; i < joints.size(); i++) {
      BOOST_ASSERT(joints[i].ident == other.joints[i].ident);
      result = std::max(
          result, std::abs(joints[i].angle_deg - other.joints[i].angle_deg));
    }
    return result;
  }
};

class IKSolver : boost::noncopyable {
 public:
  virtual ~IKSolver() {}

  /// Given a target end position in 3D coordinate space, return the
  /// required joint angles for a 3 degree of freedom leg.
  ///
  ///  +y is away from the shoulder
  ///  +x is clockwise from shoulder
  ///  +z is up
  ///
  /// If no solution is possible, return an object with NaN values.
  virtual JointAngles Solve(const Point3D&) const = 0;
};

/// Inverse kinematics solver for lizard style 3-dof legs.
class LizardIK : public IKSolver {
 public:
  struct Config {
    struct Joint {
      double min_deg = 0;
      double idle_deg = 0;
      double max_deg = 0;
      double length_mm = 0;
      double sign = 1;
      int ident = 0;

      template <typename Archive>
      void Serialize(Archive* a) {
        a->Visit(LT_NVP(min_deg));
        a->Visit(LT_NVP(idle_deg));
        a->Visit(LT_NVP(max_deg));
        a->Visit(LT_NVP(length_mm));
        a->Visit(LT_NVP(sign));
        a->Visit(LT_NVP(ident));
      }
    };

    Joint coxa;
    Joint femur;
    Joint tibia;

    double servo_speed_dps = 360.0;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(LT_NVP(coxa));
      a->Visit(LT_NVP(femur));
      a->Visit(LT_NVP(tibia));
      a->Visit(LT_NVP(servo_speed_dps));
    }
  };

  LizardIK(const Config& config)
  : config_(config) {
    if (config_.coxa.length_mm == 0.0 ||
        config_.femur.length_mm == 0.0 ||
        config_.tibia.length_mm == 0.0) {
      throw SystemError::einval("invalid lizard leg config");
    }
  }

  virtual JointAngles Solve(const Point3D& point_mm) const {
    // Solve for the coxa first, as it has only a single solution.
    const double coxa_deg = (config_.coxa.sign *
                             Degrees(std::atan2(point_mm.x, point_mm.y)) +
                             config_.coxa.idle_deg);

    if (coxa_deg < config_.coxa.min_deg ||
                   coxa_deg > config_.coxa.max_deg) {
      return JointAngles::Invalid();
    }

    // x-coordinate of femur/tibia pair after rotating to 0 coxa
    const double true_x = (std::sqrt(std::pow(point_mm.x, 2) +
                                     std::pow(point_mm.y, 2)) -
                           config_.coxa.length_mm);
    const double im = std::sqrt(std::pow(point_mm.z, 2) +
                                std::pow(true_x, 2));


    // The new femur/tibia pair makes a triangle where the 3rd side is
    // the hypotenuse of the right triangle composed of z and im, lets
    // call it c.
    //
    //           --\  femur
    //           |\ --
    //           | \   --
    //           |  --    |
    //          z|  im\   | tibia
    //           |     --\|
    //           ----------
    //            true_x
    //
    // im = math.sqrt(z ** 2 + true_x ** 2)
    //
    // Then, we can use the law of cosines to find the angle opposite
    // im, which is the angle between the femur and tibia.
    //
    // im ** 2 = a ** 2 + b ** 2 + 2 * a * b * cos(C)
    //
    // Solving for C yields:
    //
    //  C = acos((im ** 2 - a ** 2 - b ** 2) / (2 * a * b))

    const double tibia_cos =
        ((std::pow(im, 2) -
          std::pow(config_.tibia.length_mm, 2) -
          std::pow(config_.femur.length_mm, 2)) /
         (2 * config_.tibia.length_mm * config_.femur.length_mm));
    if (tibia_cos < -1.0 or tibia_cos > 1.0) {
      return JointAngles::Invalid();
    }

    // For our purposes, a 0 tibia angle should equate to a right angle
    // with the femur, so subtract off 90 degrees.
    const double tibia_deg =
        (config_.tibia.sign *
         Degrees(0.5 * M_PI - std::acos(tibia_cos)) +
         config_.tibia.idle_deg);

    if (tibia_deg < config_.tibia.min_deg ||
        tibia_deg > config_.tibia.max_deg) {
      return JointAngles::Invalid();
    }

    // To solve for the femur angle, we first get the angle opposite
    // true_x, then the angle opposite the tibia.
    const double true_x_deg = Degrees(std::atan2(true_x, -point_mm.z));

    // Then the angle opposite the tibia is also found the via the law
    // of cosines.
    //
    //  tibia ** 2 = femur ** 2 + im ** 2 + 2 * femur * im * cos(femur_im)
    //
    //  femur_im = acos ( (tibia ** 2 - im ** 2 - femur ** 2) /
    //                    (2 * femur * im) )

    const double femur_im_cos =
        -(std::pow(config_.tibia.length_mm, 2) -
          std::pow(config_.femur.length_mm, 2) -
          std::pow(im, 2)) / (2 * config_.femur.length_mm * im);
    if (femur_im_cos < -1.0 || femur_im_cos > 1.0) {
      return JointAngles::Invalid();
    }

    const double femur_im_deg = Degrees(std::acos(femur_im_cos));

    const double femur_deg =
        (config_.femur.sign * ((femur_im_deg + true_x_deg) - 90.0) +
         config_.femur.idle_deg);

    if (femur_deg < config_.femur.min_deg ||
        femur_deg > config_.femur.max_deg) {
      return JointAngles::Invalid();
    }

    JointAngles result;
    result.joints.emplace_back(config_.coxa.ident, coxa_deg);
    result.joints.emplace_back(config_.femur.ident, femur_deg);
    result.joints.emplace_back(config_.tibia.ident, tibia_deg);
    return result;
  }

  const Config& config() const { return config_; }

 private:
  Config config_;
};

/// Inverse kinematics for mammal style 3-dof legs.
///
/// The "idle" position has the leg plane being perfectly vertical
/// with the femur and tibia both pointing directly down for full
/// extension.
class MammalIK : public IKSolver {
 public:
  struct Config {
    /// In the "idle" position, where the femur center of rotation is
    /// relative to the shoulder center of rotation.  Since the leg
    /// plane must be vertical, the shoulder leg plane separation is
    /// purely the y component.
    Point3D femur_attachment_mm;

    struct Joint {
      double min_deg = 0;
      double idle_deg = 0;
      double max_deg = 0;
      double sign = 1;
      double length_mm = 0; // ignored for shoulder
      int ident = 0;

      template <typename Archive>
      void Serialize(Archive* a) {
        a->Visit(LT_NVP(min_deg));
        a->Visit(LT_NVP(idle_deg));
        a->Visit(LT_NVP(max_deg));
        a->Visit(LT_NVP(sign));
        a->Visit(LT_NVP(length_mm));
        a->Visit(LT_NVP(ident));
      }
    };

    Joint shoulder;
    Joint femur;
    Joint tibia;
    bool invert = false;

    double servo_speed_dps = 360.0;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(LT_NVP(femur_attachment_mm));
      a->Visit(LT_NVP(shoulder));
      a->Visit(LT_NVP(femur));
      a->Visit(LT_NVP(tibia));
      a->Visit(LT_NVP(invert));
      a->Visit(LT_NVP(servo_speed_dps));
    }
  };

  MammalIK(const Config& config)
  : config_(config) {
    // Make some basic sanity checks.
    if (config_.femur.length_mm == 0.0 ||
        config_.tibia.length_mm == 0.0) {
      throw SystemError::einval("invalid mammal leg config");
    }
  }

  virtual JointAngles Solve(const Point3D& point_mm) const {
    // Find the angle of the shoulder joint.  This will be tangent
    // angle between the point and the circle with radius
    // femur_attachment_mm.y.

    // From: http://mathworld.wolfram.com/CircleTangentLine.html
    const double x0 = -point_mm.y;
    const double y0 = -point_mm.z;
    const double r = config_.femur_attachment_mm.y;

    const double squared =
        std::pow(x0, 2) + std::pow(y0, 2) - std::pow(r, 2);
    if (squared <= 0.0) {
      // The point is inside our shoulder's rotating radius.  Clearly,
      // we cannot position ourselves here.
      return JointAngles::Invalid();
    }

    const double denom = (std::pow(x0, 2) + std::pow(y0, 2));
    const double subexp = y0 * std::sqrt(squared) / denom;

    auto lim = [](double value) {
      return std::max(-1.0, std::min(1.0, value));
    };
    const double t1 = std::acos(lim(-r * x0 / denom + subexp)); // and +-
    const double t2 = std::acos(lim(-r * x0 / denom - subexp)); // and +-

    // We only are about the tangent line which lies in the right
    // semi-plane.
    const double t0 = (std::abs(t1) <= 0.5 * M_PI) ? t1 : t2;

    // The sign is based on which quadrant the point is in relative to
    // the rightmost point of the circle.
    //
    //          positive   |    negative
    //          ------------------------
    //          negative   |    positive
    const double rx = point_mm.y - r;
    const double ry = point_mm.z;
    // TODO jpieper: I don't think this sign calculation is correct
    // when abs(y) < radius.
    const double sign = GetSign(-rx * ry);
    const double logical_shoulder_rad = sign * t0;
    const double shoulder_deg =
        config_.shoulder.sign * Degrees(logical_shoulder_rad) +
        config_.shoulder.idle_deg;

    if (shoulder_deg < config_.shoulder.min_deg ||
        shoulder_deg > config_.shoulder.max_deg) {
      return JointAngles::Invalid();
    }

    // Given this shoulder angle, find the center of the leg plane in
    // the joint reference system.
    const double leg_frame_y = r * std::cos(logical_shoulder_rad);
    const double leg_frame_z = r * std::sin(logical_shoulder_rad);


    // Now we project the femur attachment and the point into the leg
    // plane.
    //
    // TODO jpieper: What sign?
    const double femur_x = config_.femur_attachment_mm.x;
    const double femur_y = config_.femur_attachment_mm.z;

    const double point_x = point_mm.x; // TODO jpieper: What sign?
    // TODO jpieper: This sign correction is probably not correct for
    // all quadrants of operation.
    const double point_y =
        std::sqrt(std::pow(point_mm.y - leg_frame_y, 2) +
                  std::pow(point_mm.z - leg_frame_z, 2)) *
        GetSign(point_mm.z - leg_frame_z);

    const double prx = -(point_x - femur_x);
    const double pry = point_y - femur_y;

    // Now we have a simple triangle, with the femur attachment point
    // at the origin, and the two sides being the femur and tibia.
    //
    //                      O
    //                     /  <-- femur
    //                    /
    //                   /
    //                   -------P
    //
    //                      ^tibia
    //
    // We know the length of all three sides, which means we can find
    // all three angles.

    // Use the law of cosines to find the tibia angle first.
    const double op_sq = prx * prx + pry * pry;
    const double cos_tibia_sub =
        std::pow(config_.femur.length_mm, 2) +
        std::pow(config_.tibia.length_mm, 2) -
        op_sq;

    if (std::sqrt(op_sq) >
        (config_.femur.length_mm + config_.tibia.length_mm)) {
      return JointAngles::Invalid();
    }

    const double cos_tibiainv =
        cos_tibia_sub /
        (2 * config_.femur.length_mm * config_.tibia.length_mm);
    const double tibiainv_rad = std::acos(lim(cos_tibiainv));
    const double tibia_sign = config_.invert ? -1.0 : 1.0;
    const double logical_tibia_rad = tibia_sign * (M_PI - tibiainv_rad);

    const double tibia_deg =
        config_.tibia.sign * Degrees(logical_tibia_rad) +
        config_.tibia.idle_deg;
    if (tibia_deg < config_.tibia.min_deg ||
        tibia_deg > config_.tibia.max_deg) {
      return JointAngles::Invalid();
    }

    // Now we can solve for the femur.
    const double cos_femur_sub =
        op_sq + std::pow(config_.femur.length_mm, 2) -
        std::pow(config_.tibia.length_mm, 2);

    const double cos_femur_1 = cos_femur_sub /
        (2 * std::sqrt(op_sq ) * config_.femur.length_mm);
    const double femur_1_rad = std::acos(lim(cos_femur_1));

    const double femur_sign = config_.invert ? 1 : -1;
    const double femur_rad =
        (std::atan2(pry, prx) + femur_sign * femur_1_rad) + 0.5 * M_PI;

    const double femur_deg = config_.femur.sign * Degrees(femur_rad) +
        config_.femur.idle_deg;
    if (femur_deg < config_.femur.min_deg ||
        femur_deg > config_.femur.max_deg) {
      return JointAngles::Invalid();
    }

    JointAngles result;
    result.joints.emplace_back(config_.shoulder.ident, shoulder_deg);
    result.joints.emplace_back(config_.femur.ident, femur_deg);
    result.joints.emplace_back(config_.tibia.ident, tibia_deg);

    return result;
  }

  struct ForwardResult {
    Frame shoulder_frame;
    Frame shoulder_joint{Point3D(), Quaternion(), &shoulder_frame};
    Point3D shoulder;
    Frame femur_joint{Point3D(), Quaternion(), &shoulder_joint};
    Point3D femur;
    Frame tibia_joint{Point3D(), Quaternion(), &femur_joint};
    Point3D tibia;
    Frame end_frame{Point3D(), Quaternion(), &tibia_joint};
    Point3D end;

    ForwardResult() {}

    ForwardResult(const ForwardResult& rhs) {
      *this = rhs;
    }

    ForwardResult& operator=(const ForwardResult& rhs) {
      shoulder_frame.transform = rhs.shoulder_frame.transform;
      shoulder_joint.transform = rhs.shoulder_joint.transform;
      shoulder = rhs.shoulder;
      femur_joint.transform = rhs.femur_joint.transform;
      femur = rhs.femur;
      tibia_joint.transform = rhs.tibia_joint.transform;
      tibia = rhs.tibia;
      end_frame.transform = rhs.end_frame.transform;
      end = rhs.end;
      return *this;
    }
  };

  ForwardResult Forward(const JointAngles& joints) const {
    ForwardResult result;

    result.femur_joint.transform.translation = config_.femur_attachment_mm;
    result.tibia_joint.transform.translation.z = -config_.femur.length_mm;
    result.end_frame.transform.translation.z = -config_.tibia.length_mm;

    auto GetAngle = [](const JointAngles& joints, int ident) {
      for (const auto& joint: joints.joints) {
        if (joint.ident == ident) { return joint.angle_deg; }
      }
      BOOST_ASSERT(false);
    };

    const double shoulder_deg =
        config_.shoulder.sign * (GetAngle(joints, config_.shoulder.ident) -
                                 config_.shoulder.idle_deg);
    result.shoulder_joint.transform.rotation = Quaternion::FromEuler(
        0, Radians(shoulder_deg), 0);

    const double femur_deg =
        config_.femur.sign * (GetAngle(joints, config_.femur.ident) -
                              config_.femur.idle_deg);
    result.femur_joint.transform.rotation = Quaternion::FromEuler(
        Radians(femur_deg), 0, 0);

    const double tibia_deg =
        config_.tibia.sign * (GetAngle(joints, config_.tibia.ident) -
                              config_.tibia.idle_deg);
    result.tibia_joint.transform.rotation = Quaternion::FromEuler(
        Radians(tibia_deg), 0, 0);

    result.shoulder = result.shoulder_frame.MapFromFrame(
        &result.femur_joint, Point3D());
    result.femur = result.shoulder_frame.MapFromFrame(
        &result.tibia_joint, Point3D());
    result.end = result.shoulder_frame.MapFromFrame(
        &result.end_frame, Point3D());
    result.tibia = result.end;

    return result;
  }

  const Config& config() const { return config_; }

 private:
  Config config_;
};
}
