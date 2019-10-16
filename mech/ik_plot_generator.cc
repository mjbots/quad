// Copyright 2014-2019 Josh Pieper, jjp@pobox.com.  All rights reserved.
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

#include <boost/program_options.hpp>

#include <fmt/format.h>

#include "mech/mammal_ik.h"
#include "mech/leg_ik.h"

namespace po = boost::program_options;
using namespace mjmech::mech;

namespace {
template <typename Array>
std::string FormatList(const Array& array) {
  std::ostringstream ostr;
  bool first = true;
  for (auto x : array) {
    if (!first) { ostr << " "; }
    ostr << x;
    first = false;
  }
  return ostr.str();
}

std::vector<double> Range(double start, double end, double step) {
  std::vector<double> result;
  for (double v = start; v <= end; v += step) {
    result.push_back(v);
  }
  return result;
}

IkSolver::Joint GetJoint(const IkSolver::JointAngles& joints, int id) {
  for (const auto& joint : joints) {
    if (joint.id == id) { return joint; }
  }
  mjlib::base::Fail("joint not found");
}

IkSolver::Joint Shoulder(const IkSolver::JointAngles& joints) {
  return GetJoint(joints, 1);
}

IkSolver::Joint Femur(const IkSolver::JointAngles& joints) {
  return GetJoint(joints, 2);
}

IkSolver::Joint Tibia(const IkSolver::JointAngles& joints) {
  return GetJoint(joints, 3);
}


JointAngles::Joint GetJoint(const JointAngles& joints, int id) {
  for (const auto& joint : joints.joints) {
    if (joint.ident == id) { return joint; }
  }
  mjlib::base::Fail("joint not found");
}

JointAngles::Joint Shoulder(const JointAngles& joints) {
  return GetJoint(joints, 1);
}

JointAngles::Joint Femur(const JointAngles& joints) {
  return GetJoint(joints, 2);
}

JointAngles::Joint Tibia(const JointAngles& joints) {
  return GetJoint(joints, 3);
}
}

int main(int argc, char** argv) {
  po::options_description desc;

  bool use_new = false;
  desc.add_options()
      ("help,h", "display usage")
      ("use-new", po::bool_switch(&use_new), "use new IK")
      ;

  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(desc).run(), vm);
  po::notify(vm);

  if (vm.count("help")) {
    std::cout << desc;
    return 0;
  }

  std::vector<double> xvals = Range(-40, 40, 0.5);
  std::vector<double> yvals = Range(-40, 40, 0.5);

  std::cout << FormatList(xvals) << "\n";
  std::cout << FormatList(yvals) << "\n";

  MammalIk ik_new{[&]() {
      MammalIk::Config config;

      config.shoulder.pose_mm = { 0.0, 10.0, 20.0 };
      config.shoulder.id = 1;
      config.femur.pose_mm = {0., 0., 100.};
      config.femur.id = 2;
      config.tibia.pose_mm = {0., 0., 100.};
      config.tibia.id = 3;

      return config;
    }()};

  MammalIK ik_old{[&]() {
      MammalIK::Config config;

      config.femur_attachment_mm = { 0.0, 10.0, -20.0 };
      config.shoulder.ident = 1;
      config.shoulder.min_deg = -180;
      config.shoulder.max_deg = 180;
      config.femur.length_mm = 100.0;
      config.femur.ident = 2;
      config.femur.min_deg = -180;
      config.femur.max_deg = 180;
      config.tibia.length_mm = 100.0;
      config.tibia.ident = 3;
      config.tibia.min_deg = -180;
      config.tibia.max_deg = 180;
      config.invert = true;

      return config;
    }()};

  for (auto x : xvals) {
    for (auto y : yvals) {
      if (use_new) {
        IkSolver::Effector input;
        input.pose_mm_G = { x, y, 160 };
        input.force_N_G = { 0, 0, 10.0 };
        const auto result = ik_new.Inverse(input, {});

        BOOST_ASSERT(!!result);

        std::cout << fmt::format(
            "{} {} {} {} {} {}\n",
            Shoulder(*result).torque_Nm,
            Femur(*result).torque_Nm,
            Tibia(*result).torque_Nm,
            Shoulder(*result).angle_deg,
            Femur(*result).angle_deg,
            Tibia(*result).angle_deg);
      } else {
        const auto result = ik_old.Solve({-x, y, -160}, {0, 0, -10.0});

        std::cout << fmt::format(
            "{} {} {} {} {} {}\n",
            -Shoulder(result).torque_Nm,
            Femur(result).torque_Nm,
            Tibia(result).torque_Nm,
            Shoulder(result).angle_deg,
            Femur(result).angle_deg,
            Tibia(result).angle_deg);
      }
    }
  }

  return 0;
}
