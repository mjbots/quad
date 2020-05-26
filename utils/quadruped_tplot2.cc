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

#include "utils/quadruped_tplot2.h"

#include "mjlib/telemetry/mapped_binary_reader.h"

#include "mech/quadruped_control.h"

#include "utils/tree_view.h"

namespace mjmech {
namespace utils {

namespace {
struct Force {
  Eigen::Vector3d force_N;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(force_N));
  }
};
}

void AddQuadrupedDerived(mjlib::telemetry::FileReader* file_reader,
                         TreeView* tree_view) {
  auto qc_reader = std::make_shared<mjlib::telemetry::MappedBinaryReader<
    mech::QuadrupedControl::ControlLog>>(
        file_reader->record("qc_control")->schema->root());
  auto qs_reader = std::make_shared<mjlib::telemetry::MappedBinaryReader<
    mech::QuadrupedControl::Status>>(
        file_reader->record("qc_status")->schema->root());
  tree_view->AddDerived(
      "control_CoM_N",
      [qc_reader](const CurrentLogData& data) {
        const auto maybe_d = data.get("qc_control");
        if (!maybe_d) { return Force(); }

        auto qc = qc_reader->Read(*maybe_d);
        Eigen::Vector3d result_N;
        for (const auto& leg_B : qc.legs_B) {
          if (leg_B.stance != 0.0) {
            result_N += leg_B.force_N;
          }
        }

        Force result;
        result.force_N = result_N;
        return result;
      });
  tree_view->AddDerived(
      "status_CoM_N",
      [qc_reader, qs_reader](const CurrentLogData& data) {
        const auto maybe_c = data.get("qc_control");
        const auto maybe_s = data.get("qc_status");
        if (!maybe_c || !maybe_s) { return Force(); }

        auto qc = qc_reader->Read(*maybe_c);
        auto qs = qs_reader->Read(*maybe_s);
        if (qc.legs_B.size() != qs.state.legs_B.size()) {
          return Force();
        }

        Eigen::Vector3d result_N;
        for (size_t i = 0; i < qs.state.legs_B.size(); i++) {
          if (qc.legs_B[i].stance == 0.0) { continue; }
          result_N += qs.state.legs_B[i].force_N;
        }

        Force result;
        result.force_N = result_N;
        return result;
      });
}

}
}
