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

#pragma once

#include <string_view>

#include <fmt/format.h>

#include <imgui.h>

#include "mjlib/base/visitor.h"
#include "mjlib/base/visit_archive.h"

namespace mjmech {
namespace utils {

class ImGuiTreeArchive : public mjlib::base::VisitArchive<ImGuiTreeArchive> {
 public:
  ImGuiTreeArchive(const std::string& prefix = "") : prefix_(prefix) {}

  template <typename Serializable>
  ImGuiTreeArchive& Accept(const Serializable* serializable) {
    mjlib::base::VisitArchive<ImGuiTreeArchive>::Accept(
        const_cast<Serializable*>(serializable));
    return *this;
  }

  template <typename ValueType>
  ImGuiTreeArchive& Value(std::string_view name, const ValueType& value) {
    mjlib::base::ReferenceNameValuePair nvp(
        const_cast<ValueType*>(&value), name.data());
    mjlib::base::VisitArchive<ImGuiTreeArchive>::Visit(nvp);
    return *this;
  }

  template <typename NameValuePair>
  void VisitScalar(const NameValuePair& nvp) {
    const bool expanded =
        ImGui::TreeNodeEx(nvp.name(), ImGuiTreeNodeFlags_Leaf);

    if (ImGui::BeginDragDropSource()) {
      const std::string token = prefix_ + nvp.name();
      ImGui::SetDragDropPayload("DND_DERIV", token.data(), token.size());
      ImGui::TextUnformatted(token.c_str());
      ImGui::EndDragDropSource();
    }

    ImGui::NextColumn();
    ImGui::Text("%s", fmt::format("{}", nvp.get_value()).c_str());
    ImGui::NextColumn();

    if (expanded) {
      ImGui::TreePop();
    }
  }

  template <typename NameValuePair>
  void VisitSerializable(const NameValuePair& nvp) {
    const bool expanded = ImGui::TreeNode(nvp.name());
    ImGui::NextColumn();
    ImGui::NextColumn();

    if (expanded) {
      ImGuiTreeArchive sub_archive(prefix_ + nvp.name() + ".");
      sub_archive.Accept(nvp.value());
      ImGui::TreePop();
    }
  }

  template <typename NameValuePair>
  void VisitArray(const NameValuePair& nvp) {
    const bool expanded = ImGui::TreeNode(nvp.name());
    ImGui::NextColumn();
    ImGui::NextColumn();

    if (expanded) {
      const auto& value = *nvp.value();
      int i = 0;
      for (const auto& item : value) {
        auto istr = fmt::format("{}", i);
        ImGuiTreeArchive sub_archive (prefix_ + nvp.name() + ".");
        sub_archive.Value(istr, item);
        i++;
      }
      ImGui::TreePop();
    }
  }

 private:
  std::string prefix_;
};

}
}
