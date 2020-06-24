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

#include <vector>

#include <Eigen/Core>

#include "gl/program.h"
#include "gl/shader.h"
#include "gl/vertex_array_object.h"
#include "gl/vertex_buffer_object.h"

namespace mjmech {
namespace gl {

class SimpleLineRenderList {
 public:
  SimpleLineRenderList();

  /// Get ready for the next rendering frame.
  void Reset();

  /// Upload all geometry to the GPU.
  void Upload();

  void SetProjMatrix(const Eigen::Matrix4f&);
  void SetViewMatrix(const Eigen::Matrix4f&);
  void SetModelMatrix(const Eigen::Matrix4f&);

  /// Draw all primitives currently uploaded to GPU.
  void Render();

  void SetTransform(const Eigen::Matrix4f&);

  uint32_t AddVertex(const Eigen::Vector3f&,
                     const Eigen::Vector4f& rgba);
  void AddSegment(const Eigen::Vector3f&,
                  const Eigen::Vector3f&,
                  const Eigen::Vector4f& rgba);

 private:
  std::vector<float> data_;
  std::vector<uint32_t> indices_;

  Shader vertex_shader_;
  Shader fragment_shader_;
  Program program_;

  VertexArrayObject vao_;
  VertexBufferObject vertices_;
  VertexBufferObject elements_;

  Eigen::Matrix4f transform_{Eigen::Matrix4f::Identity()};
};

}
}
