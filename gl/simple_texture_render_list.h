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
#include "gl/texture.h"
#include "gl/vertex_array_object.h"
#include "gl/vertex_buffer_object.h"

namespace mjmech {
namespace gl {

/// Given a single texture, provide a mechanism for queuing up a
/// dynamic vertex list and rendering it.  It renders into a
/// right-handed coordinate system.
class SimpleTextureRenderList {
 public:
  SimpleTextureRenderList(gl::Texture* texture);

  /// Get ready for the next rendering frame.
  void Reset();

  /// Upload all geometry to the GPU.
  void Upload();

  /// Set the various uniforms.
  void SetAmbient(float value);
  void SetProjMatrix(const Eigen::Matrix4f&);
  void SetViewMatrix(const Eigen::Matrix4f&);
  void SetModelMatrix(const Eigen::Matrix4f&);
  void SetLightPos(const Eigen::Vector3f&);

  /// Draw all primitives currently uploaded to GPU.
  void Render();

  void SetTransform(const Eigen::Matrix4f&);

  /// Queue geometry to be rendered.
  uint32_t AddVertex(const Eigen::Vector3f& point,
                     const Eigen::Vector3f& normal,
                     const Eigen::Vector2f& uv,
                     const Eigen::Vector4f& rgba);

  void AddTriangle(uint32_t i1, uint32_t i2, uint32_t i3);

  void AddTriangle(const Eigen::Vector3f& p1, const Eigen::Vector2f& uv1,
                   const Eigen::Vector3f& p2, const Eigen::Vector2f& uv2,
                   const Eigen::Vector3f& p3, const Eigen::Vector2f& uv3,
                   const Eigen::Vector4f& rgba);

  void AddQuad(uint32_t i1, uint32_t i2, uint32_t i3, uint32_t i4);

  void AddQuad(const Eigen::Vector3f& p1, const Eigen::Vector2f& uv1,
               const Eigen::Vector3f& p2, const Eigen::Vector2f& uv2,
               const Eigen::Vector3f& p3, const Eigen::Vector2f& uv3,
               const Eigen::Vector3f& p4, const Eigen::Vector2f& uv4,
               const Eigen::Vector4f& rgba);

 private:
  Texture* const texture_;

  std::vector<float> data_;
  std::vector<uint32_t> indices_;

  Eigen::Matrix4f transform_{Eigen::Matrix4f::Identity()};

  Shader vertex_shader_;
  Shader fragment_shader_;
  Program program_;
  VertexArrayObject vao_;
  VertexBufferObject vertices_;
  VertexBufferObject elements_;
};

}
}
