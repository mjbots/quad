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

#include "gl/simple_texture_render_list.h"

#include <Eigen/Geometry>

namespace mjmech {
namespace gl {

namespace {
Eigen::Vector3f Transform(const Eigen::Matrix4f& matrix, const Eigen::Vector3f& p) {
  Eigen::Vector4f q(p.x(), p.y(), p.z(), 1.0);
  Eigen::Vector4f r = matrix * q;
  r = r / (r(3));
  return r.head<3>();
}

constexpr const char* kVertexShaderSource = R"XX(

#version 400

in vec3 inVertex;
in vec3 inNormal;
in vec2 inUv;
in vec4 inColor;
uniform mat4 projMatrix;
uniform mat4 viewMatrix;
uniform mat4 modelMatrix;
out vec2 fragUv;
out vec4 fragColor;
out vec3 fragNormal;
out vec3 fragPos;
void main(){
  fragUv = inUv;
  fragColor = inColor;
  fragNormal = inNormal;

  // Switch things to a right handed view coordinate system.
  vec4 vertex = vec4(inVertex.x, inVertex.y, -inVertex.z, 1.0);
  fragPos = vec3(viewMatrix * modelMatrix * vertex);
  gl_Position = projMatrix * viewMatrix * modelMatrix * vertex;
}

)XX";

constexpr const char* kFragmentShaderSource = R"XX(

#version 400
in vec2 fragUv;
in vec4 fragColor;
in vec3 fragNormal;
in vec3 fragPos;
uniform float ambient;
uniform vec3 lightPos;
uniform sampler2D currentTexture;
void main() {
  vec3 lightDir = normalize(lightPos - fragPos);
  float diff = max(dot(fragNormal, lightDir), 0);
  float light = min(diff + ambient, 1.0);
  vec4 lightModel = vec4(light * vec3(1.0, 1.0, 1.0), 1.0);
  vec4 texColor = texture(currentTexture, fragUv);
  if (texColor.a == 0.0)
    discard;
  gl_FragColor = lightModel * fragColor * texColor;
}

)XX";
}

SimpleTextureRenderList::SimpleTextureRenderList(Texture* texture)
    : texture_(texture),
      vertex_shader_(kVertexShaderSource, GL_VERTEX_SHADER),
      fragment_shader_(kFragmentShaderSource, GL_FRAGMENT_SHADER),
      program_(vertex_shader_, fragment_shader_) {
    program_.use();
    vao_.bind();

    vertices_.bind(GL_ARRAY_BUFFER);

    program_.VertexAttribPointer(
        program_.attribute("inVertex"), 3, GL_FLOAT, GL_FALSE, 48, 0);
    program_.VertexAttribPointer(
        program_.attribute("inNormal"), 3, GL_FLOAT, GL_FALSE, 48, 12);
    program_.VertexAttribPointer(
        program_.attribute("inUv"), 2, GL_FLOAT, GL_FALSE, 48, 24);
    program_.VertexAttribPointer(
        program_.attribute("inColor"), 4, GL_FLOAT, GL_FALSE, 48, 32);

    vao_.unbind();

    program_.SetUniform(program_.uniform("lightPos"),
                        Eigen::Vector3f({-1000, 0, -3000}));
    program_.SetUniform(program_.uniform("currentTexture"), 0);
    SetAmbient(0.3f);
}

void SimpleTextureRenderList::Reset() {
  data_.clear();
  indices_.clear();
}

void SimpleTextureRenderList::Upload() {
  vao_.bind();
  vertices_.set_vector(GL_ARRAY_BUFFER, data_, GL_STATIC_DRAW);
  elements_.set_vector(GL_ELEMENT_ARRAY_BUFFER, indices_, GL_STATIC_DRAW);
}

void SimpleTextureRenderList::SetAmbient(float value) {
    program_.SetUniform(program_.uniform("ambient"), value);
}

void SimpleTextureRenderList::SetProjMatrix(const Eigen::Matrix4f& matrix) {
  program_.use();
  program_.SetUniform(program_.uniform("projMatrix"), matrix);
}

void SimpleTextureRenderList::SetViewMatrix(const Eigen::Matrix4f& matrix) {
  program_.use();
  program_.SetUniform(program_.uniform("viewMatrix"), matrix);
}

void SimpleTextureRenderList::SetModelMatrix(const Eigen::Matrix4f& matrix) {
  program_.use();
  program_.SetUniform(program_.uniform("modelMatrix"), matrix);
}

void SimpleTextureRenderList::SetLightPos(const Eigen::Vector3f& position) {
  program_.use();
  program_.SetUniform(program_.uniform("lightPos"), position);
}

void SimpleTextureRenderList::Render() {
  program_.use();
  vao_.bind();
  texture_->bind(GL_TEXTURE_2D);
  glDrawElements(GL_TRIANGLES, indices_.size(), GL_UNSIGNED_INT, 0);
  vao_.unbind();
}

void SimpleTextureRenderList::SetTransform(const Eigen::Matrix4f& transform) {
    transform_ = transform;
}

uint32_t SimpleTextureRenderList::AddVertex(const Eigen::Vector3f& point_in,
                                            const Eigen::Vector3f& normal,
                                            const Eigen::Vector2f& uv,
                                            const Eigen::Vector4f& rgba) {
  Eigen::Vector3f p1 = Transform(transform_, point_in);
  auto& d = data_;
  const auto i = d.size();
  d.resize(i + 12);

  d[i + 0] = p1.x();
  d[i + 1] = p1.y();
  d[i + 2] = p1.z();
  d[i + 3] = normal.x();
  d[i + 4] = normal.y();
  d[i + 5] = normal.z();
  d[i + 6] = uv.x();
  d[i + 7] = uv.y();
  d[i + 8] = rgba(0);
  d[i + 9] = rgba(1);
  d[i + 10] = rgba(2);
  d[i + 11] = rgba(3);
  return i / 12;
}

void SimpleTextureRenderList::AddTriangle(uint32_t i1, uint32_t i2, uint32_t i3) {
  indices_.push_back(i1);
  indices_.push_back(i2);
  indices_.push_back(i3);
}

void SimpleTextureRenderList::AddTriangle(
    const Eigen::Vector3f& p1, const Eigen::Vector2f& uv1,
    const Eigen::Vector3f& p2, const Eigen::Vector2f& uv2,
    const Eigen::Vector3f& p3, const Eigen::Vector2f& uv3,
    const Eigen::Vector4f& rgba) {
  const Eigen::Vector3f normal = (p3 - p1).cross(p2 - p1).normalized();
  const auto i1 = AddVertex(p1, normal, uv1, rgba);
  const auto i2 = AddVertex(p2, normal, uv2, rgba);
  const auto i3 = AddVertex(p3, normal, uv3, rgba);
  AddTriangle(i1, i2, i3);
}

void SimpleTextureRenderList::AddQuad(
    uint32_t i1, uint32_t i2, uint32_t i3, uint32_t i4) {
  AddTriangle(i1, i2, i3);
  AddTriangle(i3, i4, i1);
}

void SimpleTextureRenderList::AddQuad(
    const Eigen::Vector3f& p1, const Eigen::Vector2f& uv1,
    const Eigen::Vector3f& p2, const Eigen::Vector2f& uv2,
    const Eigen::Vector3f& p3, const Eigen::Vector2f& uv3,
    const Eigen::Vector3f& p4, const Eigen::Vector2f& uv4,
    const Eigen::Vector4f& rgba) {
  const Eigen::Vector3f normal = (p3 - p1).cross(p2 - p1).normalized();
  const auto i1 = AddVertex(p1, normal, uv1, rgba);
  const auto i2 = AddVertex(p2, normal, uv2, rgba);
  const auto i3 = AddVertex(p3, normal, uv3, rgba);
  const auto i4 = AddVertex(p4, normal, uv4, rgba);
  AddQuad(i1, i2, i3, i4);
}

}
}
