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

#include "gl/simple_line_render_list.h"

namespace mjmech {
namespace gl {

namespace {
Eigen::Vector3f Transform(const Eigen::Matrix4f& matrix, const Eigen::Vector3f& p) {
  Eigen::Vector4f q(p.x(), p.y(), p.z(), 1.0);
  Eigen::Vector4f r = matrix * q;
  r = r / (r(3));
  return r.head<3>();
}

constexpr const char* kVertexShaderSource =
      "#version 400\n"
      "in vec3 inVertex;\n"
      "in vec4 inColor;\n"
      "uniform mat4 projMatrix;\n"
      "uniform mat4 viewMatrix;\n"
      "uniform mat4 modelMatrix;\n"
      "out vec4 fragColor;\n"
      "void main() {\n"
      "  fragColor = inColor;\n"
      "  vec4 vertex = vec4(inVertex.x, inVertex.y, -inVertex.z, 1.0);\n"
      "  gl_Position = projMatrix * viewMatrix * modelMatrix * vertex;\n"
      "}\n"
      ;

constexpr const char* kFragShaderSource =
      "#version 400\n"
      "in vec4 fragColor;\n"
      "void main() {\n"
      "  gl_FragColor = fragColor;\n"
      "}\n"
      ;

}

SimpleLineRenderList::SimpleLineRenderList()
    : vertex_shader_{kVertexShaderSource, GL_VERTEX_SHADER},
      fragment_shader_{kFragShaderSource, GL_FRAGMENT_SHADER},
      program_{vertex_shader_, fragment_shader_} {
  program_.use();
  vao_.bind();

  vertices_.bind(GL_ARRAY_BUFFER);

  program_.VertexAttribPointer(
      program_.attribute("inVertex"), 3, GL_FLOAT, GL_FALSE, 28, 0);
  program_.VertexAttribPointer(
      program_.attribute("inColor"), 4, GL_FLOAT, GL_FALSE, 28, 12);

  vao_.unbind();
}

void SimpleLineRenderList::Reset() {
  data_.clear();
  indices_.clear();
}

void SimpleLineRenderList::Upload() {
  vao_.bind();
  vertices_.set_vector(GL_ARRAY_BUFFER, data_, GL_STATIC_DRAW);
  elements_.set_vector(GL_ELEMENT_ARRAY_BUFFER, indices_, GL_STATIC_DRAW);
}

void SimpleLineRenderList::SetProjMatrix(const Eigen::Matrix4f& matrix) {
  program_.use();
  program_.SetUniform(program_.uniform("projMatrix"), matrix);
}

void SimpleLineRenderList::SetViewMatrix(const Eigen::Matrix4f& matrix) {
  program_.use();
  program_.SetUniform(program_.uniform("viewMatrix"), matrix);
}

void SimpleLineRenderList::SetModelMatrix(const Eigen::Matrix4f& matrix) {
  program_.use();
  program_.SetUniform(program_.uniform("modelMatrix"), matrix);
}

void SimpleLineRenderList::Render() {
  program_.use();
  vao_.bind();

  glDrawElements(GL_LINES, indices_.size(), GL_UNSIGNED_INT, 0);
  vao_.unbind();
}

void SimpleLineRenderList::SetTransform(const Eigen::Matrix4f& transform) {
  transform_ = transform;
}

uint32_t SimpleLineRenderList::AddVertex(const Eigen::Vector3f& p1_in,
                                         const Eigen::Vector4f& rgba) {
  Eigen::Vector3f p1 = Transform(transform_, p1_in);
  auto& d = data_;
  const auto i = d.size();
  d.resize(i + 7);
  d[i + 0] = p1.x();
  d[i + 1] = p1.y();
  d[i + 2] = p1.z();
  d[i + 3] = rgba(0);
  d[i + 4] = rgba(1);
  d[i + 5] = rgba(2);
  d[i + 6] = rgba(3);
  return i / 7;
}

void SimpleLineRenderList::AddSegment(const Eigen::Vector3f& p1,
                                      const Eigen::Vector3f& p2,
                                      const Eigen::Vector4f& rgba) {
  auto index1 = AddVertex(p1, rgba);
  auto index2 = AddVertex(p2, rgba);
  auto& li = indices_;
  li.push_back(index1);
  li.push_back(index2);
}

}
}
