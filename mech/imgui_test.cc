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

#include <stdint.h>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavdevice/avdevice.h>
#include <libavformat/avformat.h>
#include <libswscale/swscale.h>
}

#include "imgui.h"
#include "examples/imgui_impl_glfw.h"
#include "examples/imgui_impl_opengl3.h"

#include <GL/gl3w.h>

#include <GLFW/glfw3.h>

#include <iostream>
#include <string>
#include <vector>

#include <Eigen/Core>

#include <boost/assert.hpp>

#include <fmt/format.h>

#define TRACE_GL_ERROR() { auto v = glGetError(); if (v != GL_NO_ERROR) { fmt::print("{}: GL ERR {}\n", __LINE__, static_cast<int>(v)); std::exit(1); } }

Eigen::Matrix4f Ortho(float left, float right, float bottom, float top,
                      float zNear, float zFar) {
  Eigen::Matrix4f result = Eigen::Matrix4f::Identity();
  result(0, 0) = 2.0f / (right - left);
  result(1, 1) = 2.0f / (top - bottom);
  result(2, 2) = 1.0f / (zFar - zNear);
  result(3, 0) = - (right + left) / (right - left);
  result(3, 1) = - (top + bottom) / (top - bottom);
  result(3, 2) = - zNear / (zFar - zNear);
  return result;
}

static void glfw_error_callback(int error, const char* description) {
  std::cerr << fmt::format("Glfw Error {}: {}\n", error, description);
}

const char* kVertShaderSource =
        "#version 150\n"
	"in vec3 vertex;\n"
	"in vec2 texCoord0;\n"
	"uniform mat4 mvpMatrix;\n"
	"out vec2 texCoord;\n"
	"void main() {\n"
	"	gl_Position = mvpMatrix * vec4(vertex, 1.0);\n"
	"	texCoord = texCoord0;\n"
	"}\n";

const char* kFragShaderSource =
  	"#version 150\n"
	"uniform sampler2D frameTex;\n"
	"in vec2 texCoord;\n"
	"out vec4 fragColor;\n"
	"void main() {\n"
	"	fragColor = texture(frameTex, texCoord);\n"
	"}\n";

class Shader {
 public:
  Shader(const char* source, GLenum type) {
    int size = std::strlen(source);
    shader_ = glCreateShader(type);
    glShaderSource(shader_, 1, (GLchar const **)&source, &size);
    glCompileShader(shader_);
    GLint status = -1;
    glGetShaderiv(shader_, GL_COMPILE_STATUS, &status);
    if (status != GL_TRUE) {
      fmt::print("error with shader\n");
      std::exit(1);
    }
  }

  ~Shader() {
    glDeleteShader(shader_);
  }

  int gl() const { return shader_; }

 private:
  GLuint shader_ = 0;
};

class Vao {
 public:
  Vao() {
    glGenVertexArrays(1, &vao_);
  }

  ~Vao() {
    glDeleteVertexArrays(1, &vao_);
  }

  void bind() {
    glBindVertexArray(vao_);
  }

  void unbind() {
    glBindVertexArray(0);
  }

 private:
  GLuint vao_ = 0;
};

class VertexBuffers {
 public:
  VertexBuffers() {
    glGenBuffers(1, &vbuf_);
  }

  ~VertexBuffers() {
    glDeleteBuffers(1, &vbuf_);
  }

  void bind(GLenum target) {
    glBindBuffer(target, vbuf_);
  }

 private:
  GLuint vbuf_ = 0;
};

class Program {
 public:
  Program(const Shader& vertex_shader, const Shader& fragment_shader) {
    program_ = glCreateProgram();
    glAttachShader(program_, vertex_shader.gl());
    glAttachShader(program_, fragment_shader.gl());
    glLinkProgram(program_);
  }

  ~Program() {
    glDeleteProgram(program_);
  }

  void SetUniform(const char* name, GLint value) {
    glUniform1i(uniform(name), value);
  }

  void SetUniform(const char* name, const Eigen::Matrix4f& matrix) {
    glUniformMatrix4fv(uniform(name), 1, GL_FALSE, matrix.data());
  }

  void use() {
    glUseProgram(program_);
  }

  GLuint uniform(const char* name) {
    return glGetUniformLocation(program_, name);
  }

  GLuint attrib(const char* name) {
    return glGetAttribLocation(program_, name);
  }

 private:
  GLuint program_ = 0;
};

class Texture {
 public:
  Texture() {
    glActiveTexture(GL_TEXTURE0);
    glGenTextures(1, &tex_);
  }

  ~Texture() {
    glDeleteTextures(1, &tex_);
  }

  void bind(GLenum target) {
    glBindTexture(target, tex_);
  }

 private:
  GLuint tex_ = 0;
};

void capture_init() {
  avdevice_register_all();
  avcodec_register_all();
  av_register_all();
}

void* BufferOffset(uint64_t value) {
  return reinterpret_cast<void*>(value);
}

int main(int, char**) {
  capture_init();

  glfwSetErrorCallback(glfw_error_callback);
  if (!glfwInit()) { return 1; }

  // GL 3.0 + GLSL 130
  const char* glsl_version = "#version 130";
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);

  GLFWwindow* window = glfwCreateWindow(1280, 720, "imgui test", nullptr, nullptr);
  if (window == nullptr) { return 1; }
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);

  bool err = gl3wInit() != 0;
  if (err) {
    std::cerr << "Failed to initialize gl3w\n";
    return 1;
  }

  AVInputFormat* input_format = av_find_input_format("v4l2");
  BOOST_ASSERT(input_format);
  AVDictionary* options = nullptr;
  av_dict_set(&options, "framerate", "10", 0);
  av_dict_set(&options, "video_size", "960x720", 0);

  AVFormatContext* av_format_context = nullptr;
  int ret = 0;
  if ((ret = avformat_open_input(
           &av_format_context, "/dev/video0", input_format, &options)) < 0) {
    fmt::print("could not open video source: {}\n", av_err2str(ret));
    return 1;
  }

  if ((ret = avformat_find_stream_info(av_format_context, nullptr)) < 0) {
    fmt::print("could not find stream info: {}\n", av_err2str(ret));
    return 1;
  }

  const int video_stream = [&]() {
    for (int i = 0; i < av_format_context->nb_streams; i++) {
      if (av_format_context->streams[i]->codecpar->codec_type == AVMEDIA_TYPE_VIDEO) {
        return i;
      }
    }
    fmt::print("could not find video stream\n");
    std::exit(1);
  }();

  const auto codecpar = av_format_context->streams[video_stream]->codecpar;

  const auto av_codec = avcodec_find_decoder(codecpar->codec_id);
  if (av_codec == nullptr) {
    fmt::print("unsupported codec\n");
    return 1;
  }

  auto* avcodec_context = avcodec_alloc_context3(av_codec);
  BOOST_ASSERT(avcodec_context);

  avcodec_context->width = 960;
  avcodec_context->height = 720;
  avcodec_context->framerate = (AVRational){10, 1};
  avcodec_context->pix_fmt = AV_PIX_FMT_YUYV422; // TODO: this needs to come from somewhere!

  if ((ret = avcodec_open2(avcodec_context, av_codec, nullptr)) < 0) {
    fmt::print("could not open codec {}\n", av_err2str(ret));
    return 1;
  }

  auto* frame = av_frame_alloc();
  auto* gl_frame = av_frame_alloc();

  int size = avpicture_get_size(AV_PIX_FMT_RGB24, codecpar->width, codecpar->height);
  auto* internal_buffer = (uint8_t*)av_malloc(size);
  avpicture_fill((AVPicture*)gl_frame, internal_buffer, AV_PIX_FMT_RGB24,
                 codecpar->width, codecpar->height);

  auto* sws_ctx = sws_getContext(
      codecpar->width, codecpar->height, avcodec_context->pix_fmt,
      codecpar->width, codecpar->height, AV_PIX_FMT_RGB24, SWS_BICUBIC,
      nullptr, nullptr, nullptr);

  AVPacket av_packet;
  av_init_packet(&av_packet);


  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGuiIO& io = ImGui::GetIO();  (void)io;

  ImGui::StyleColorsDark();

  ImGui_ImplGlfw_InitForOpenGL(window, true);
  ImGui_ImplOpenGL3_Init(glsl_version);

  TRACE_GL_ERROR();

  // ******* SHADERS **********
  auto v_shader = Shader(kVertShaderSource, GL_VERTEX_SHADER);
  auto f_shader = Shader(kFragShaderSource, GL_FRAGMENT_SHADER);
  auto program = Program(v_shader, f_shader);
  program.use();

  TRACE_GL_ERROR();

  // ******* VAO *********
  auto vao = Vao();
  vao.bind();

  auto vert_buf = VertexBuffers();
  vert_buf.bind(GL_ARRAY_BUFFER);
  float quad[20] = {
    -1.0f,  1.0f, 0.0f, 0.0f, 0.0f,
    -1.0f, -1.0f, 0.0f, 0.0f, 1.0f,
    1.0f, -1.0f, 0.0f, 1.0f, 1.0f,
    1.0f,  1.0f, 0.0f, 1.0f, 0.0f
  };
  glBufferData(GL_ARRAY_BUFFER, sizeof(quad), quad, GL_STATIC_DRAW);
  const auto vertex_id = program.attrib("vertex");
  glVertexAttribPointer(vertex_id, 3, GL_FLOAT, GL_FALSE, 20, BufferOffset(0));
  glEnableVertexAttribArray(vertex_id);
  const auto tex_coords_id = program.attrib("texCoord0");
  glVertexAttribPointer(tex_coords_id, 2, GL_FLOAT, GL_FALSE, 20, BufferOffset(12));
  glEnableVertexAttribArray(tex_coords_id);

  auto elem_buf = VertexBuffers();
  elem_buf.bind(GL_ELEMENT_ARRAY_BUFFER);
  unsigned char elem[6] = {
    0, 1, 2,
    0, 2, 3,
  };
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(elem), elem, GL_STATIC_DRAW);

  vao.unbind();

  TRACE_GL_ERROR();

  // ********* Texture ****************
  auto texture = Texture();
  TRACE_GL_ERROR();
  texture.bind(GL_TEXTURE_2D);
  TRACE_GL_ERROR();
  glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
  TRACE_GL_ERROR();
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

  TRACE_GL_ERROR();
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB,
               avcodec_context->width, avcodec_context->height,
               0, GL_RGB, GL_UNSIGNED_BYTE, NULL);
  TRACE_GL_ERROR();
  program.SetUniform("frameTex", 0);

  TRACE_GL_ERROR();
  program.SetUniform("mvpMatrix", Ortho(-1, 1, -1, 1, -1, 1));
  TRACE_GL_ERROR();

  bool show_demo_window = true;
  bool show_another_window = false;
  ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);

  TRACE_GL_ERROR();

  while (!glfwWindowShouldClose(window)) {
    glfwPollEvents();

    // Start the Dear ImGui frame
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    if (show_demo_window) {
      ImGui::ShowDemoWindow(&show_demo_window);
    }

    {
      static float f = 0.0f;
      static int counter = 0;

      ImGui::Begin("Hello, world!");                          // Create a window called "Hello, world!" and append into it.

      ImGui::Text("This is some useful text.");               // Display some text (you can use a format strings too)
      ImGui::Checkbox("Demo Window", &show_demo_window);      // Edit bools storing our window open/close state
      ImGui::Checkbox("Another Window", &show_another_window);

      ImGui::SliderFloat("float", &f, 0.0f, 1.0f);            // Edit 1 float using a slider from 0.0f to 1.0f
      ImGui::ColorEdit3("clear color", (float*)&clear_color); // Edit 3 floats representing a color

      if (ImGui::Button("Button"))                            // Buttons return true when clicked (most widgets return true when edited/activated)
        counter++;
      ImGui::SameLine();
      ImGui::Text("counter = %d", counter);

      ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
      ImGui::End();
    }

    TRACE_GL_ERROR();

    if (show_another_window)
    {
      ImGui::Begin("Another Window", &show_another_window);   // Pass a pointer to our bool variable (the window will have a closing button that will clear the bool when clicked)
      ImGui::Text("Hello from another window!");
      if (ImGui::Button("Close Me"))
        show_another_window = false;
      ImGui::End();
    }

    if ((ret = av_read_frame(av_format_context, &av_packet)) < 0) {
      std::cerr << "error reading frame: " << av_err2str(ret) << "\n";
      return 1;
    }
    if (av_packet.stream_index == video_stream) {
      if ((ret = avcodec_send_packet(avcodec_context, &av_packet)) < 0) {
        fmt::print("error sending frame {}\n", av_err2str(ret));
        std::exit(1);
      }
      if ((ret = avcodec_receive_frame(avcodec_context, frame)) < 0) {
        if (ret == AVERROR(EAGAIN)) {
          // nothing to do.
        } else {
          fmt::print("error receiving frame {}\n", av_err2str(ret));
          std::exit(1);
        }
      } else {
        // We got a frame!
        sws_scale(sws_ctx, frame->data, frame->linesize, 0,
                  codecpar->height,
                  gl_frame->data, gl_frame->linesize);

        texture.bind(GL_TEXTURE_2D);
        glTexSubImage2D(
            GL_TEXTURE_2D, 0, 0, 0,
            avcodec_context->width, avcodec_context->height,
            GL_RGB, GL_UNSIGNED_BYTE,
            gl_frame->data[0]);
      }
      av_free_packet(&av_packet);
      av_frame_unref(frame);
    }
    av_packet_unref(&av_packet);

    TRACE_GL_ERROR();

    // Rendering
    ImGui::Render();
    int display_w, display_h;
    glfwGetFramebufferSize(window, &display_w, &display_h);
    glViewport(0, 0, display_w, display_h);
    glClearColor(clear_color.x, clear_color.y, clear_color.z, clear_color.w);
    glClear(GL_COLOR_BUFFER_BIT);

    TRACE_GL_ERROR();

    program.use();

    TRACE_GL_ERROR();

    texture.bind(GL_TEXTURE_2D);
    vao.bind();
    glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_BYTE, BufferOffset(0));
    vao.unbind();

    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

    glfwSwapBuffers(window);
  }

  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImGui::DestroyContext();

  glfwDestroyWindow(window);
  glfwTerminate();
}
