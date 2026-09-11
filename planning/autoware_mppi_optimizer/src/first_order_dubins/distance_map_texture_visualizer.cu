// Copyright 2026 TIER IV, Inc.
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

#include <mppi/cost_functions/dubins/distance_map_texture.cuh>
#include <mppi/utils/gpu_err_chk.cuh>

#define GLFW_INCLUDE_NONE
#include <GL/gl.h>
#include <GLFW/glfw3.h>
#include <cuda_gl_interop.h>

#include <algorithm>
#include <array>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <string>

namespace
{
constexpr float kDistanceMapDebugRange = 10.0F;

std::mutex & visualizerInstanceMutex()
{
  static std::mutex mutex;
  return mutex;
}

std::mutex & glfwMutex()
{
  static std::mutex mutex;
  return mutex;
}

int & glfwUserCount()
{
  static int count = 0;
  return count;
}

bool acquireGlfw()
{
  std::lock_guard<std::mutex> lock(glfwMutex());
  if (glfwUserCount() == 0 && glfwInit() != GLFW_TRUE) {
    return false;
  }
  ++glfwUserCount();
  return true;
}

void releaseGlfw()
{
  std::lock_guard<std::mutex> lock(glfwMutex());
  --glfwUserCount();
  if (glfwUserCount() == 0) {
    glfwTerminate();
  }
}

__device__ inline uchar4 distanceMapDebugColor(
  const float distance, const float safe_margin, const float resolution, const bool valid)
{
  if (!valid) {
    return make_uchar4(45U, 45U, 45U, 255U);
  }
  if (!isfinite(distance)) {
    return make_uchar4(255U, 0U, 255U, 255U);
  }

  const float color_margin = fmaxf(safe_margin, resolution);
  const float clamped_distance = fmaxf(distance, 0.0F);
  if (clamped_distance <= color_margin) {
    const float margin_fraction = clamped_distance / color_margin;
    return make_uchar4(255U, static_cast<unsigned char>(255.0F * margin_fraction), 0U, 255U);
  }

  const float display_range = fmaxf(kDistanceMapDebugRange, color_margin + resolution);
  const float field_fraction =
    fminf((clamped_distance - color_margin) / (display_range - color_margin), 1.0F);
  if (field_fraction <= 0.5F) {
    const float transition = 2.0F * field_fraction;
    return make_uchar4(static_cast<unsigned char>(255.0F * (1.0F - transition)), 255U, 0U, 255U);
  }

  const float transition = 2.0F * field_fraction - 1.0F;
  return make_uchar4(
    0U, static_cast<unsigned char>(255.0F - 175.0F * transition),
    static_cast<unsigned char>(255.0F * transition), 255U);
}

__global__ void colorizeStaticDistanceMapKernel(
  const cudaTextureObject_t input, const cudaSurfaceObject_t output, const int width,
  const int height, const int channel, const float safe_margin, const float resolution,
  const bool valid)
{
  for (int y = static_cast<int>(blockIdx.y * blockDim.y + threadIdx.y); y < height;
       y += static_cast<int>(blockDim.y * gridDim.y)) {
    for (int x = static_cast<int>(blockIdx.x * blockDim.x + threadIdx.x); x < width;
         x += static_cast<int>(blockDim.x * gridDim.x)) {
      float distance = 0.0F;
      if (valid) {
        const float2 distances =
          tex2D<float2>(input, static_cast<float>(x) + 0.5F, static_cast<float>(y) + 0.5F);
        distance = channel == 0 ? distances.x : distances.y;
      }
      surf2Dwrite(
        distanceMapDebugColor(distance, safe_margin, resolution, valid), output,
        x * static_cast<int>(sizeof(uchar4)), y);
    }
  }
}

__global__ void colorizeObstacleDistanceMapKernel(
  const cudaTextureObject_t input, const cudaSurfaceObject_t output, const int width,
  const int height, const int timestep, const float safe_margin, const float resolution,
  const bool valid, const bool has_obstacles)
{
  for (int y = static_cast<int>(blockIdx.y * blockDim.y + threadIdx.y); y < height;
       y += static_cast<int>(blockDim.y * gridDim.y)) {
    for (int x = static_cast<int>(blockIdx.x * blockDim.x + threadIdx.x); x < width;
         x += static_cast<int>(blockDim.x * gridDim.x)) {
      float distance = kDistanceMapEmptyDistance;
      if (valid && has_obstacles) {
        distance = tex3D<float>(
          input, static_cast<float>(x) + 0.5F, static_cast<float>(y) + 0.5F,
          static_cast<float>(timestep) + 0.5F);
      }
      surf2Dwrite(
        distanceMapDebugColor(distance, safe_margin, resolution, valid), output,
        x * static_cast<int>(sizeof(uchar4)), y);
    }
  }
}
}  // namespace

class DistanceMapTextureVisualizer
{
public:
  DistanceMapTextureVisualizer(
    const int static_width, const int static_height, const int obstacle_width,
    const int obstacle_height, const int time_steps)
  : static_width_(static_width),
    static_height_(static_height),
    obstacle_width_(obstacle_width),
    obstacle_height_(obstacle_height),
    time_steps_(std::max(1, time_steps))
  {
    if (!acquireGlfw()) {
      throw std::runtime_error("GLFW initialization failed (is a graphical display available?)");
    }
    glfw_acquired_ = true;
    try {
      glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
      glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);
      window_ =
        glfwCreateWindow(kWindowWidth, kWindowHeight, "MPPI distance textures", nullptr, nullptr);
      if (window_ == nullptr) {
        throw std::runtime_error("failed to create the OpenGL distance-map debug window");
      }
      glfwMakeContextCurrent(window_);
      glfwSwapInterval(0);
      glfwSetWindowUserPointer(window_, this);
      glfwSetMouseButtonCallback(window_, mouseButtonCallback);
      glfwSetCursorPosCallback(window_, cursorPositionCallback);
      glfwSetKeyCallback(window_, keyCallback);

      HANDLE_ERROR(cudaStreamCreateWithFlags(&visualization_stream_, cudaStreamNonBlocking));
      HANDLE_ERROR(cudaEventCreateWithFlags(&distance_maps_ready_event_, cudaEventDisableTiming));
      createInteropTexture(0, static_width_, static_height_);
      createInteropTexture(1, static_width_, static_height_);
      createInteropTexture(2, obstacle_width_, obstacle_height_);
      glfwMakeContextCurrent(nullptr);
    } catch (...) {
      release();
      throw;
    }
  }

  ~DistanceMapTextureVisualizer() { release(); }

  DistanceMapTextureVisualizer(const DistanceMapTextureVisualizer &) = delete;
  DistanceMapTextureVisualizer & operator=(const DistanceMapTextureVisualizer &) = delete;

  bool render(
    const cudaTextureObject_t static_texture, const cudaTextureObject_t obstacle_texture,
    const bool road_border_valid, const bool drivable_area_valid, const bool obstacle_valid,
    const bool obstacle_has_obstacles, const float road_border_margin,
    const float drivable_area_margin, const float obstacle_margin, const float static_resolution,
    const float obstacle_resolution, const cudaStream_t producer_stream, const bool update_static)
  {
    glfwMakeContextCurrent(window_);
    glfwPollEvents();
    if (glfwWindowShouldClose(window_) == GLFW_TRUE) {
      return false;
    }

    HANDLE_ERROR(cudaEventRecord(distance_maps_ready_event_, producer_stream));
    HANDLE_ERROR(cudaStreamWaitEvent(visualization_stream_, distance_maps_ready_event_, 0));

    std::array<cudaGraphicsResource_t, 3> mapped_resources{};
    int mapped_count = 0;
    if (update_static) {
      mapped_resources[static_cast<size_t>(mapped_count++)] = cuda_resources_[0];
      mapped_resources[static_cast<size_t>(mapped_count++)] = cuda_resources_[1];
    }
    mapped_resources[static_cast<size_t>(mapped_count++)] = cuda_resources_[2];
    HANDLE_ERROR(
      cudaGraphicsMapResources(mapped_count, mapped_resources.data(), visualization_stream_));

    std::array<cudaSurfaceObject_t, 3> output_surfaces{};
    if (update_static) {
      output_surfaces[0] = mappedSurface(cuda_resources_[0]);
      output_surfaces[1] = mappedSurface(cuda_resources_[1]);
    }
    output_surfaces[2] = mappedSurface(cuda_resources_[2]);

    const dim3 block(16, 16, 1);
    if (update_static) {
      const dim3 static_grid(
        (static_width_ + static_cast<int>(block.x) - 1) / static_cast<int>(block.x),
        (static_height_ + static_cast<int>(block.y) - 1) / static_cast<int>(block.y), 1);
      colorizeStaticDistanceMapKernel<<<static_grid, block, 0, visualization_stream_>>>(
        static_texture, output_surfaces[0], static_width_, static_height_, 0, road_border_margin,
        static_resolution, road_border_valid);
      HANDLE_ERROR(cudaGetLastError());
      colorizeStaticDistanceMapKernel<<<static_grid, block, 0, visualization_stream_>>>(
        static_texture, output_surfaces[1], static_width_, static_height_, 1, drivable_area_margin,
        static_resolution, drivable_area_valid);
      HANDLE_ERROR(cudaGetLastError());
    }

    const dim3 obstacle_grid(
      (obstacle_width_ + static_cast<int>(block.x) - 1) / static_cast<int>(block.x),
      (obstacle_height_ + static_cast<int>(block.y) - 1) / static_cast<int>(block.y), 1);
    colorizeObstacleDistanceMapKernel<<<obstacle_grid, block, 0, visualization_stream_>>>(
      obstacle_texture, output_surfaces[2], obstacle_width_, obstacle_height_, selected_timestep_,
      obstacle_margin, obstacle_resolution, obstacle_valid, obstacle_has_obstacles);
    HANDLE_ERROR(cudaGetLastError());

    HANDLE_ERROR(cudaStreamSynchronize(visualization_stream_));
    for (cudaSurfaceObject_t & surface : output_surfaces) {
      if (surface != 0) {
        HANDLE_ERROR(cudaDestroySurfaceObject(surface));
        surface = 0;
      }
    }
    HANDLE_ERROR(
      cudaGraphicsUnmapResources(mapped_count, mapped_resources.data(), visualization_stream_));
    HANDLE_ERROR(cudaStreamSynchronize(visualization_stream_));

    drawWindow();
    glfwMakeContextCurrent(nullptr);
    return true;
  }

private:
  static constexpr int kWindowWidth = 1500;
  static constexpr int kWindowHeight = 560;
  static constexpr float kProjectionWidth = 3.0F;
  static constexpr float kProjectionHeight = 1.15F;
  static constexpr float kSliderLeft = 2.05F;
  static constexpr float kSliderRight = 2.95F;

  void createInteropTexture(const size_t index, const int width, const int height)
  {
    glGenTextures(1, &gl_textures_[index]);
    glBindTexture(GL_TEXTURE_2D, gl_textures_[index]);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
    glFinish();
    if (glGetError() != GL_NO_ERROR) {
      throw std::runtime_error("failed to allocate an OpenGL distance-map texture");
    }
    constexpr unsigned int flags =
      cudaGraphicsRegisterFlagsWriteDiscard | cudaGraphicsRegisterFlagsSurfaceLoadStore;
    const cudaError_t err = cudaGraphicsGLRegisterImage(
      &cuda_resources_[index], gl_textures_[index], GL_TEXTURE_2D, flags);
    if (err != cudaSuccess) {
      throw std::runtime_error(
        std::string("cudaGraphicsGLRegisterImage failed: ") + cudaGetErrorString(err));
    }
  }

  cudaSurfaceObject_t mappedSurface(cudaGraphicsResource_t resource) const
  {
    cudaArray_t mapped_array = nullptr;
    HANDLE_ERROR(cudaGraphicsSubResourceGetMappedArray(&mapped_array, resource, 0, 0));
    cudaResourceDesc resource_desc{};
    resource_desc.resType = cudaResourceTypeArray;
    resource_desc.res.array.array = mapped_array;
    cudaSurfaceObject_t surface = 0;
    HANDLE_ERROR(cudaCreateSurfaceObject(&surface, &resource_desc));
    return surface;
  }

  void drawTexturedPanel(const GLuint texture, const float left, const float right) const
  {
    glBindTexture(GL_TEXTURE_2D, texture);
    glBegin(GL_QUADS);
    glTexCoord2f(0.0F, 0.0F);
    glVertex2f(left, 0.16F);
    glTexCoord2f(1.0F, 0.0F);
    glVertex2f(right, 0.16F);
    glTexCoord2f(1.0F, 1.0F);
    glVertex2f(right, 1.12F);
    glTexCoord2f(0.0F, 1.0F);
    glVertex2f(left, 1.12F);
    glEnd();
  }

  void drawWindow()
  {
    int framebuffer_width = 0;
    int framebuffer_height = 0;
    glfwGetFramebufferSize(window_, &framebuffer_width, &framebuffer_height);
    glViewport(0, 0, framebuffer_width, framebuffer_height);
    glClearColor(0.06F, 0.06F, 0.06F, 1.0F);
    glClear(GL_COLOR_BUFFER_BIT);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(0.0, kProjectionWidth, 0.0, kProjectionHeight, -1.0, 1.0);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    glColor3f(1.0F, 1.0F, 1.0F);
    glEnable(GL_TEXTURE_2D);
    drawTexturedPanel(gl_textures_[0], 0.02F, 0.98F);
    drawTexturedPanel(gl_textures_[1], 1.02F, 1.98F);
    drawTexturedPanel(gl_textures_[2], 2.02F, 2.98F);
    glDisable(GL_TEXTURE_2D);

    glColor3f(0.35F, 0.35F, 0.35F);
    glBegin(GL_QUADS);
    glVertex2f(kSliderLeft, 0.065F);
    glVertex2f(kSliderRight, 0.065F);
    glVertex2f(kSliderRight, 0.095F);
    glVertex2f(kSliderLeft, 0.095F);
    glEnd();
    const float fraction =
      time_steps_ > 1 ? static_cast<float>(selected_timestep_) / static_cast<float>(time_steps_ - 1)
                      : 0.0F;
    const float knob_x = kSliderLeft + fraction * (kSliderRight - kSliderLeft);
    glColor3f(0.95F, 0.95F, 0.95F);
    glBegin(GL_QUADS);
    glVertex2f(knob_x - 0.018F, 0.04F);
    glVertex2f(knob_x + 0.018F, 0.04F);
    glVertex2f(knob_x + 0.018F, 0.12F);
    glVertex2f(knob_x - 0.018F, 0.12F);
    glEnd();

    const std::string title =
      "MPPI distance textures: road border | drivable area | obstacles (timestep " +
      std::to_string(selected_timestep_) + "/" + std::to_string(time_steps_ - 1) + ")";
    glfwSetWindowTitle(window_, title.c_str());
    glfwSwapBuffers(window_);
  }

  void updateSliderFromCursor(const double cursor_x)
  {
    int window_width = 0;
    int window_height = 0;
    glfwGetWindowSize(window_, &window_width, &window_height);
    (void)window_height;
    if (window_width <= 0) {
      return;
    }
    const float projection_x =
      static_cast<float>(cursor_x / static_cast<double>(window_width)) * kProjectionWidth;
    const float fraction =
      fminf(fmaxf((projection_x - kSliderLeft) / (kSliderRight - kSliderLeft), 0.0F), 1.0F);
    selected_timestep_ = static_cast<int>(fraction * static_cast<float>(time_steps_ - 1) + 0.5F);
  }

  bool cursorIsOverSlider(const double cursor_x, const double cursor_y) const
  {
    int window_width = 0;
    int window_height = 0;
    glfwGetWindowSize(window_, &window_width, &window_height);
    if (window_width <= 0 || window_height <= 0) {
      return false;
    }
    const float projection_x =
      static_cast<float>(cursor_x / static_cast<double>(window_width)) * kProjectionWidth;
    const float projection_y =
      (1.0F - static_cast<float>(cursor_y / static_cast<double>(window_height))) *
      kProjectionHeight;
    return projection_x >= kSliderLeft - 0.04F && projection_x <= kSliderRight + 0.04F &&
           projection_y >= 0.0F && projection_y <= 0.15F;
  }

  static void mouseButtonCallback(
    GLFWwindow * window, const int button, const int action, const int /*mods*/)
  {
    auto * self = static_cast<DistanceMapTextureVisualizer *>(glfwGetWindowUserPointer(window));
    if (self == nullptr || button != GLFW_MOUSE_BUTTON_LEFT) {
      return;
    }
    if (action == GLFW_RELEASE) {
      self->slider_dragging_ = false;
      return;
    }
    if (action == GLFW_PRESS) {
      double cursor_x = 0.0;
      double cursor_y = 0.0;
      glfwGetCursorPos(window, &cursor_x, &cursor_y);
      self->slider_dragging_ = self->cursorIsOverSlider(cursor_x, cursor_y);
      if (self->slider_dragging_) {
        self->updateSliderFromCursor(cursor_x);
      }
    }
  }

  static void cursorPositionCallback(
    GLFWwindow * window, const double cursor_x, const double /*cursor_y*/)
  {
    auto * self = static_cast<DistanceMapTextureVisualizer *>(glfwGetWindowUserPointer(window));
    if (self != nullptr && self->slider_dragging_) {
      self->updateSliderFromCursor(cursor_x);
    }
  }

  static void keyCallback(
    GLFWwindow * window, const int key, const int /*scancode*/, const int action,
    const int /*mods*/)
  {
    if (action != GLFW_PRESS && action != GLFW_REPEAT) {
      return;
    }
    auto * self = static_cast<DistanceMapTextureVisualizer *>(glfwGetWindowUserPointer(window));
    if (self == nullptr) {
      return;
    }
    if (key == GLFW_KEY_LEFT || key == GLFW_KEY_DOWN) {
      self->selected_timestep_ = std::max(0, self->selected_timestep_ - 1);
    } else if (key == GLFW_KEY_RIGHT || key == GLFW_KEY_UP) {
      self->selected_timestep_ = std::min(self->time_steps_ - 1, self->selected_timestep_ + 1);
    } else if (key == GLFW_KEY_HOME) {
      self->selected_timestep_ = 0;
    } else if (key == GLFW_KEY_END) {
      self->selected_timestep_ = self->time_steps_ - 1;
    }
  }

  void release()
  {
    if (window_ != nullptr) {
      glfwMakeContextCurrent(window_);
      if (visualization_stream_ != nullptr) {
        HANDLE_ERROR(cudaStreamSynchronize(visualization_stream_));
      }
      for (cudaGraphicsResource_t & resource : cuda_resources_) {
        if (resource != nullptr) {
          HANDLE_ERROR(cudaGraphicsUnregisterResource(resource));
          resource = nullptr;
        }
      }
      glDeleteTextures(static_cast<GLsizei>(gl_textures_.size()), gl_textures_.data());
      gl_textures_.fill(0U);
      glfwDestroyWindow(window_);
      window_ = nullptr;
    }
    if (distance_maps_ready_event_ != nullptr) {
      HANDLE_ERROR(cudaEventDestroy(distance_maps_ready_event_));
      distance_maps_ready_event_ = nullptr;
    }
    if (visualization_stream_ != nullptr) {
      HANDLE_ERROR(cudaStreamDestroy(visualization_stream_));
      visualization_stream_ = nullptr;
    }
    if (glfw_acquired_) {
      releaseGlfw();
      glfw_acquired_ = false;
    }
  }

  int static_width_ = 0;
  int static_height_ = 0;
  int obstacle_width_ = 0;
  int obstacle_height_ = 0;
  int time_steps_ = 1;
  int selected_timestep_ = 0;
  bool slider_dragging_ = false;
  bool glfw_acquired_ = false;
  GLFWwindow * window_ = nullptr;
  cudaStream_t visualization_stream_ = nullptr;
  cudaEvent_t distance_maps_ready_event_ = nullptr;
  std::array<GLuint, 3> gl_textures_{};
  std::array<cudaGraphicsResource_t, 3> cuda_resources_{};
};

__host__ void configureDistanceMapTextureVisualizer(
  DistanceMapTextureVisualizer *& visualizer, const bool enable, const int static_width,
  const int static_height, const int obstacle_width, const int obstacle_height,
  const int time_steps)
{
  const std::lock_guard<std::mutex> lock(visualizerInstanceMutex());
  if (!enable) {
    delete visualizer;
    visualizer = nullptr;
    return;
  }
  if (visualizer != nullptr) {
    return;
  }
  try {
    visualizer = new DistanceMapTextureVisualizer(
      static_width, static_height, obstacle_width, obstacle_height, time_steps);
  } catch (const std::exception & error) {
    std::cerr << "MPPI distance-map texture visualization disabled: " << error.what() << '\n';
  }
}

__host__ bool renderDistanceMapTextureVisualizer(
  DistanceMapTextureVisualizer *& visualizer, const cudaTextureObject_t static_texture,
  const cudaTextureObject_t obstacle_texture, const bool road_border_valid,
  const bool drivable_area_valid, const bool obstacle_valid, const bool obstacle_has_obstacles,
  const float road_border_margin, const float drivable_area_margin, const float obstacle_margin,
  const float static_resolution, const float obstacle_resolution,
  const cudaStream_t producer_stream, const bool update_static)
{
  const std::lock_guard<std::mutex> lock(visualizerInstanceMutex());
  if (visualizer == nullptr) {
    return false;
  }
  const bool keep_open = visualizer->render(
    static_texture, obstacle_texture, road_border_valid, drivable_area_valid, obstacle_valid,
    obstacle_has_obstacles, road_border_margin, drivable_area_margin, obstacle_margin,
    static_resolution, obstacle_resolution, producer_stream, update_static);
  if (!keep_open) {
    delete visualizer;
    visualizer = nullptr;
  }
  return true;
}
