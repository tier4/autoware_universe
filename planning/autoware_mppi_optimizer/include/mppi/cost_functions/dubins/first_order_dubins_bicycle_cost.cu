#include "mppi/cost_functions/sat.cuh"

#include <mppi/cost_functions/dubins/first_order_dubins_bicycle_cost.cuh>
#include <mppi/cost_functions/path_tracking_geometry.cuh>
#include <mppi/utils/angle_utils.cuh>

#include <mppi/utils/math_utils.h>

#define GLFW_INCLUDE_NONE
#include <GL/gl.h>
#include <GLFW/glfw3.h>
#include <cuda_gl_interop.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <string>

namespace
{
using O = FirstOrderDubinsBicycleParams::OutputIndex;
using C = FirstOrderDubinsBicycleParams::ControlIndex;
using mppi::cost::detail::crossTrackDistanceToPolyline;
using mppi::cost::detail::distancePointToSegment;
using mppi::cost::detail::orientedBoxCorners;
using mppi::cost::detail::orientedBoxesOverlap;
using mppi::cost::detail::pathLengthAtProjection;
using mppi::cost::detail::pointInPolygon;
using mppi::cost::detail::projectPointToPolyline;
using mppi::cost::detail::vectorLength;

constexpr float kDistanceMapEmptyDistance = 1.0E8F;
constexpr float kDistanceMapDebugRange = 10.0F;
constexpr int kEgoSpineCircleCount = 4;

__host__ __device__ inline void computeEgoSpineCircles(
  const float center_x, const float center_y, const float cos_yaw, const float sin_yaw,
  const float half_length, const float half_width, float circle_x[kEgoSpineCircleCount],
  float circle_y[kEgoSpineCircleCount], float & circle_radius)
{
  const float clamped_half_length = fmaxf(half_length, 0.0F);
  const float slice_half_length = clamped_half_length / static_cast<float>(kEgoSpineCircleCount);
  circle_radius = vectorLength(slice_half_length, fmaxf(half_width, 0.0F));
#pragma unroll
  for (int circle = 0; circle < kEgoSpineCircleCount; ++circle) {
    // Each circle circumscribes one equal-length slice of the ego rectangle.
    const float longitudinal_offset =
      -clamped_half_length + static_cast<float>(2 * circle + 1) * slice_half_length;
    circle_x[circle] = center_x + longitudinal_offset * cos_yaw;
    circle_y[circle] = center_y + longitudinal_offset * sin_yaw;
  }
}

__device__ inline bool textureCoordinateInBounds(
  const float texture_x, const float texture_y, const DistanceMapTextureGrid & grid)
{
  return texture_x >= 0.0F && texture_x < static_cast<float>(grid.width) && texture_y >= 0.0F &&
         texture_y < static_cast<float>(grid.height);
}

__host__ __device__ inline float signedDistancePointToOrientedBox(
  const float x, const float y, const float cx, const float cy, const float cos_yaw,
  const float sin_yaw, const float half_length, const float half_width)
{
  const float dx = x - cx;
  const float dy = y - cy;
  const float local_x = fabsf(cos_yaw * dx + sin_yaw * dy) - half_length;
  const float local_y = fabsf(-sin_yaw * dx + cos_yaw * dy) - half_width;
  const float outside_x = fmaxf(local_x, 0.0F);
  const float outside_y = fmaxf(local_y, 0.0F);
  const float outside_distance = vectorLength(outside_x, outside_y);
  const float inside_distance = fminf(fmaxf(local_x, local_y), 0.0F);
  return outside_distance + inside_distance;
}

__host__ __device__ inline float distanceEgoSpineToSegments(
  const float circle_x[kEgoSpineCircleCount], const float circle_y[kEgoSpineCircleCount],
  const float circle_radius, const float * segment_x0, const float * segment_y0,
  const float * segment_x1, const float * segment_y1, const int segment_count,
  const bool signed_penetration)
{
  if (segment_count <= 0) {
    return kDistanceMapEmptyDistance;
  }

  float minimum = kDistanceMapEmptyDistance;
#pragma unroll
  for (int circle = 0; circle < kEgoSpineCircleCount; ++circle) {
    for (int segment = 0; segment < segment_count; ++segment) {
      minimum = fminf(
        minimum, distancePointToSegment(
                   circle_x[circle], circle_y[circle], segment_x0[segment], segment_y0[segment],
                   segment_x1[segment], segment_y1[segment]) -
                   circle_radius);
    }
  }
  return signed_penetration ? minimum : fmaxf(minimum, 0.0F);
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

  // Keep the safety margin visually distinct without saturating the rest of the distance field.
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

template <class COST_T>
__global__ void generateStaticDistanceMapKernel(
  const COST_T * cost, const cudaSurfaceObject_t output, const DistanceMapTextureGrid grid,
  const bool update_road_border, const bool update_drivable_area)
{
  constexpr int kMaxRoadSegments = COST_T::kMaxRoadBorderSegments;
  constexpr int kMaxDrivableSegments = COST_T::kMaxDrivableAreaSegments;
  __shared__ float road_x0[kMaxRoadSegments];
  __shared__ float road_y0[kMaxRoadSegments];
  __shared__ float road_x1[kMaxRoadSegments];
  __shared__ float road_y1[kMaxRoadSegments];
  __shared__ float drivable_x0[kMaxDrivableSegments];
  __shared__ float drivable_y0[kMaxDrivableSegments];
  __shared__ float drivable_x1[kMaxDrivableSegments];
  __shared__ float drivable_y1[kMaxDrivableSegments];
  const int local_thread = static_cast<int>(threadIdx.y * blockDim.x + threadIdx.x);
  const int local_thread_count = static_cast<int>(blockDim.x * blockDim.y);
  if (update_road_border) {
    for (int segment = local_thread; segment < cost->num_road_border_segments_;
         segment += local_thread_count) {
      road_x0[segment] = cost->road_border_x0_[segment];
      road_y0[segment] = cost->road_border_y0_[segment];
      road_x1[segment] = cost->road_border_x1_[segment];
      road_y1[segment] = cost->road_border_y1_[segment];
    }
  }
  if (update_drivable_area) {
    for (int segment = local_thread; segment < cost->num_drivable_area_segments_;
         segment += local_thread_count) {
      drivable_x0[segment] = cost->drivable_area_x0_[segment];
      drivable_y0[segment] = cost->drivable_area_y0_[segment];
      drivable_x1[segment] = cost->drivable_area_x1_[segment];
      drivable_y1[segment] = cost->drivable_area_y1_[segment];
    }
  }
  __syncthreads();

  for (int gy = static_cast<int>(blockIdx.y * blockDim.y + threadIdx.y); gy < grid.height;
       gy += static_cast<int>(blockDim.y * gridDim.y)) {
    for (int gx = static_cast<int>(blockIdx.x * blockDim.x + threadIdx.x); gx < grid.width;
         gx += static_cast<int>(blockDim.x * gridDim.x)) {
      const float world_x = grid.origin_x + (static_cast<float>(gx) + 0.5F) * grid.resolution;
      const float world_y = grid.origin_y + (static_cast<float>(gy) + 0.5F) * grid.resolution;
      float2 distances = make_float2(kDistanceMapEmptyDistance, kDistanceMapEmptyDistance);
      if (!update_road_border || !update_drivable_area) {
        distances = surf2Dread<float2>(output, gx * static_cast<int>(sizeof(float2)), gy);
      }

      if (update_road_border) {
        float minimum = kDistanceMapEmptyDistance;
        for (int segment = 0; segment < cost->num_road_border_segments_; ++segment) {
          minimum = fminf(
            minimum, distancePointToSegment(
                       world_x, world_y, road_x0[segment], road_y0[segment], road_x1[segment],
                       road_y1[segment]));
        }
        distances.x = minimum;
      }

      if (update_drivable_area) {
        float minimum = kDistanceMapEmptyDistance;
        for (int segment = 0; segment < cost->num_drivable_area_segments_; ++segment) {
          minimum = fminf(
            minimum, distancePointToSegment(
                       world_x, world_y, drivable_x0[segment], drivable_y0[segment],
                       drivable_x1[segment], drivable_y1[segment]));
        }
        distances.y = minimum;
      }

      surf2Dwrite(distances, output, gx * static_cast<int>(sizeof(float2)), gy);
    }
  }
}

template <class COST_T>
__global__ void generateObstacleDistanceMapKernel(
  const COST_T * cost, const cudaSurfaceObject_t output, const DistanceMapTextureGrid grid)
{
  constexpr int kMaxObstacles = COST_T::kMaxObstacles;
  __shared__ float obstacle_x[kMaxObstacles];
  __shared__ float obstacle_y[kMaxObstacles];
  __shared__ float obstacle_cos[kMaxObstacles];
  __shared__ float obstacle_sin[kMaxObstacles];
  __shared__ float obstacle_half_length[kMaxObstacles];
  __shared__ float obstacle_half_width[kMaxObstacles];
  const int local_thread = static_cast<int>(threadIdx.y * blockDim.x + threadIdx.x);
  const int local_thread_count = static_cast<int>(blockDim.x * blockDim.y);
  for (int timestep = static_cast<int>(blockIdx.z * blockDim.z + threadIdx.z);
       timestep < grid.time_steps; timestep += static_cast<int>(blockDim.z * gridDim.z)) {
    for (int obstacle = local_thread; obstacle < cost->num_obstacles_;
         obstacle += local_thread_count) {
      obstacle_x[obstacle] = cost->obs_x_[obstacle][timestep];
      obstacle_y[obstacle] = cost->obs_y_[obstacle][timestep];
      __sincosf(
        cost->obs_yaw_[obstacle][timestep], &obstacle_sin[obstacle], &obstacle_cos[obstacle]);
      obstacle_half_length[obstacle] = cost->obs_half_length_[obstacle];
      obstacle_half_width[obstacle] = cost->obs_half_width_[obstacle];
    }
    __syncthreads();
    for (int gy = static_cast<int>(blockIdx.y * blockDim.y + threadIdx.y); gy < grid.height;
         gy += static_cast<int>(blockDim.y * gridDim.y)) {
      for (int gx = static_cast<int>(blockIdx.x * blockDim.x + threadIdx.x); gx < grid.width;
           gx += static_cast<int>(blockDim.x * gridDim.x)) {
        const float world_x = grid.origin_x + (static_cast<float>(gx) + 0.5F) * grid.resolution;
        const float world_y = grid.origin_y + (static_cast<float>(gy) + 0.5F) * grid.resolution;
        float minimum = kDistanceMapEmptyDistance;

        for (int obstacle = 0; obstacle < cost->num_obstacles_; ++obstacle) {
          minimum = fminf(
            minimum, signedDistancePointToOrientedBox(
                       world_x, world_y, obstacle_x[obstacle], obstacle_y[obstacle],
                       obstacle_cos[obstacle], obstacle_sin[obstacle],
                       obstacle_half_length[obstacle], obstacle_half_width[obstacle]));
        }

        surf3Dwrite(minimum, output, gx * static_cast<int>(sizeof(float)), gy, timestep);
      }
    }
    __syncthreads();
  }
}

struct CostPathBuffers
{
  float total_path_length_s = 0.0F;
  int num_corridor = 0;
  bool has_corridor_s = false;
  const float * corridor_x = nullptr;
  const float * corridor_y = nullptr;
  const float * corridor_s = nullptr;
  const float * ref_x = nullptr;
  const float * ref_y = nullptr;
  const float * ref_v = nullptr;
  const float * ref_yaw = nullptr;
};

__host__ __device__ inline int clampTimestep(const int timestep, const int num_timesteps)
{
  if (timestep < 0) {
    return 0;
  }
  if (timestep >= num_timesteps) {
    return num_timesteps - 1;
  }
  return timestep;
}

template <class CLASS_T, int NUM_TIMESTEPS, class PARAMS_T, class DYN_PARAMS_T>
__host__ __device__ inline CostPathBuffers resolvePathBuffers(
  const FirstOrderDubinsBicycleCostImpl<CLASS_T, NUM_TIMESTEPS, PARAMS_T, DYN_PARAMS_T> & cost,
  const float * theta_c)
{
  using Cost = FirstOrderDubinsBicycleCostImpl<CLASS_T, NUM_TIMESTEPS, PARAMS_T, DYN_PARAMS_T>;
  CostPathBuffers b;
  if (theta_c != nullptr) {
    b.total_path_length_s = theta_c[Cost::kSharedTotalOffset];
    const float n_raw = theta_c[Cost::kSharedNumCorridorOffset];
#ifdef __CUDA_ARCH__
    b.num_corridor = static_cast<int>(fabsf(n_raw) + 0.5F);
#else
    b.num_corridor = static_cast<int>(std::fabs(n_raw) + 0.5F);
#endif
    b.has_corridor_s = (n_raw > 0.0F);
    b.corridor_x = theta_c + Cost::kSharedCorridorXOffset;
    b.corridor_y = theta_c + Cost::kSharedCorridorYOffset;
    b.corridor_s = theta_c + Cost::kSharedCorridorSOffset;
    b.ref_x = theta_c + Cost::kSharedRefXOffset;
    b.ref_y = theta_c + Cost::kSharedRefYOffset;
    b.ref_v = theta_c + Cost::kSharedRefVOffset;
    b.ref_yaw = theta_c + Cost::kSharedRefYawOffset;
  } else {
    b.total_path_length_s = cost.lateral_corridor_total_length_s_;
    b.num_corridor = cost.num_lateral_corridor_points_;
    b.has_corridor_s = cost.lateral_corridor_has_s_;
    b.corridor_x = cost.lateral_corridor_x_;
    b.corridor_y = cost.lateral_corridor_y_;
    b.corridor_s = cost.lateral_corridor_s_;
    b.ref_x = cost.ref_x_;
    b.ref_y = cost.ref_y_;
    b.ref_v = cost.ref_v_;
    b.ref_yaw = cost.ref_yaw_;
  }
  return b;
}

template <int NUM_TIMESTEPS>
__host__ __device__ float referenceEndYaw(
  const float * x, const float * y, const float * yaw, int count)
{
  if (count <= 0) {
    return 0.0F;
  }
  if (yaw != nullptr) {
    return yaw[count - 1];
  }
  if (count >= 2) {
#ifdef __CUDA_ARCH__
    return atan2f(y[count - 1] - y[count - 2], x[count - 1] - x[count - 2]);
#else
    return std::atan2(y[count - 1] - y[count - 2], x[count - 1] - x[count - 2]);
#endif
  }
  return 0.0F;
}

template <class PARAMS_T>
__host__ __device__ inline bool needsLateralPathMetrics(const PARAMS_T & params)
{
  return params.lateral_distance_coeff > 0.0F || params.lateral_yaw_error_coeff > 0.0F ||
         params.remaining_distance_coeff > 0.0F || params.path_overshoot_coeff > 0.0F ||
         params.lateral_boundary_barrier_weight > 0.0F;
}

__host__ __device__ inline float absLateralDistance(const float signed_lateral_distance)
{
#ifdef __CUDA_ARCH__
  return fabsf(signed_lateral_distance);
#else
  return std::fabs(signed_lateral_distance);
#endif
}

template <class PARAMS_T>
__host__ __device__ float lateralBoundaryBarrierCost(
  const PARAMS_T & params, const float signed_lateral_distance)
{
  const float clearance_to_boundary =
    params.boundary_threshold - absLateralDistance(signed_lateral_distance);
  return computeSmoothBarrierCost(
    clearance_to_boundary, params.lateral_boundary_soft_margin,
    params.lateral_boundary_barrier_weight);
}

template <class PARAMS_T>
__host__ __device__ void comfortTerms(
  const PARAMS_T & params, const float * u, const float * y, float & lateral_accel,
  float & lateral_jerk, float & longitudinal_jerk, float & steer_rate)
{
  const float v = y[static_cast<int>(O::BASELINK_VEL_B_X)];
  const float steer = y[static_cast<int>(O::STEER_ANGLE)];
  const float accel = y[static_cast<int>(O::ACCELERATION)];
  const float accel_cmd = u[static_cast<int>(C::ACCELERATION_CMD)];
  const float steer_cmd = u[static_cast<int>(C::STEER_CMD)];

  const float accel_tau = fmaxf(params.accel_time_constant, 1.0E-4F);
  const float steer_tau = fmaxf(params.steer_time_constant, 1.0E-4F);
  const float wheel_base = fmaxf(params.wheel_base, 1.0E-4F);

  longitudinal_jerk = (accel_cmd - accel) / accel_tau;

  steer_rate = clampSteerRate(params, (steer_cmd - steer) / steer_tau);
  const float curvature = tanf(steer) / wheel_base;
#ifdef __CUDA_ARCH__
  const float sec_sq = 1.0F / fmaxf(cosf(steer) * cosf(steer), 1.0E-6F);
#else
  const float sec_sq = 1.0F / std::max(std::cos(steer) * std::cos(steer), 1.0E-6F);
#endif
  const float curvature_dot = sec_sq * steer_rate / wheel_base;

  lateral_accel = v * v * curvature;
  lateral_jerk = v * v * curvature_dot + 3.0F * v * accel * curvature;
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

      // Relinquish the context so the MPPI control thread can acquire it.
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

    // The mapped arrays and their temporary surface objects must outlive all colorization work.
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

    // Relinquish the context in case teardown runs on the parameter thread.
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
    // Force the driver to allocate texture storage before CUDA registers the image.
    glFinish();
    if (glGetError() != GL_NO_ERROR) {
      throw std::runtime_error("failed to allocate an OpenGL distance-map texture");
    }
    constexpr unsigned int flags =
      cudaGraphicsRegisterFlagsWriteDiscard | cudaGraphicsRegisterFlagsSurfaceLoadStore;
    HANDLE_ERROR(cudaGraphicsGLRegisterImage(
      &cuda_resources_[index], gl_textures_[index], GL_TEXTURE_2D, flags));
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

template <class CLASS_T, int NUM_TIMESTEPS, class PARAMS_T, class DYN_PARAMS_T>
__host__ FirstOrderDubinsBicycleCostImpl<CLASS_T, NUM_TIMESTEPS, PARAMS_T, DYN_PARAMS_T>::
  FirstOrderDubinsBicycleCostImpl(cudaStream_t stream)
{
  this->bindToStream(stream);
  this->SHARED_MEM_REQUEST_GRD_BYTES = static_cast<int>(kSharedNumFloats * sizeof(float));
  this->SHARED_MEM_REQUEST_BLK_BYTES = static_cast<int>(kSharedBlkHintFloats * sizeof(float));
}

template <class CLASS_T, int NUM_TIMESTEPS, class PARAMS_T, class DYN_PARAMS_T>
__host__ FirstOrderDubinsBicycleCostImpl<
  CLASS_T, NUM_TIMESTEPS, PARAMS_T, DYN_PARAMS_T>::~FirstOrderDubinsBicycleCostImpl()
{
  setDistanceMapTextureDebugEnabled(false);
  releaseDistanceMapResources();
}

template <class CLASS_T, int NUM_TIMESTEPS, class PARAMS_T, class DYN_PARAMS_T>
__host__ void FirstOrderDubinsBicycleCostImpl<CLASS_T, NUM_TIMESTEPS, PARAMS_T, DYN_PARAMS_T>::
  setDistanceMapTextureDebugEnabled(const bool enable)
{
  if (!enable) {
    delete distance_map_visualizer_;
    distance_map_visualizer_ = nullptr;
    return;
  }
  if (distance_map_visualizer_ != nullptr) {
    return;
  }
  try {
    distance_map_visualizer_ = new DistanceMapTextureVisualizer(
      kStaticDistanceMapWidth, kStaticDistanceMapHeight, kObstacleDistanceMapWidth,
      kObstacleDistanceMapHeight, NUM_TIMESTEPS);
    static_distance_visualization_dirty_ = true;
  } catch (const std::exception & error) {
    std::cerr << "MPPI distance-map texture visualization disabled: " << error.what() << '\n';
  }
}

template <class CLASS_T, int NUM_TIMESTEPS, class PARAMS_T, class DYN_PARAMS_T>
__host__ void FirstOrderDubinsBicycleCostImpl<
  CLASS_T, NUM_TIMESTEPS, PARAMS_T, DYN_PARAMS_T>::renderDistanceMapTextureDebug()
{
  if (distance_map_visualizer_ == nullptr) {
    return;
  }
  const bool keep_open = distance_map_visualizer_->render(
    static_distance_texture_, obstacle_distance_texture_, road_border_texture_valid_,
    drivable_area_texture_valid_, obstacle_texture_valid_, obstacle_texture_has_obstacles_,
    this->params_.road_border_safe_margin, this->params_.drivable_area_safe_margin,
    this->params_.obstacle_safe_margin, static_distance_map_grid_.resolution,
    obstacle_distance_map_grid_.resolution, this->stream_, static_distance_visualization_dirty_);
  static_distance_visualization_dirty_ = false;
  if (!keep_open) {
    delete distance_map_visualizer_;
    distance_map_visualizer_ = nullptr;
  }
}

template <class CLASS_T, int NUM_TIMESTEPS, class PARAMS_T, class DYN_PARAMS_T>
__host__ bool FirstOrderDubinsBicycleCostImpl<CLASS_T, NUM_TIMESTEPS, PARAMS_T, DYN_PARAMS_T>::
  updateDistanceMapGrid(
    DistanceMapTextureGrid & grid, const int width, const int height, const float resolution)
{
  float minimum_x = ref_x_[0];
  float maximum_x = ref_x_[0];
  float minimum_y = ref_y_[0];
  float maximum_y = ref_y_[0];
  for (int timestep = 1; timestep < NUM_TIMESTEPS; ++timestep) {
    minimum_x = std::min(minimum_x, ref_x_[timestep]);
    maximum_x = std::max(maximum_x, ref_x_[timestep]);
    minimum_y = std::min(minimum_y, ref_y_[timestep]);
    maximum_y = std::max(maximum_y, ref_y_[timestep]);
  }

  // Snapping the ego-centric window to texels avoids regenerating unchanged maps for sub-cell
  // vehicle motion while keeping the entire time-aligned reference centered in the window.
  const float center_x = 0.5F * (minimum_x + maximum_x);
  const float center_y = 0.5F * (minimum_y + maximum_y);
  const float snapped_center_x = std::floor(center_x / resolution + 0.5F) * resolution;
  const float snapped_center_y = std::floor(center_y / resolution + 0.5F) * resolution;
  const float next_origin_x = snapped_center_x - 0.5F * static_cast<float>(width) * resolution;
  const float next_origin_y = snapped_center_y - 0.5F * static_cast<float>(height) * resolution;

  const bool changed = grid.width != width || grid.height != height ||
                       grid.time_steps != NUM_TIMESTEPS || grid.origin_x != next_origin_x ||
                       grid.origin_y != next_origin_y || grid.resolution != resolution;
  grid.origin_x = next_origin_x;
  grid.origin_y = next_origin_y;
  grid.resolution = resolution;
  grid.width = width;
  grid.height = height;
  grid.time_steps = NUM_TIMESTEPS;
  return changed;
}

template <class CLASS_T, int NUM_TIMESTEPS, class PARAMS_T, class DYN_PARAMS_T>
__host__ void FirstOrderDubinsBicycleCostImpl<
  CLASS_T, NUM_TIMESTEPS, PARAMS_T, DYN_PARAMS_T>::ensureDistanceMapResources()
{
  if (static_distance_array_ != nullptr) {
    return;
  }

  const cudaChannelFormatDesc static_channel = cudaCreateChannelDesc<float2>();
  HANDLE_ERROR(cudaMallocArray(
    &static_distance_array_, &static_channel, kStaticDistanceMapWidth, kStaticDistanceMapHeight,
    cudaArraySurfaceLoadStore));
  const cudaChannelFormatDesc obstacle_channel = cudaCreateChannelDesc<float>();
  const cudaExtent obstacle_extent =
    make_cudaExtent(kObstacleDistanceMapWidth, kObstacleDistanceMapHeight, NUM_TIMESTEPS);
  HANDLE_ERROR(cudaMalloc3DArray(
    &obstacle_distance_array_, &obstacle_channel, obstacle_extent, cudaArraySurfaceLoadStore));

  cudaResourceDesc static_resource{};
  static_resource.resType = cudaResourceTypeArray;
  static_resource.res.array.array = static_distance_array_;
  HANDLE_ERROR(cudaCreateSurfaceObject(&static_distance_surface_, &static_resource));
  cudaTextureDesc static_texture{};
  static_texture.addressMode[0] = cudaAddressModeClamp;
  static_texture.addressMode[1] = cudaAddressModeClamp;
  static_texture.filterMode = cudaFilterModeLinear;
  static_texture.readMode = cudaReadModeElementType;
  static_texture.normalizedCoords = 0;
  HANDLE_ERROR(
    cudaCreateTextureObject(&static_distance_texture_, &static_resource, &static_texture, nullptr));

  cudaResourceDesc obstacle_resource{};
  obstacle_resource.resType = cudaResourceTypeArray;
  obstacle_resource.res.array.array = obstacle_distance_array_;
  HANDLE_ERROR(cudaCreateSurfaceObject(&obstacle_distance_surface_, &obstacle_resource));
  cudaTextureDesc obstacle_texture{};
  obstacle_texture.addressMode[0] = cudaAddressModeClamp;
  obstacle_texture.addressMode[1] = cudaAddressModeClamp;
  obstacle_texture.addressMode[2] = cudaAddressModeClamp;
  obstacle_texture.filterMode = cudaFilterModeLinear;
  obstacle_texture.readMode = cudaReadModeElementType;
  obstacle_texture.normalizedCoords = 0;
  HANDLE_ERROR(cudaCreateTextureObject(
    &obstacle_distance_texture_, &obstacle_resource, &obstacle_texture, nullptr));
}

template <class CLASS_T, int NUM_TIMESTEPS, class PARAMS_T, class DYN_PARAMS_T>
__host__ void FirstOrderDubinsBicycleCostImpl<
  CLASS_T, NUM_TIMESTEPS, PARAMS_T, DYN_PARAMS_T>::distanceMapStateToDevice()
{
  if (!this->GPUMemStatus_) {
    return;
  }
  HANDLE_ERROR(cudaMemcpyAsync(
    &this->cost_d_->static_distance_map_grid_, &static_distance_map_grid_,
    sizeof(static_distance_map_grid_), cudaMemcpyHostToDevice, this->stream_));
  HANDLE_ERROR(cudaMemcpyAsync(
    &this->cost_d_->obstacle_distance_map_grid_, &obstacle_distance_map_grid_,
    sizeof(obstacle_distance_map_grid_), cudaMemcpyHostToDevice, this->stream_));
  HANDLE_ERROR(cudaMemcpyAsync(
    &this->cost_d_->static_distance_texture_, &static_distance_texture_,
    sizeof(static_distance_texture_), cudaMemcpyHostToDevice, this->stream_));
  HANDLE_ERROR(cudaMemcpyAsync(
    &this->cost_d_->obstacle_distance_texture_, &obstacle_distance_texture_,
    sizeof(obstacle_distance_texture_), cudaMemcpyHostToDevice, this->stream_));
  HANDLE_ERROR(cudaMemcpyAsync(
    &this->cost_d_->road_border_texture_valid_, &road_border_texture_valid_,
    sizeof(road_border_texture_valid_), cudaMemcpyHostToDevice, this->stream_));
  HANDLE_ERROR(cudaMemcpyAsync(
    &this->cost_d_->drivable_area_texture_valid_, &drivable_area_texture_valid_,
    sizeof(drivable_area_texture_valid_), cudaMemcpyHostToDevice, this->stream_));
  HANDLE_ERROR(cudaMemcpyAsync(
    &this->cost_d_->obstacle_texture_valid_, &obstacle_texture_valid_,
    sizeof(obstacle_texture_valid_), cudaMemcpyHostToDevice, this->stream_));
  HANDLE_ERROR(cudaMemcpyAsync(
    &this->cost_d_->obstacle_texture_has_obstacles_, &obstacle_texture_has_obstacles_,
    sizeof(obstacle_texture_has_obstacles_), cudaMemcpyHostToDevice, this->stream_));
}

template <class CLASS_T, int NUM_TIMESTEPS, class PARAMS_T, class DYN_PARAMS_T>
__host__ void FirstOrderDubinsBicycleCostImpl<CLASS_T, NUM_TIMESTEPS, PARAMS_T, DYN_PARAMS_T>::
  rebuildStaticDistanceTexture(const bool update_road_border, const bool update_drivable_area)
{
  if (!this->GPUMemStatus_ || (!update_road_border && !update_drivable_area)) {
    return;
  }
  ensureDistanceMapResources();
  const dim3 block(16, 16, 1);
  const dim3 grid(
    (static_distance_map_grid_.width + static_cast<int>(block.x) - 1) / static_cast<int>(block.x),
    (static_distance_map_grid_.height + static_cast<int>(block.y) - 1) / static_cast<int>(block.y),
    1);
  generateStaticDistanceMapKernel<<<grid, block, 0, this->stream_>>>(
    this->cost_d_, static_distance_surface_, static_distance_map_grid_, update_road_border,
    update_drivable_area);
  HANDLE_ERROR(cudaGetLastError());
  static_distance_visualization_dirty_ = true;
  road_border_texture_valid_ = road_border_texture_valid_ || update_road_border;
  drivable_area_texture_valid_ = drivable_area_texture_valid_ || update_drivable_area;
  distanceMapStateToDevice();
}

template <class CLASS_T, int NUM_TIMESTEPS, class PARAMS_T, class DYN_PARAMS_T>
__host__ void FirstOrderDubinsBicycleCostImpl<
  CLASS_T, NUM_TIMESTEPS, PARAMS_T, DYN_PARAMS_T>::rebuildObstacleDistanceTexture()
{
  if (!this->GPUMemStatus_) {
    return;
  }
  obstacle_texture_has_obstacles_ = num_obstacles_ > 0;
  if (!obstacle_texture_has_obstacles_) {
    obstacle_texture_valid_ = true;
    distanceMapStateToDevice();
    return;
  }
  ensureDistanceMapResources();
  const dim3 block(16, 16, 1);
  const dim3 grid(
    (obstacle_distance_map_grid_.width + static_cast<int>(block.x) - 1) / static_cast<int>(block.x),
    (obstacle_distance_map_grid_.height + static_cast<int>(block.y) - 1) /
      static_cast<int>(block.y),
    obstacle_distance_map_grid_.time_steps);
  generateObstacleDistanceMapKernel<<<grid, block, 0, this->stream_>>>(
    this->cost_d_, obstacle_distance_surface_, obstacle_distance_map_grid_);
  HANDLE_ERROR(cudaGetLastError());
  obstacle_texture_valid_ = true;
  distanceMapStateToDevice();
}

template <class CLASS_T, int NUM_TIMESTEPS, class PARAMS_T, class DYN_PARAMS_T>
__host__ void FirstOrderDubinsBicycleCostImpl<
  CLASS_T, NUM_TIMESTEPS, PARAMS_T, DYN_PARAMS_T>::releaseDistanceMapResources()
{
  if (
    static_distance_texture_ == 0 && obstacle_distance_texture_ == 0 &&
    static_distance_surface_ == 0 && obstacle_distance_surface_ == 0 &&
    static_distance_array_ == nullptr && obstacle_distance_array_ == nullptr) {
    return;
  }

  HANDLE_ERROR(cudaStreamSynchronize(this->stream_));
  if (static_distance_surface_ != 0) {
    HANDLE_ERROR(cudaDestroySurfaceObject(static_distance_surface_));
    static_distance_surface_ = 0;
  }
  if (obstacle_distance_surface_ != 0) {
    HANDLE_ERROR(cudaDestroySurfaceObject(obstacle_distance_surface_));
    obstacle_distance_surface_ = 0;
  }
  if (static_distance_texture_ != 0) {
    HANDLE_ERROR(cudaDestroyTextureObject(static_distance_texture_));
    static_distance_texture_ = 0;
  }
  if (obstacle_distance_texture_ != 0) {
    HANDLE_ERROR(cudaDestroyTextureObject(obstacle_distance_texture_));
    obstacle_distance_texture_ = 0;
  }
  if (static_distance_array_ != nullptr) {
    HANDLE_ERROR(cudaFreeArray(static_distance_array_));
    static_distance_array_ = nullptr;
  }
  if (obstacle_distance_array_ != nullptr) {
    HANDLE_ERROR(cudaFreeArray(obstacle_distance_array_));
    obstacle_distance_array_ = nullptr;
  }
  road_border_texture_valid_ = false;
  drivable_area_texture_valid_ = false;
  obstacle_texture_valid_ = false;
  obstacle_texture_has_obstacles_ = false;
}

template <class CLASS_T, int NUM_TIMESTEPS, class PARAMS_T, class DYN_PARAMS_T>
__device__ float *
FirstOrderDubinsBicycleCostImpl<CLASS_T, NUM_TIMESTEPS, PARAMS_T, DYN_PARAMS_T>::projectionHintSlot(
  float * theta_c) const
{
  // Must match mppi::kernels::calcClassSharedMemSize float4 alignment.
  const int grd_floats = mppi::math::int_multiple_const(
                           this->SHARED_MEM_REQUEST_GRD_BYTES, static_cast<int>(sizeof(float4))) /
                         static_cast<int>(sizeof(float));
  const int blk_floats = mppi::math::int_multiple_const(
                           this->SHARED_MEM_REQUEST_BLK_BYTES, static_cast<int>(sizeof(float4))) /
                         static_cast<int>(sizeof(float));
  const int shared_idx = static_cast<int>(blockDim.x * threadIdx.z + threadIdx.x);
  return theta_c + grd_floats + shared_idx * blk_floats;
}

template <class CLASS_T, int NUM_TIMESTEPS, class PARAMS_T, class DYN_PARAMS_T>
__device__ void
FirstOrderDubinsBicycleCostImpl<CLASS_T, NUM_TIMESTEPS, PARAMS_T, DYN_PARAMS_T>::initializeCosts(
  float * /*output*/, float * /*control*/, float * theta_c, float /*t_0*/, float /*dt*/)
{
  const int tid =
    static_cast<int>(threadIdx.x + blockDim.x * (threadIdx.y + blockDim.y * threadIdx.z));
  const int nthreads = static_cast<int>(blockDim.x * blockDim.y * blockDim.z);

  if (tid == 0) {
    theta_c[kSharedTotalOffset] = lateral_corridor_total_length_s_;
    // Sign encodes has_s: positive = s valid, negative = recompute from xy, 0 = empty.
    theta_c[kSharedNumCorridorOffset] = lateral_corridor_has_s_
                                          ? static_cast<float>(num_lateral_corridor_points_)
                                          : -static_cast<float>(num_lateral_corridor_points_);
  }

  for (int i = tid; i < kMaxLateralCorridorPoints; i += nthreads) {
    theta_c[kSharedCorridorXOffset + i] = lateral_corridor_x_[i];
    theta_c[kSharedCorridorYOffset + i] = lateral_corridor_y_[i];
    theta_c[kSharedCorridorSOffset + i] = lateral_corridor_s_[i];
  }
  for (int i = tid; i < NUM_TIMESTEPS; i += nthreads) {
    theta_c[kSharedRefXOffset + i] = ref_x_[i];
    theta_c[kSharedRefYOffset + i] = ref_y_[i];
    theta_c[kSharedRefVOffset + i] = ref_v_[i];
    theta_c[kSharedRefYawOffset + i] = ref_yaw_[i];
  }

  // One warm-start slot per sample; -1 forces a full scan on the first projection.
  if (threadIdx.y == 0) {
    *projectionHintSlot(theta_c) = -1.0F;
  }
  __syncthreads();
}

template <class CLASS_T, int NUM_TIMESTEPS, class PARAMS_T, class DYN_PARAMS_T>
void FirstOrderDubinsBicycleCostImpl<
  CLASS_T, NUM_TIMESTEPS, PARAMS_T, DYN_PARAMS_T>::paramsToDevice()
{
  PARENT_CLASS::paramsToDevice();
}

template <class CLASS_T, int NUM_TIMESTEPS, class PARAMS_T, class DYN_PARAMS_T>
void FirstOrderDubinsBicycleCostImpl<CLASS_T, NUM_TIMESTEPS, PARAMS_T, DYN_PARAMS_T>::dataToDevice()
{
  if (!this->GPUMemStatus_) {
    return;
  }

  HANDLE_ERROR(cudaMemcpyAsync(
    this->cost_d_->ref_x_, ref_x_, sizeof(ref_x_), cudaMemcpyHostToDevice, this->stream_));
  HANDLE_ERROR(cudaMemcpyAsync(
    this->cost_d_->ref_y_, ref_y_, sizeof(ref_y_), cudaMemcpyHostToDevice, this->stream_));
  HANDLE_ERROR(cudaMemcpyAsync(
    this->cost_d_->ref_v_, ref_v_, sizeof(ref_v_), cudaMemcpyHostToDevice, this->stream_));
  HANDLE_ERROR(cudaMemcpyAsync(
    this->cost_d_->ref_yaw_, ref_yaw_, sizeof(ref_yaw_), cudaMemcpyHostToDevice, this->stream_));
  HANDLE_ERROR(cudaMemcpyAsync(
    this->cost_d_->ref_max_velocity_, ref_max_velocity_, sizeof(ref_max_velocity_),
    cudaMemcpyHostToDevice, this->stream_));
  HANDLE_ERROR(cudaMemcpyAsync(
    this->cost_d_->ref_velocity_limit_active_, ref_velocity_limit_active_,
    sizeof(ref_velocity_limit_active_), cudaMemcpyHostToDevice, this->stream_));
  HANDLE_ERROR(cudaMemcpyAsync(
    &this->cost_d_->has_pointwise_velocity_limits_, &has_pointwise_velocity_limits_,
    sizeof(has_pointwise_velocity_limits_), cudaMemcpyHostToDevice, this->stream_));
  HANDLE_ERROR(cudaMemcpyAsync(
    &this->cost_d_->kinematic_limits_, &kinematic_limits_, sizeof(kinematic_limits_),
    cudaMemcpyHostToDevice, this->stream_));
  HANDLE_ERROR(cudaMemcpyAsync(
    &this->cost_d_->num_lateral_corridor_points_, &num_lateral_corridor_points_,
    sizeof(num_lateral_corridor_points_), cudaMemcpyHostToDevice, this->stream_));
  if (num_lateral_corridor_points_ > 0) {
    const size_t bytes = static_cast<size_t>(num_lateral_corridor_points_) * sizeof(float);
    HANDLE_ERROR(cudaMemcpyAsync(
      this->cost_d_->lateral_corridor_x_, lateral_corridor_x_, bytes, cudaMemcpyHostToDevice,
      this->stream_));
    HANDLE_ERROR(cudaMemcpyAsync(
      this->cost_d_->lateral_corridor_y_, lateral_corridor_y_, bytes, cudaMemcpyHostToDevice,
      this->stream_));
    HANDLE_ERROR(cudaMemcpyAsync(
      this->cost_d_->lateral_corridor_s_, lateral_corridor_s_, bytes, cudaMemcpyHostToDevice,
      this->stream_));
  }
  HANDLE_ERROR(cudaMemcpyAsync(
    &this->cost_d_->lateral_corridor_has_s_, &lateral_corridor_has_s_,
    sizeof(lateral_corridor_has_s_), cudaMemcpyHostToDevice, this->stream_));
  HANDLE_ERROR(cudaMemcpyAsync(
    &this->cost_d_->lateral_corridor_total_length_s_, &lateral_corridor_total_length_s_,
    sizeof(lateral_corridor_total_length_s_), cudaMemcpyHostToDevice, this->stream_));
  HANDLE_ERROR(cudaMemcpyAsync(
    &this->cost_d_->num_obstacles_, &num_obstacles_, sizeof(num_obstacles_), cudaMemcpyHostToDevice,
    this->stream_));
  HANDLE_ERROR(cudaMemcpyAsync(
    this->cost_d_->obs_x_, obs_x_, sizeof(obs_x_), cudaMemcpyHostToDevice, this->stream_));
  HANDLE_ERROR(cudaMemcpyAsync(
    this->cost_d_->obs_y_, obs_y_, sizeof(obs_y_), cudaMemcpyHostToDevice, this->stream_));
  HANDLE_ERROR(cudaMemcpyAsync(
    this->cost_d_->obs_yaw_, obs_yaw_, sizeof(obs_yaw_), cudaMemcpyHostToDevice, this->stream_));
  HANDLE_ERROR(cudaMemcpyAsync(
    this->cost_d_->obs_half_length_, obs_half_length_, sizeof(obs_half_length_),
    cudaMemcpyHostToDevice, this->stream_));
  HANDLE_ERROR(cudaMemcpyAsync(
    this->cost_d_->obs_half_width_, obs_half_width_, sizeof(obs_half_width_),
    cudaMemcpyHostToDevice, this->stream_));
  HANDLE_ERROR(cudaMemcpyAsync(
    this->cost_d_->obs_is_static_, obs_is_static_, sizeof(obs_is_static_), cudaMemcpyHostToDevice,
    this->stream_));
  HANDLE_ERROR(cudaMemcpyAsync(
    &this->cost_d_->num_road_border_segments_, &num_road_border_segments_,
    sizeof(num_road_border_segments_), cudaMemcpyHostToDevice, this->stream_));
  if (num_road_border_segments_ > 0) {
    const size_t bytes = num_road_border_segments_ * sizeof(float);
    HANDLE_ERROR(cudaMemcpyAsync(
      this->cost_d_->road_border_x0_, road_border_x0_, bytes, cudaMemcpyHostToDevice,
      this->stream_));
    HANDLE_ERROR(cudaMemcpyAsync(
      this->cost_d_->road_border_y0_, road_border_y0_, bytes, cudaMemcpyHostToDevice,
      this->stream_));
    HANDLE_ERROR(cudaMemcpyAsync(
      this->cost_d_->road_border_x1_, road_border_x1_, bytes, cudaMemcpyHostToDevice,
      this->stream_));
    HANDLE_ERROR(cudaMemcpyAsync(
      this->cost_d_->road_border_y1_, road_border_y1_, bytes, cudaMemcpyHostToDevice,
      this->stream_));
  }
  HANDLE_ERROR(cudaMemcpyAsync(
    &this->cost_d_->num_drivable_area_segments_, &num_drivable_area_segments_,
    sizeof(num_drivable_area_segments_), cudaMemcpyHostToDevice, this->stream_));
  if (num_drivable_area_segments_ > 0) {
    const size_t bytes = num_drivable_area_segments_ * sizeof(float);
    HANDLE_ERROR(cudaMemcpyAsync(
      this->cost_d_->drivable_area_x0_, drivable_area_x0_, bytes, cudaMemcpyHostToDevice,
      this->stream_));
    HANDLE_ERROR(cudaMemcpyAsync(
      this->cost_d_->drivable_area_y0_, drivable_area_y0_, bytes, cudaMemcpyHostToDevice,
      this->stream_));
    HANDLE_ERROR(cudaMemcpyAsync(
      this->cost_d_->drivable_area_x1_, drivable_area_x1_, bytes, cudaMemcpyHostToDevice,
      this->stream_));
    HANDLE_ERROR(cudaMemcpyAsync(
      this->cost_d_->drivable_area_y1_, drivable_area_y1_, bytes, cudaMemcpyHostToDevice,
      this->stream_));
  }
}

template <class CLASS_T, int NUM_TIMESTEPS, class PARAMS_T, class DYN_PARAMS_T>
void FirstOrderDubinsBicycleCostImpl<CLASS_T, NUM_TIMESTEPS, PARAMS_T, DYN_PARAMS_T>::
  setKinematicLimits(const FirstOrderDubinsBicycleKinematicLimitData & limits)
{
  kinematic_limits_ = limits;
  dataToDevice();
}

template <class CLASS_T, int NUM_TIMESTEPS, class PARAMS_T, class DYN_PARAMS_T>
void FirstOrderDubinsBicycleCostImpl<CLASS_T, NUM_TIMESTEPS, PARAMS_T, DYN_PARAMS_T>::
  setReferenceTrajectory(
    const float * x, const float * y, const float * v, const int count, const float * yaw,
    const float * max_velocity, const std::uint8_t * velocity_limit_active)
{
  const int n = std::max(0, std::min(count, NUM_TIMESTEPS));
  has_pointwise_velocity_limits_ = max_velocity != nullptr && velocity_limit_active != nullptr;
  const float end_yaw = referenceEndYaw<NUM_TIMESTEPS>(x, y, yaw, n);
  for (int i = 0; i < n; ++i) {
    ref_x_[i] = x[i];
    ref_y_[i] = y[i];
    ref_v_[i] = v[i];
    ref_max_velocity_[i] = has_pointwise_velocity_limits_ ? max_velocity[i] : 0.0F;
    ref_velocity_limit_active_[i] = has_pointwise_velocity_limits_ ? velocity_limit_active[i] : 0U;
    if (yaw != nullptr) {
      ref_yaw_[i] = yaw[i];
    } else if (i >= 1) {
#ifdef __CUDA_ARCH__
      ref_yaw_[i] = atan2f(y[i] - y[i - 1], x[i] - x[i - 1]);
#else
      ref_yaw_[i] = std::atan2(y[i] - y[i - 1], x[i] - x[i - 1]);
#endif
    } else {
      ref_yaw_[i] = end_yaw;
    }
  }
  if (n > 0) {
    for (int i = n; i < NUM_TIMESTEPS; ++i) {
      ref_x_[i] = x[n - 1];
      ref_y_[i] = y[n - 1];
      ref_v_[i] = v[n - 1];
      ref_yaw_[i] = end_yaw;
      ref_max_velocity_[i] = ref_max_velocity_[n - 1];
      ref_velocity_limit_active_[i] = ref_velocity_limit_active_[n - 1];
    }
  } else {
    for (int i = 0; i < NUM_TIMESTEPS; ++i) {
      ref_x_[i] = 0.0F;
      ref_y_[i] = 0.0F;
      ref_v_[i] = 0.0F;
      ref_yaw_[i] = 0.0F;
      ref_max_velocity_[i] = 0.0F;
      ref_velocity_limit_active_[i] = 0U;
    }
  }
  dataToDevice();
}

template <class CLASS_T, int NUM_TIMESTEPS, class PARAMS_T, class DYN_PARAMS_T>
void FirstOrderDubinsBicycleCostImpl<CLASS_T, NUM_TIMESTEPS, PARAMS_T, DYN_PARAMS_T>::
  setLateralCorridor(const float * x, const float * y, const int count, const float * s)
{
  const int n = std::max(0, std::min(count, kMaxLateralCorridorPoints));
  num_lateral_corridor_points_ = n;
  lateral_corridor_has_s_ = (s != nullptr && n > 0);
  for (int i = 0; i < n; ++i) {
    lateral_corridor_x_[i] = x[i];
    lateral_corridor_y_[i] = y[i];
    lateral_corridor_s_[i] = lateral_corridor_has_s_ ? s[i] : 0.0F;
  }
  if (!lateral_corridor_has_s_ && n > 0) {
    lateral_corridor_s_[0] = 0.0F;
    for (int i = 1; i < n; ++i) {
      lateral_corridor_s_[i] =
        lateral_corridor_s_[i - 1] + vectorLength(
                                       lateral_corridor_x_[i] - lateral_corridor_x_[i - 1],
                                       lateral_corridor_y_[i] - lateral_corridor_y_[i - 1]);
    }
    lateral_corridor_has_s_ = true;
  }
  lateral_corridor_total_length_s_ = (n > 0) ? lateral_corridor_s_[n - 1] : 0.0F;
  dataToDevice();
}

template <class CLASS_T, int NUM_TIMESTEPS, class PARAMS_T, class DYN_PARAMS_T>
void FirstOrderDubinsBicycleCostImpl<
  CLASS_T, NUM_TIMESTEPS, PARAMS_T, DYN_PARAMS_T>::clearLateralCorridor()
{
  num_lateral_corridor_points_ = 0;
  lateral_corridor_has_s_ = false;
  lateral_corridor_total_length_s_ = 0.0F;
  dataToDevice();
}

template <class CLASS_T, int NUM_TIMESTEPS, class PARAMS_T, class DYN_PARAMS_T>
void FirstOrderDubinsBicycleCostImpl<CLASS_T, NUM_TIMESTEPS, PARAMS_T, DYN_PARAMS_T>::
  setOrientedBoxObstacles(
    const float * x, const float * y, const float * yaw, const float * half_length,
    const float * half_width, const int count)
{
  const int n = std::max(0, std::min(count, kMaxObstacles));
  bool geometry_changed = n != num_obstacles_;
  for (int i = 0; i < n && !geometry_changed; ++i) {
    geometry_changed = !obs_is_static_[i] || obs_x_[i][0] != x[i] || obs_y_[i][0] != y[i] ||
                       obs_yaw_[i][0] != yaw[i] || obs_half_length_[i] != half_length[i] ||
                       obs_half_width_[i] != half_width[i];
  }
  num_obstacles_ = n;
  for (int i = 0; i < n; ++i) {
    obs_half_length_[i] = half_length[i];
    obs_half_width_[i] = half_width[i];
    obs_is_static_[i] = true;
    for (int t = 0; t < NUM_TIMESTEPS; ++t) {
      obs_x_[i][t] = x[i];
      obs_y_[i][t] = y[i];
      obs_yaw_[i][t] = yaw[i];
    }
  }
  dataToDevice();
  if (this->GPUMemStatus_) {
    const bool static_grid_changed = updateDistanceMapGrid(
      static_distance_map_grid_, kStaticDistanceMapWidth, kStaticDistanceMapHeight,
      kStaticDistanceMapResolution);
    const bool obstacle_grid_changed = updateDistanceMapGrid(
      obstacle_distance_map_grid_, kObstacleDistanceMapWidth, kObstacleDistanceMapHeight,
      kObstacleDistanceMapResolution);
    if (static_grid_changed) {
      road_border_texture_valid_ = false;
      drivable_area_texture_valid_ = false;
      rebuildStaticDistanceTexture(true, true);
    }
    if (obstacle_grid_changed) {
      obstacle_texture_valid_ = false;
    }
    if (obstacle_grid_changed || geometry_changed) {
      rebuildObstacleDistanceTexture();
    }
  }
}

template <class CLASS_T, int NUM_TIMESTEPS, class PARAMS_T, class DYN_PARAMS_T>
void FirstOrderDubinsBicycleCostImpl<CLASS_T, NUM_TIMESTEPS, PARAMS_T, DYN_PARAMS_T>::
  setOrientedBoxObstacleTrajectories(
    const float * x, const float * y, const float * yaw, const float * half_length,
    const float * half_width, const int obstacle_count, const int num_timesteps)
{
  const int n = std::max(0, std::min(obstacle_count, kMaxObstacles));
  const int nt = std::max(0, std::min(num_timesteps, NUM_TIMESTEPS));
  bool geometry_changed = num_obstacles_ != (nt > 0 ? n : 0);
  num_obstacles_ = nt > 0 ? n : 0;
  constexpr float kStaticPoseTolerance = 1.0E-4F;
  for (int i = 0; i < n; ++i) {
    const bool was_static = obs_is_static_[i];
    bool obstacle_geometry_changed =
      obs_half_length_[i] != half_length[i] || obs_half_width_[i] != half_width[i];
    obs_half_length_[i] = half_length[i];
    obs_half_width_[i] = half_width[i];
    obs_is_static_[i] = true;
    for (int t = 0; t < nt; ++t) {
      const int idx = i * nt + t;
      obstacle_geometry_changed = obstacle_geometry_changed || obs_x_[i][t] != x[idx] ||
                                  obs_y_[i][t] != y[idx] || obs_yaw_[i][t] != yaw[idx];
      obs_x_[i][t] = x[idx];
      obs_y_[i][t] = y[idx];
      obs_yaw_[i][t] = yaw[idx];
      if (
        std::fabs(x[idx] - x[i * nt]) > kStaticPoseTolerance ||
        std::fabs(y[idx] - y[i * nt]) > kStaticPoseTolerance ||
        std::fabs(yaw[idx] - yaw[i * nt]) > kStaticPoseTolerance) {
        obs_is_static_[i] = false;
      }
    }
    obstacle_geometry_changed = obstacle_geometry_changed || was_static != obs_is_static_[i];
    if (nt > 0) {
      for (int t = nt; t < NUM_TIMESTEPS; ++t) {
        obstacle_geometry_changed =
          obstacle_geometry_changed || obs_x_[i][t] != obs_x_[i][nt - 1] ||
          obs_y_[i][t] != obs_y_[i][nt - 1] || obs_yaw_[i][t] != obs_yaw_[i][nt - 1];
        obs_x_[i][t] = obs_x_[i][nt - 1];
        obs_y_[i][t] = obs_y_[i][nt - 1];
        obs_yaw_[i][t] = obs_yaw_[i][nt - 1];
      }
    }
    geometry_changed = geometry_changed || obstacle_geometry_changed;
  }
  dataToDevice();
  if (this->GPUMemStatus_) {
    const bool static_grid_changed = updateDistanceMapGrid(
      static_distance_map_grid_, kStaticDistanceMapWidth, kStaticDistanceMapHeight,
      kStaticDistanceMapResolution);
    const bool obstacle_grid_changed = updateDistanceMapGrid(
      obstacle_distance_map_grid_, kObstacleDistanceMapWidth, kObstacleDistanceMapHeight,
      kObstacleDistanceMapResolution);
    if (static_grid_changed) {
      road_border_texture_valid_ = false;
      drivable_area_texture_valid_ = false;
      rebuildStaticDistanceTexture(true, true);
    }
    if (obstacle_grid_changed) {
      obstacle_texture_valid_ = false;
    }
    if (obstacle_grid_changed || geometry_changed) {
      rebuildObstacleDistanceTexture();
    }
  }
}

template <class CLASS_T, int NUM_TIMESTEPS, class PARAMS_T, class DYN_PARAMS_T>
void FirstOrderDubinsBicycleCostImpl<
  CLASS_T, NUM_TIMESTEPS, PARAMS_T, DYN_PARAMS_T>::clearObstacles()
{
  const bool geometry_changed = num_obstacles_ != 0;
  num_obstacles_ = 0;
  dataToDevice();
  if (this->GPUMemStatus_) {
    const bool static_grid_changed = updateDistanceMapGrid(
      static_distance_map_grid_, kStaticDistanceMapWidth, kStaticDistanceMapHeight,
      kStaticDistanceMapResolution);
    const bool obstacle_grid_changed = updateDistanceMapGrid(
      obstacle_distance_map_grid_, kObstacleDistanceMapWidth, kObstacleDistanceMapHeight,
      kObstacleDistanceMapResolution);
    if (static_grid_changed) {
      road_border_texture_valid_ = false;
      drivable_area_texture_valid_ = false;
      rebuildStaticDistanceTexture(true, true);
    }
    if (obstacle_grid_changed) {
      obstacle_texture_valid_ = false;
    }
    if (obstacle_grid_changed || geometry_changed) {
      rebuildObstacleDistanceTexture();
    }
  }
}

template <class CLASS_T, int NUM_TIMESTEPS, class PARAMS_T, class DYN_PARAMS_T>
void FirstOrderDubinsBicycleCostImpl<CLASS_T, NUM_TIMESTEPS, PARAMS_T, DYN_PARAMS_T>::
  setRoadBorderSegments(const std::vector<autoware::mppi_optimizer::Segment> & segments)
{
  const int n = std::min(static_cast<int>(segments.size()), kMaxRoadBorderSegments);
  bool geometry_changed = n != num_road_border_segments_;
  for (int i = 0; i < n && !geometry_changed; ++i) {
    geometry_changed = road_border_x0_[i] != segments[i].x0 ||
                       road_border_y0_[i] != segments[i].y0 ||
                       road_border_x1_[i] != segments[i].x1 || road_border_y1_[i] != segments[i].y1;
  }
  num_road_border_segments_ = n;
  for (int i = 0; i < n; ++i) {
    road_border_x0_[i] = segments[i].x0;
    road_border_y0_[i] = segments[i].y0;
    road_border_x1_[i] = segments[i].x1;
    road_border_y1_[i] = segments[i].y1;
  }
  dataToDevice();
  if (this->GPUMemStatus_) {
    const bool static_grid_changed = updateDistanceMapGrid(
      static_distance_map_grid_, kStaticDistanceMapWidth, kStaticDistanceMapHeight,
      kStaticDistanceMapResolution);
    const bool obstacle_grid_changed = updateDistanceMapGrid(
      obstacle_distance_map_grid_, kObstacleDistanceMapWidth, kObstacleDistanceMapHeight,
      kObstacleDistanceMapResolution);
    if (static_grid_changed) {
      road_border_texture_valid_ = false;
      drivable_area_texture_valid_ = false;
      rebuildStaticDistanceTexture(true, true);
    } else if (geometry_changed) {
      rebuildStaticDistanceTexture(true, false);
    }
    if (obstacle_grid_changed) {
      obstacle_texture_valid_ = false;
      rebuildObstacleDistanceTexture();
    }
  }
}

template <class CLASS_T, int NUM_TIMESTEPS, class PARAMS_T, class DYN_PARAMS_T>
void FirstOrderDubinsBicycleCostImpl<
  CLASS_T, NUM_TIMESTEPS, PARAMS_T, DYN_PARAMS_T>::clearRoadBorders()
{
  const bool geometry_changed = num_road_border_segments_ != 0;
  num_road_border_segments_ = 0;
  dataToDevice();
  if (this->GPUMemStatus_) {
    const bool static_grid_changed = updateDistanceMapGrid(
      static_distance_map_grid_, kStaticDistanceMapWidth, kStaticDistanceMapHeight,
      kStaticDistanceMapResolution);
    const bool obstacle_grid_changed = updateDistanceMapGrid(
      obstacle_distance_map_grid_, kObstacleDistanceMapWidth, kObstacleDistanceMapHeight,
      kObstacleDistanceMapResolution);
    if (static_grid_changed) {
      road_border_texture_valid_ = false;
      drivable_area_texture_valid_ = false;
      rebuildStaticDistanceTexture(true, true);
    } else if (geometry_changed) {
      rebuildStaticDistanceTexture(true, false);
    }
    if (obstacle_grid_changed) {
      obstacle_texture_valid_ = false;
      rebuildObstacleDistanceTexture();
    }
  }
}

template <class CLASS_T, int NUM_TIMESTEPS, class PARAMS_T, class DYN_PARAMS_T>
void FirstOrderDubinsBicycleCostImpl<CLASS_T, NUM_TIMESTEPS, PARAMS_T, DYN_PARAMS_T>::
  setDrivableAreaSegments(const std::vector<autoware::mppi_optimizer::Segment> & segments)
{
  const int n = std::min(static_cast<int>(segments.size()), kMaxDrivableAreaSegments);
  bool geometry_changed = n != num_drivable_area_segments_;
  for (int i = 0; i < n && !geometry_changed; ++i) {
    geometry_changed =
      drivable_area_x0_[i] != segments[i].x0 || drivable_area_y0_[i] != segments[i].y0 ||
      drivable_area_x1_[i] != segments[i].x1 || drivable_area_y1_[i] != segments[i].y1;
  }
  num_drivable_area_segments_ = n;
  for (int i = 0; i < n; ++i) {
    drivable_area_x0_[i] = segments[i].x0;
    drivable_area_y0_[i] = segments[i].y0;
    drivable_area_x1_[i] = segments[i].x1;
    drivable_area_y1_[i] = segments[i].y1;
  }
  dataToDevice();
  if (this->GPUMemStatus_) {
    const bool static_grid_changed = updateDistanceMapGrid(
      static_distance_map_grid_, kStaticDistanceMapWidth, kStaticDistanceMapHeight,
      kStaticDistanceMapResolution);
    const bool obstacle_grid_changed = updateDistanceMapGrid(
      obstacle_distance_map_grid_, kObstacleDistanceMapWidth, kObstacleDistanceMapHeight,
      kObstacleDistanceMapResolution);
    if (static_grid_changed) {
      road_border_texture_valid_ = false;
      drivable_area_texture_valid_ = false;
      rebuildStaticDistanceTexture(true, true);
    } else if (geometry_changed) {
      rebuildStaticDistanceTexture(false, true);
    }
    if (obstacle_grid_changed) {
      obstacle_texture_valid_ = false;
      rebuildObstacleDistanceTexture();
    }
  }
}

template <class CLASS_T, int NUM_TIMESTEPS, class PARAMS_T, class DYN_PARAMS_T>
void FirstOrderDubinsBicycleCostImpl<
  CLASS_T, NUM_TIMESTEPS, PARAMS_T, DYN_PARAMS_T>::clearDrivableAreaSegments()
{
  const bool geometry_changed = num_drivable_area_segments_ != 0;
  num_drivable_area_segments_ = 0;
  dataToDevice();
  if (this->GPUMemStatus_) {
    const bool static_grid_changed = updateDistanceMapGrid(
      static_distance_map_grid_, kStaticDistanceMapWidth, kStaticDistanceMapHeight,
      kStaticDistanceMapResolution);
    const bool obstacle_grid_changed = updateDistanceMapGrid(
      obstacle_distance_map_grid_, kObstacleDistanceMapWidth, kObstacleDistanceMapHeight,
      kObstacleDistanceMapResolution);
    if (static_grid_changed) {
      road_border_texture_valid_ = false;
      drivable_area_texture_valid_ = false;
      rebuildStaticDistanceTexture(true, true);
    } else if (geometry_changed) {
      rebuildStaticDistanceTexture(false, true);
    }
    if (obstacle_grid_changed) {
      obstacle_texture_valid_ = false;
      rebuildObstacleDistanceTexture();
    }
  }
}

template <class CLASS_T, int NUM_TIMESTEPS, class PARAMS_T, class DYN_PARAMS_T>
__host__ __device__ float
FirstOrderDubinsBicycleCostImpl<CLASS_T, NUM_TIMESTEPS, PARAMS_T, DYN_PARAMS_T>::computeTrackValue(
  float x, float y, int timestep, const float * theta_c) const
{
  const auto buf = resolvePathBuffers(*this, theta_c);
  const int t = clampTimestep(timestep, NUM_TIMESTEPS);
  const float dx = x - buf.ref_x[t];
  const float dy = y - buf.ref_y[t];
  return dx * dx + dy * dy;
}

template <class CLASS_T, int NUM_TIMESTEPS, class PARAMS_T, class DYN_PARAMS_T>
__host__ __device__ float
FirstOrderDubinsBicycleCostImpl<CLASS_T, NUM_TIMESTEPS, PARAMS_T, DYN_PARAMS_T>::
  computeHeadingValue(const float yaw, const int timestep, const float * theta_c) const
{
  const auto buf = resolvePathBuffers(*this, theta_c);
  const int t = clampTimestep(timestep, NUM_TIMESTEPS);
  const float yaw_diff = angle_utils::shortestAngularDistance(yaw, buf.ref_yaw[t]);
  return yaw_diff * yaw_diff;
}

template <class CLASS_T, int NUM_TIMESTEPS, class PARAMS_T, class DYN_PARAMS_T>
__host__ __device__ typename FirstOrderDubinsBicycleCostImpl<
  CLASS_T, NUM_TIMESTEPS, PARAMS_T, DYN_PARAMS_T>::LateralPathMetrics
FirstOrderDubinsBicycleCostImpl<CLASS_T, NUM_TIMESTEPS, PARAMS_T, DYN_PARAMS_T>::
  computeLateralPathMetrics(const float x, const float y, const float yaw, float * theta_c) const
{
  LateralPathMetrics metrics;
  const auto buf = resolvePathBuffers(*this, theta_c);
  const float * poly_x = buf.ref_x;
  const float * poly_y = buf.ref_y;
  const float * poly_s = nullptr;
  int n_pts = NUM_TIMESTEPS;
  float total_s = buf.total_path_length_s;
  if (buf.num_corridor >= 2) {
    poly_x = buf.corridor_x;
    poly_y = buf.corridor_y;
    n_pts = buf.num_corridor;
    poly_s = buf.has_corridor_s ? buf.corridor_s : nullptr;
  } else {
    // No corridor: total from xy unless already staged (usually 0 on host fallback).
    total_s = 0.0F;
    for (int i = 0; i < n_pts - 1; ++i) {
      total_s += vectorLength(poly_x[i + 1] - poly_x[i], poly_y[i + 1] - poly_y[i]);
    }
  }

  int hint_i = -1;
#ifdef __CUDA_ARCH__
  float * hint_slot = nullptr;
  if (theta_c != nullptr) {
    hint_slot = projectionHintSlot(theta_c);
    hint_i = static_cast<int>(*hint_slot);
  }
#endif
  const auto proj = projectPointToPolyline(x, y, poly_x, poly_y, n_pts, hint_i);
#ifdef __CUDA_ARCH__
  if (hint_slot != nullptr) {
    *hint_slot = static_cast<float>(proj.best_i);
  }
#endif
  metrics.lateral_distance = proj.lateral_distance;
  metrics.best_segment_i = proj.best_i;

  float path_length_s = 0.0F;
  float remaining_distance_s = 0.0F;
  float overshoot_distance_s = 0.0F;
  pathLengthAtProjection(
    proj, poly_x, poly_y, poly_s, n_pts, total_s, path_length_s, remaining_distance_s,
    overshoot_distance_s);
  metrics.path_length_s = path_length_s;
  metrics.remaining_distance_s = remaining_distance_s;
  metrics.overshoot_distance_s = overshoot_distance_s;

  float tangent_yaw = 0.0F;
  if (n_pts > 1) {
    const int i = proj.best_i;
    const float dx = poly_x[i + 1] - poly_x[i];
    const float dy = poly_y[i + 1] - poly_y[i];
    const float len_sq = dx * dx + dy * dy;
    if (len_sq > 1.0E-8F) {
#ifdef __CUDA_ARCH__
      tangent_yaw = atan2f(dy, dx);
#else
      tangent_yaw = std::atan2(dy, dx);
#endif
    }
  } else if (buf.num_corridor < 2) {
    tangent_yaw = buf.ref_yaw[0];
  }

  const float yaw_diff = angle_utils::shortestAngularDistance(yaw, tangent_yaw);
  metrics.lateral_yaw_error_sq = yaw_diff * yaw_diff;
  return metrics;
}

template <class CLASS_T, int NUM_TIMESTEPS, class PARAMS_T, class DYN_PARAMS_T>
__host__ __device__ float FirstOrderDubinsBicycleCostImpl<
  CLASS_T, NUM_TIMESTEPS, PARAMS_T,
  DYN_PARAMS_T>::computeLateralDistanceValue(const float x, const float y, float * theta_c) const
{
  return computeLateralPathMetrics(x, y, 0.0F, theta_c).lateral_distance;
}

template <class CLASS_T, int NUM_TIMESTEPS, class PARAMS_T, class DYN_PARAMS_T>
__host__ __device__ bool FirstOrderDubinsBicycleCostImpl<
  CLASS_T, NUM_TIMESTEPS, PARAMS_T,
  DYN_PARAMS_T>::exceedsLateralBoundary(const float x, const float y, float * theta_c) const
{
  return absLateralDistance(computeLateralDistanceValue(x, y, theta_c)) >=
         this->params_.boundary_threshold;
}

template <class CLASS_T, int NUM_TIMESTEPS, class PARAMS_T, class DYN_PARAMS_T>
__host__ __device__ bool
FirstOrderDubinsBicycleCostImpl<CLASS_T, NUM_TIMESTEPS, PARAMS_T, DYN_PARAMS_T>::
  egoIntersectsObstacleAtStep(
    const float x, const float y, const float yaw, const int timestep) const
{
  int t = timestep;
  if (t < 0) {
    t = 0;
  } else if (t >= NUM_TIMESTEPS) {
    t = NUM_TIMESTEPS - 1;
  }

#ifdef __CUDA_ARCH__
  const float ego_cos = cosf(yaw);
  const float ego_sin = sinf(yaw);
#else
  const float ego_cos = std::cos(yaw);
  const float ego_sin = std::sin(yaw);
#endif
  const float ego_cx = x + this->params_.ego_axle_to_box_center * ego_cos;
  const float ego_cy = y + this->params_.ego_axle_to_box_center * ego_sin;
  const float margin = this->params_.obstacle_collision_margin;
  const float ego_hl = this->params_.ego_length * 0.5F + margin;
  const float ego_hw = this->params_.ego_width * 0.5F + margin;

#ifdef __CUDA_ARCH__
#pragma unroll
#endif
  for (int i = 0; i < num_obstacles_; ++i) {
#ifdef __CUDA_ARCH__
    const float obs_cos = cosf(obs_yaw_[i][t]);
    const float obs_sin = sinf(obs_yaw_[i][t]);
#else
    const float obs_cos = std::cos(obs_yaw_[i][t]);
    const float obs_sin = std::sin(obs_yaw_[i][t]);
#endif
    if (orientedBoxesOverlap(
          ego_cx, ego_cy, ego_cos, ego_sin, ego_hl, ego_hw, obs_x_[i][t], obs_y_[i][t], obs_cos,
          obs_sin, obs_half_length_[i], obs_half_width_[i])) {
      return true;
    }
  }
  return false;
}

template <class CLASS_T, int NUM_TIMESTEPS, class PARAMS_T, class DYN_PARAMS_T>
__host__ __device__ float
FirstOrderDubinsBicycleCostImpl<CLASS_T, NUM_TIMESTEPS, PARAMS_T, DYN_PARAMS_T>::
  distanceToClosestObstacle(const float x, const float y, const float yaw, const int timestep) const
{
  const int t = timestep < 0 ? 0 : (timestep >= NUM_TIMESTEPS ? NUM_TIMESTEPS - 1 : timestep);
#ifdef __CUDA_ARCH__
  if (obstacle_texture_valid_ && !obstacle_texture_has_obstacles_) {
    return kDistanceMapEmptyDistance;
  }
#endif
  float ego_cos;
  float ego_sin;
#ifdef __CUDA_ARCH__
  __sincosf(yaw, &ego_sin, &ego_cos);
#else
  ego_cos = std::cos(yaw);
  ego_sin = std::sin(yaw);
#endif
  const float ego_cx = x + this->params_.ego_axle_to_box_center * ego_cos;
  const float ego_cy = y + this->params_.ego_axle_to_box_center * ego_sin;
  const float ego_half_length = this->params_.ego_length * 0.5F;
  const float ego_half_width = this->params_.ego_width * 0.5F;
  float circle_x[kEgoSpineCircleCount];
  float circle_y[kEgoSpineCircleCount];
  float circle_radius = 0.0F;
  computeEgoSpineCircles(
    ego_cx, ego_cy, ego_cos, ego_sin, ego_half_length, ego_half_width, circle_x, circle_y,
    circle_radius);
#ifdef __CUDA_ARCH__
  if (obstacle_texture_valid_ && obstacle_distance_texture_ != 0) {
    float texture_x[kEgoSpineCircleCount];
    float texture_y[kEgoSpineCircleCount];
    bool all_circles_in_bounds = true;
#pragma unroll
    for (int circle = 0; circle < kEgoSpineCircleCount; ++circle) {
      texture_x[circle] = (circle_x[circle] - obstacle_distance_map_grid_.origin_x) /
                          obstacle_distance_map_grid_.resolution;
      texture_y[circle] = (circle_y[circle] - obstacle_distance_map_grid_.origin_y) /
                          obstacle_distance_map_grid_.resolution;
      all_circles_in_bounds = all_circles_in_bounds &&
                              textureCoordinateInBounds(
                                texture_x[circle], texture_y[circle], obstacle_distance_map_grid_);
    }
    if (all_circles_in_bounds) {
      const float texture_t = static_cast<float>(t) + 0.5F;
      float minimum = kDistanceMapEmptyDistance;
#pragma unroll
      for (int circle = 0; circle < kEgoSpineCircleCount; ++circle) {
        minimum = fminf(
          minimum, tex3D<float>(
                     obstacle_distance_texture_, texture_x[circle], texture_y[circle], texture_t) -
                     circle_radius);
      }
      return minimum;
    }
  }
#endif
  float min_distance = kDistanceMapEmptyDistance;
#ifdef __CUDA_ARCH__
#pragma unroll
#endif
  for (int i = 0; i < num_obstacles_; ++i) {
    float obs_cos;
    float obs_sin;
#ifdef __CUDA_ARCH__
    __sincosf(obs_yaw_[i][t], &obs_sin, &obs_cos);
#else
    obs_cos = std::cos(obs_yaw_[i][t]);
    obs_sin = std::sin(obs_yaw_[i][t]);
#endif
#pragma unroll
    for (int circle = 0; circle < kEgoSpineCircleCount; ++circle) {
      min_distance = fminf(
        min_distance, signedDistancePointToOrientedBox(
                        circle_x[circle], circle_y[circle], obs_x_[i][t], obs_y_[i][t], obs_cos,
                        obs_sin, obs_half_length_[i], obs_half_width_[i]) -
                        circle_radius);
    }
  }
  return min_distance;
}

template <class CLASS_T, int NUM_TIMESTEPS, class PARAMS_T, class DYN_PARAMS_T>
__host__ __device__ float
FirstOrderDubinsBicycleCostImpl<CLASS_T, NUM_TIMESTEPS, PARAMS_T, DYN_PARAMS_T>::
  computeTrackCenterValue(float x, float y, float yaw, int timestep, const float * theta_c) const
{
  const auto buf = resolvePathBuffers(*this, theta_c);
  const int t = clampTimestep(timestep, NUM_TIMESTEPS);
#ifdef __CUDA_ARCH__
  const float x_center = x + this->params_.ego_axle_to_box_center * cosf(yaw);
  const float y_center = y + this->params_.ego_axle_to_box_center * sinf(yaw);
#else
  const float x_center = x + this->params_.ego_axle_to_box_center * std::cos(yaw);
  const float y_center = y + this->params_.ego_axle_to_box_center * std::sin(yaw);
#endif
  const float dx = x_center - buf.ref_x[t];
  const float dy = y_center - buf.ref_y[t];
  return dx * dx + dy * dy;
}

template <class CLASS_T, int NUM_TIMESTEPS, class PARAMS_T, class DYN_PARAMS_T>
__host__ __device__ float FirstOrderDubinsBicycleCostImpl<
  CLASS_T, NUM_TIMESTEPS, PARAMS_T,
  DYN_PARAMS_T>::computeCornerBufferCost(const float x, const float y, const float yaw) const
{
  if (num_drivable_area_segments_ <= 0 || this->params_.corner_buffer_coeff <= 0.0F) {
    return 0.0F;
  }

  const float half_length = this->params_.ego_length * 0.5F;
  const float half_width = this->params_.ego_width * 0.5F;

  float sin_yaw, cos_yaw;
#ifdef __CUDA_ARCH__
  __sincosf(yaw, &sin_yaw, &cos_yaw);
#else
  cos_yaw = std::cos(yaw);
  sin_yaw = std::sin(yaw);
#endif

  const float center_x = x + this->params_.ego_axle_to_box_center * cos_yaw;
  const float center_y = y + this->params_.ego_axle_to_box_center * sin_yaw;

  float corners_x[4];
  float corners_y[4];
  orientedBoxCorners(
    center_x, center_y, cos_yaw, sin_yaw, half_length, half_width, corners_x, corners_y);

  const float margin = this->params_.corner_safe_margin;
  float total_cost = 0.0F;

#ifdef __CUDA_ARCH__
  if (drivable_area_texture_valid_ && static_distance_texture_ != 0) {
    bool all_corners_in_bounds = true;
#pragma unroll
    for (int corner = 0; corner < 4; ++corner) {
      const float texture_x = (corners_x[corner] - static_distance_map_grid_.origin_x) /
                              static_distance_map_grid_.resolution;
      const float texture_y = (corners_y[corner] - static_distance_map_grid_.origin_y) /
                              static_distance_map_grid_.resolution;
      all_corners_in_bounds =
        all_corners_in_bounds &&
        textureCoordinateInBounds(texture_x, texture_y, static_distance_map_grid_);
    }
    if (all_corners_in_bounds) {
#pragma unroll
      for (int corner = 0; corner < 4; ++corner) {
        const float texture_x = (corners_x[corner] - static_distance_map_grid_.origin_x) /
                                static_distance_map_grid_.resolution;
        const float texture_y = (corners_y[corner] - static_distance_map_grid_.origin_y) /
                                static_distance_map_grid_.resolution;
        const float distance = tex2D<float2>(static_distance_texture_, texture_x, texture_y).y;
        const float violation = fmaxf(0.0F, margin - distance);
        total_cost += violation * violation;
      }
      return this->params_.corner_buffer_coeff * total_cost;
    }
  }
#endif
  for (int corner = 0; corner < 4; ++corner) {
    float min_distance = kDistanceMapEmptyDistance;

    for (int segment = 0; segment < num_drivable_area_segments_; ++segment) {
      const float distance = distancePointToSegment(
        corners_x[corner], corners_y[corner], drivable_area_x0_[segment],
        drivable_area_y0_[segment], drivable_area_x1_[segment], drivable_area_y1_[segment]);

#ifdef __CUDA_ARCH__
      min_distance = fminf(min_distance, distance);
#else
      min_distance = std::min(min_distance, distance);
#endif
    }

    const float violation = fmaxf(0.0F, margin - min_distance);
    total_cost += violation * violation;
  }

  return this->params_.corner_buffer_coeff * total_cost;
}

template <class CLASS_T, int NUM_TIMESTEPS, class PARAMS_T, class DYN_PARAMS_T>
__host__ __device__ bool FirstOrderDubinsBicycleCostImpl<
  CLASS_T, NUM_TIMESTEPS, PARAMS_T,
  DYN_PARAMS_T>::egoIntersectsRoadBorder(const float x, const float y, const float yaw) const
{
  const float half_length = this->params_.ego_length * 0.5f;
  const float half_width = this->params_.ego_width * 0.5f;
  const float offset = this->params_.ego_axle_to_box_center;
  const float front_ext = offset + half_length;
  const float back_ext = half_length - offset;
  const float left_ext = half_width;
  const float right_ext = half_width;
  const float margin = this->params_.road_border_collision_margin;

  return checkRectSegmentIntersections(
    x, y, yaw, front_ext, back_ext, left_ext, right_ext, margin, road_border_x0_, road_border_y0_,
    road_border_x1_, road_border_y1_, num_road_border_segments_);
}

template <class CLASS_T, int NUM_TIMESTEPS, class PARAMS_T, class DYN_PARAMS_T>
__host__ __device__ float FirstOrderDubinsBicycleCostImpl<
  CLASS_T, NUM_TIMESTEPS, PARAMS_T,
  DYN_PARAMS_T>::distanceToRoadBorder(const float x, const float y, const float yaw) const
{
  float cos_yaw;
  float sin_yaw;
#ifdef __CUDA_ARCH__
  __sincosf(yaw, &sin_yaw, &cos_yaw);
#else
  cos_yaw = std::cos(yaw);
  sin_yaw = std::sin(yaw);
#endif
  const float center_x = x + this->params_.ego_axle_to_box_center * cos_yaw;
  const float center_y = y + this->params_.ego_axle_to_box_center * sin_yaw;
  float circle_x[kEgoSpineCircleCount];
  float circle_y[kEgoSpineCircleCount];
  float circle_radius = 0.0F;
  computeEgoSpineCircles(
    center_x, center_y, cos_yaw, sin_yaw, this->params_.ego_length * 0.5F,
    this->params_.ego_width * 0.5F, circle_x, circle_y, circle_radius);
#ifdef __CUDA_ARCH__
  if (road_border_texture_valid_ && static_distance_texture_ != 0) {
    float texture_x[kEgoSpineCircleCount];
    float texture_y[kEgoSpineCircleCount];
    bool all_circles_in_bounds = true;
#pragma unroll
    for (int circle = 0; circle < kEgoSpineCircleCount; ++circle) {
      texture_x[circle] = (circle_x[circle] - static_distance_map_grid_.origin_x) /
                          static_distance_map_grid_.resolution;
      texture_y[circle] = (circle_y[circle] - static_distance_map_grid_.origin_y) /
                          static_distance_map_grid_.resolution;
      all_circles_in_bounds =
        all_circles_in_bounds &&
        textureCoordinateInBounds(texture_x[circle], texture_y[circle], static_distance_map_grid_);
    }
    if (all_circles_in_bounds) {
      float minimum = kDistanceMapEmptyDistance;
#pragma unroll
      for (int circle = 0; circle < kEgoSpineCircleCount; ++circle) {
        minimum = fminf(
          minimum, tex2D<float2>(static_distance_texture_, texture_x[circle], texture_y[circle]).x -
                     circle_radius);
      }
      return fmaxf(minimum, 0.0F);
    }
  }
#endif
  return distanceEgoSpineToSegments(
    circle_x, circle_y, circle_radius, road_border_x0_, road_border_y0_, road_border_x1_,
    road_border_y1_, num_road_border_segments_, false);
}

template <class CLASS_T, int NUM_TIMESTEPS, class PARAMS_T, class DYN_PARAMS_T>
__host__ __device__ float FirstOrderDubinsBicycleCostImpl<
  CLASS_T, NUM_TIMESTEPS, PARAMS_T,
  DYN_PARAMS_T>::distanceToDrivableArea(const float x, const float y, const float yaw) const
{
  float cos_yaw;
  float sin_yaw;
#ifdef __CUDA_ARCH__
  __sincosf(yaw, &sin_yaw, &cos_yaw);
#else
  cos_yaw = std::cos(yaw);
  sin_yaw = std::sin(yaw);
#endif
  const float center_x = x + this->params_.ego_axle_to_box_center * cos_yaw;
  const float center_y = y + this->params_.ego_axle_to_box_center * sin_yaw;
  float circle_x[kEgoSpineCircleCount];
  float circle_y[kEgoSpineCircleCount];
  float circle_radius = 0.0F;
  computeEgoSpineCircles(
    center_x, center_y, cos_yaw, sin_yaw, this->params_.ego_length * 0.5F,
    this->params_.ego_width * 0.5F, circle_x, circle_y, circle_radius);
#ifdef __CUDA_ARCH__
  if (drivable_area_texture_valid_ && static_distance_texture_ != 0) {
    float texture_x[kEgoSpineCircleCount];
    float texture_y[kEgoSpineCircleCount];
    bool all_circles_in_bounds = true;
#pragma unroll
    for (int circle = 0; circle < kEgoSpineCircleCount; ++circle) {
      texture_x[circle] = (circle_x[circle] - static_distance_map_grid_.origin_x) /
                          static_distance_map_grid_.resolution;
      texture_y[circle] = (circle_y[circle] - static_distance_map_grid_.origin_y) /
                          static_distance_map_grid_.resolution;
      all_circles_in_bounds =
        all_circles_in_bounds &&
        textureCoordinateInBounds(texture_x[circle], texture_y[circle], static_distance_map_grid_);
    }
    if (all_circles_in_bounds) {
      float minimum = kDistanceMapEmptyDistance;
#pragma unroll
      for (int circle = 0; circle < kEgoSpineCircleCount; ++circle) {
        minimum = fminf(
          minimum, tex2D<float2>(static_distance_texture_, texture_x[circle], texture_y[circle]).y -
                     circle_radius);
      }
      return minimum;
    }
  }
#endif
  return distanceEgoSpineToSegments(
    circle_x, circle_y, circle_radius, drivable_area_x0_, drivable_area_y0_, drivable_area_x1_,
    drivable_area_y1_, num_drivable_area_segments_, true);
}

template <class CLASS_T, int NUM_TIMESTEPS, class PARAMS_T, class DYN_PARAMS_T>
__host__ __device__ void
FirstOrderDubinsBicycleCostImpl<CLASS_T, NUM_TIMESTEPS, PARAMS_T, DYN_PARAMS_T>::
  computeGradualCrashCosts(
    const float x, const float y, const float yaw, const int timestep, float & drivable_area_cost,
    float & obstacle_cost, float & road_border_cost) const
{
  drivable_area_cost =
    this->params_.drivable_area_barrier_weight == 0.0F
      ? 0.0F
      : computeSmoothBarrierCost(
          distanceToDrivableArea(x, y, yaw), this->params_.drivable_area_safe_margin,
          this->params_.drivable_area_barrier_weight);
  obstacle_cost =
    this->params_.obstacle_barrier_weight == 0.0F
      ? 0.0F
      : computeSmoothBarrierCost(
          distanceToClosestObstacle(x, y, yaw, timestep),
          this->params_.obstacle_collision_margin + this->params_.obstacle_safe_margin,
          this->params_.obstacle_barrier_weight);
  road_border_cost =
    this->params_.road_border_barrier_weight == 0.0F
      ? 0.0F
      : computeSmoothBarrierCost(
          distanceToRoadBorder(x, y, yaw),
          this->params_.road_border_collision_margin + this->params_.road_border_safe_margin,
          this->params_.road_border_barrier_weight);
}

template <class CLASS_T, int NUM_TIMESTEPS, class PARAMS_T, class DYN_PARAMS_T>
autoware::mppi_optimizer::FirstOrderDubinsMppiCostBreakdown
FirstOrderDubinsBicycleCostImpl<CLASS_T, NUM_TIMESTEPS, PARAMS_T, DYN_PARAMS_T>::
  computeRunningCostBreakdown(
    const Eigen::Ref<const output_array> & y, const Eigen::Ref<const control_array> & u,
    const int timestep, int * crash_status) const
{
  autoware::mppi_optimizer::FirstOrderDubinsMppiCostBreakdown result;
  if (crash_status != nullptr) {
    crash_status[0] = 0;
  }

  const float x_pos = y[static_cast<int>(O::BASELINK_POS_I_X)];
  const float y_pos = y[static_cast<int>(O::BASELINK_POS_I_Y)];
  const float yaw = y[static_cast<int>(O::YAW)];
  const float vel = y[static_cast<int>(O::TOTAL_VELOCITY)];
  const float vel_diff = vel - ref_v_[timestep];

  result.speed = this->params_.speed_coeff * vel_diff * vel_diff;
  result.track = this->params_.track_coeff * computeTrackValue(x_pos, y_pos, timestep);
  result.heading = this->params_.heading_coeff * computeHeadingValue(yaw, timestep);
  if (needsLateralPathMetrics(this->params_)) {
    const LateralPathMetrics lateral = computeLateralPathMetrics(x_pos, y_pos, yaw);
    result.lateral_distance =
      this->params_.lateral_distance_coeff * lateral.lateral_distance * lateral.lateral_distance;
    result.lateral_boundary = lateralBoundaryBarrierCost(this->params_, lateral.lateral_distance);
    result.lateral_yaw_error = this->params_.lateral_yaw_error_coeff * lateral.lateral_yaw_error_sq;
    result.remaining_distance = this->params_.remaining_distance_coeff *
                                lateral.remaining_distance_s * lateral.remaining_distance_s;
    result.path_overshoot = this->params_.path_overshoot_coeff * lateral.overshoot_distance_s *
                            lateral.overshoot_distance_s;
  }
  result.track_center =
    this->params_.track_center_coeff * computeTrackCenterValue(x_pos, y_pos, yaw, timestep);
  result.corner_buffer = computeCornerBufferCost(x_pos, y_pos, yaw);
  computeGradualCrashCosts(
    x_pos, y_pos, yaw, timestep, result.drivable_area, result.obstacle, result.road_border);

  const float accel_cmd = u(static_cast<int>(C::ACCELERATION_CMD));
  const float steer_cmd = u(static_cast<int>(C::STEER_CMD));
  result.acceleration_command = this->params_.accel_cmd_coeff * accel_cmd * accel_cmd;
  result.steering_command = this->params_.steer_cmd_coeff * steer_cmd * steer_cmd;

  float lateral_accel = 0.0F;
  float lateral_jerk = 0.0F;
  float longitudinal_jerk = 0.0F;
  float steer_rate = 0.0F;
  comfortTerms(
    this->params_, u.data(), y.data(), lateral_accel, lateral_jerk, longitudinal_jerk, steer_rate);
  result.lateral_acceleration =
    this->params_.lateral_acceleration_coeff * lateral_accel * lateral_accel;
  result.lateral_jerk = this->params_.lateral_jerk_coeff * lateral_jerk * lateral_jerk;
  result.longitudinal_jerk =
    this->params_.longitudinal_jerk_coeff * longitudinal_jerk * longitudinal_jerk;
  result.steering_rate = this->params_.steer_rate_coeff * steer_rate * steer_rate;
  const auto kinematic_cost = computeKinematicLimitCost(
    y[static_cast<int>(O::BASELINK_VEL_B_X)], y[static_cast<int>(O::ACCELERATION)],
    longitudinal_jerk, timestep);
  result.kinematic_velocity_overlimit = kinematic_cost.velocity;
  result.kinematic_acceleration_overlimit = kinematic_cost.acceleration;
  result.kinematic_jerk_overlimit = kinematic_cost.jerk;

  result.running_total = result.componentTotal();
  result.total = result.running_total;
  return result;
}

template <class CLASS_T, int NUM_TIMESTEPS, class PARAMS_T, class DYN_PARAMS_T>
autoware::mppi_optimizer::FirstOrderDubinsMppiCostBreakdown FirstOrderDubinsBicycleCostImpl<
  CLASS_T, NUM_TIMESTEPS, PARAMS_T,
  DYN_PARAMS_T>::computeTerminalCostBreakdown(const Eigen::Ref<const output_array> & y) const
{
  autoware::mppi_optimizer::FirstOrderDubinsMppiCostBreakdown result;
  constexpr int timestep = NUM_TIMESTEPS - 1;
  const float x_pos = y[static_cast<int>(O::BASELINK_POS_I_X)];
  const float y_pos = y[static_cast<int>(O::BASELINK_POS_I_Y)];
  const float yaw = y[static_cast<int>(O::YAW)];

  result.track = this->params_.track_coeff * computeTrackValue(x_pos, y_pos, timestep) *
                 this->params_.track_terminal_scale;
  result.heading = this->params_.heading_coeff * computeHeadingValue(yaw, timestep) *
                   this->params_.track_terminal_scale;
  if (needsLateralPathMetrics(this->params_)) {
    const LateralPathMetrics lateral = computeLateralPathMetrics(x_pos, y_pos, yaw);
    result.lateral_distance = this->params_.lateral_distance_coeff * lateral.lateral_distance *
                              lateral.lateral_distance * this->params_.track_terminal_scale;
    result.lateral_boundary = lateralBoundaryBarrierCost(this->params_, lateral.lateral_distance);
    result.lateral_yaw_error = this->params_.lateral_yaw_error_coeff *
                               lateral.lateral_yaw_error_sq * this->params_.track_terminal_scale;
    result.remaining_distance = this->params_.remaining_distance_coeff *
                                lateral.remaining_distance_s * lateral.remaining_distance_s *
                                this->params_.track_terminal_scale;
    result.path_overshoot = this->params_.path_overshoot_coeff * lateral.overshoot_distance_s *
                            lateral.overshoot_distance_s * this->params_.track_terminal_scale;
  }
  result.track_center = this->params_.track_center_coeff *
                        computeTrackCenterValue(x_pos, y_pos, yaw, timestep) *
                        this->params_.track_terminal_scale;
  result.corner_buffer = computeCornerBufferCost(x_pos, y_pos, yaw);
  computeGradualCrashCosts(
    x_pos, y_pos, yaw, timestep, result.drivable_area, result.obstacle, result.road_border);

  result.terminal_total = result.componentTotal();
  result.total = result.terminal_total;
  return result;
}

template <class CLASS_T, int NUM_TIMESTEPS, class PARAMS_T, class DYN_PARAMS_T>
__device__ float
FirstOrderDubinsBicycleCostImpl<CLASS_T, NUM_TIMESTEPS, PARAMS_T, DYN_PARAMS_T>::computeStateCost(
  float * y, int timestep, float * theta_c, int * crash_status)
{
  const auto buf = resolvePathBuffers(*this, theta_c);
  const float x_pos = y[static_cast<int>(O::BASELINK_POS_I_X)];
  const float y_pos = y[static_cast<int>(O::BASELINK_POS_I_Y)];
  const float yaw = y[static_cast<int>(O::YAW)];
  const float vel = y[static_cast<int>(O::TOTAL_VELOCITY)];

  const float track_val = computeTrackValue(x_pos, y_pos, timestep, theta_c);
  const float vel_diff = vel - buf.ref_v[timestep];
  const float speed_cost = this->params_.speed_coeff * (vel_diff * vel_diff);
  const float track_cost = this->params_.track_coeff * track_val;
  const float heading_cost =
    this->params_.heading_coeff * computeHeadingValue(yaw, timestep, theta_c);
  float lateral_distance_cost = 0.0F;
  float lateral_boundary_cost = 0.0F;
  float lateral_yaw_error_cost = 0.0F;
  float remaining_distance_cost = 0.0F;
  float path_overshoot_cost = 0.0F;
  if (needsLateralPathMetrics(this->params_)) {
    const LateralPathMetrics lateral = computeLateralPathMetrics(x_pos, y_pos, yaw, theta_c);
    lateral_distance_cost =
      this->params_.lateral_distance_coeff * lateral.lateral_distance * lateral.lateral_distance;
    lateral_boundary_cost = lateralBoundaryBarrierCost(this->params_, lateral.lateral_distance);
    lateral_yaw_error_cost = this->params_.lateral_yaw_error_coeff * lateral.lateral_yaw_error_sq;
    remaining_distance_cost = this->params_.remaining_distance_coeff *
                              lateral.remaining_distance_s * lateral.remaining_distance_s;
    path_overshoot_cost = this->params_.path_overshoot_coeff * lateral.overshoot_distance_s *
                          lateral.overshoot_distance_s;
  }
  const float track_center_cost = this->params_.track_center_coeff *
                                  computeTrackCenterValue(x_pos, y_pos, yaw, timestep, theta_c);
  const float corner_buffer_cost = computeCornerBufferCost(x_pos, y_pos, yaw);
  float drivable_area_cost = 0.0F;
  float obstacle_cost = 0.0F;
  float road_border_cost = 0.0F;
  computeGradualCrashCosts(
    x_pos, y_pos, yaw, timestep, drivable_area_cost, obstacle_cost, road_border_cost);

  return speed_cost + track_cost + heading_cost + lateral_distance_cost + lateral_boundary_cost +
         lateral_yaw_error_cost + remaining_distance_cost + path_overshoot_cost +
         drivable_area_cost + track_center_cost + corner_buffer_cost + obstacle_cost +
         road_border_cost;
}

template <class CLASS_T, int NUM_TIMESTEPS, class PARAMS_T, class DYN_PARAMS_T>
float FirstOrderDubinsBicycleCostImpl<CLASS_T, NUM_TIMESTEPS, PARAMS_T, DYN_PARAMS_T>::
  computeStateCost(const Eigen::Ref<const output_array> & y, int timestep, int * crash_status)
{
  if (crash_status != nullptr) {
    crash_status[0] = 0;
  }
  const float x_pos = y[static_cast<int>(O::BASELINK_POS_I_X)];
  const float y_pos = y[static_cast<int>(O::BASELINK_POS_I_Y)];
  const float yaw = y[static_cast<int>(O::YAW)];
  const float vel = y[static_cast<int>(O::TOTAL_VELOCITY)];

  const float track_val = computeTrackValue(x_pos, y_pos, timestep);
  const float vel_diff = vel - ref_v_[timestep];
  const float speed_cost = this->params_.speed_coeff * (vel_diff * vel_diff);
  const float track_cost = this->params_.track_coeff * track_val;
  const float heading_cost = this->params_.heading_coeff * computeHeadingValue(yaw, timestep);
  float lateral_distance_cost = 0.0F;
  float lateral_boundary_cost = 0.0F;
  float lateral_yaw_error_cost = 0.0F;
  float remaining_distance_cost = 0.0F;
  float path_overshoot_cost = 0.0F;
  if (needsLateralPathMetrics(this->params_)) {
    const LateralPathMetrics lateral = computeLateralPathMetrics(x_pos, y_pos, yaw);
    lateral_distance_cost =
      this->params_.lateral_distance_coeff * lateral.lateral_distance * lateral.lateral_distance;
    lateral_boundary_cost = lateralBoundaryBarrierCost(this->params_, lateral.lateral_distance);
    lateral_yaw_error_cost = this->params_.lateral_yaw_error_coeff * lateral.lateral_yaw_error_sq;
    remaining_distance_cost = this->params_.remaining_distance_coeff *
                              lateral.remaining_distance_s * lateral.remaining_distance_s;
    path_overshoot_cost = this->params_.path_overshoot_coeff * lateral.overshoot_distance_s *
                          lateral.overshoot_distance_s;
  }
  const float track_center_cost =
    this->params_.track_center_coeff * computeTrackCenterValue(x_pos, y_pos, yaw, timestep);
  const float corner_buffer_cost = computeCornerBufferCost(x_pos, y_pos, yaw);
  float drivable_area_cost = 0.0F;
  float obstacle_cost = 0.0F;
  float road_border_cost = 0.0F;
  computeGradualCrashCosts(
    x_pos, y_pos, yaw, timestep, drivable_area_cost, obstacle_cost, road_border_cost);

  return speed_cost + track_cost + heading_cost + lateral_distance_cost + lateral_boundary_cost +
         lateral_yaw_error_cost + remaining_distance_cost + path_overshoot_cost +
         drivable_area_cost + track_center_cost + corner_buffer_cost + obstacle_cost +
         road_border_cost;
}

template <class CLASS_T, int NUM_TIMESTEPS, class PARAMS_T, class DYN_PARAMS_T>
__device__ float
FirstOrderDubinsBicycleCostImpl<CLASS_T, NUM_TIMESTEPS, PARAMS_T, DYN_PARAMS_T>::computeControlCost(
  float * u, int timestep, float * theta_c, int * crash)
{
  (void)timestep;
  (void)theta_c;
  (void)crash;
  const float accel_cmd = u[static_cast<int>(C::ACCELERATION_CMD)];
  const float steer_cmd = u[static_cast<int>(C::STEER_CMD)];
  return this->params_.accel_cmd_coeff * (accel_cmd * accel_cmd) +
         this->params_.steer_cmd_coeff * (steer_cmd * steer_cmd);
}

template <class CLASS_T, int NUM_TIMESTEPS, class PARAMS_T, class DYN_PARAMS_T>
float FirstOrderDubinsBicycleCostImpl<CLASS_T, NUM_TIMESTEPS, PARAMS_T, DYN_PARAMS_T>::
  computeControlCost(const Eigen::Ref<const control_array> & u, int timestep, int * crash)
{
  (void)timestep;
  (void)crash;
  const float accel_cmd = u(static_cast<int>(C::ACCELERATION_CMD));
  const float steer_cmd = u(static_cast<int>(C::STEER_CMD));
  return this->params_.accel_cmd_coeff * (accel_cmd * accel_cmd) +
         this->params_.steer_cmd_coeff * (steer_cmd * steer_cmd);
}

template <class CLASS_T, int NUM_TIMESTEPS, class PARAMS_T, class DYN_PARAMS_T>
__device__ float
FirstOrderDubinsBicycleCostImpl<CLASS_T, NUM_TIMESTEPS, PARAMS_T, DYN_PARAMS_T>::terminalCost(
  float * y, float * theta_c)
{
  if (threadIdx.y == 0) {
    const float x_pos = y[static_cast<int>(O::BASELINK_POS_I_X)];
    const float y_pos = y[static_cast<int>(O::BASELINK_POS_I_Y)];
    const float yaw = y[static_cast<int>(O::YAW)];
    constexpr int timestep = NUM_TIMESTEPS - 1;
    const float track_val = computeTrackValue(x_pos, y_pos, timestep, theta_c);
    const float track_cost =
      this->params_.track_coeff * track_val * this->params_.track_terminal_scale;
    const float heading_cost = this->params_.heading_coeff *
                               computeHeadingValue(yaw, timestep, theta_c) *
                               this->params_.track_terminal_scale;
    float lateral_distance_cost = 0.0F;
    float lateral_boundary_cost = 0.0F;
    float lateral_yaw_error_cost = 0.0F;
    float remaining_distance_cost = 0.0F;
    float path_overshoot_cost = 0.0F;
    if (needsLateralPathMetrics(this->params_)) {
      const LateralPathMetrics lateral = computeLateralPathMetrics(x_pos, y_pos, yaw, theta_c);
      lateral_distance_cost = this->params_.lateral_distance_coeff * lateral.lateral_distance *
                              lateral.lateral_distance * this->params_.track_terminal_scale;
      lateral_boundary_cost = lateralBoundaryBarrierCost(this->params_, lateral.lateral_distance);
      lateral_yaw_error_cost = this->params_.lateral_yaw_error_coeff *
                               lateral.lateral_yaw_error_sq * this->params_.track_terminal_scale;
      remaining_distance_cost = this->params_.remaining_distance_coeff *
                                lateral.remaining_distance_s * lateral.remaining_distance_s *
                                this->params_.track_terminal_scale;
      path_overshoot_cost = this->params_.path_overshoot_coeff * lateral.overshoot_distance_s *
                            lateral.overshoot_distance_s * this->params_.track_terminal_scale;
    }
    const float track_center_cost = this->params_.track_center_coeff *
                                    computeTrackCenterValue(x_pos, y_pos, yaw, timestep, theta_c) *
                                    this->params_.track_terminal_scale;
    const float corner_buffer_cost = computeCornerBufferCost(x_pos, y_pos, yaw);
    float drivable_area_cost = 0.0F;
    float obstacle_cost = 0.0F;
    float road_border_cost = 0.0F;
    computeGradualCrashCosts(
      x_pos, y_pos, yaw, NUM_TIMESTEPS - 1, drivable_area_cost, obstacle_cost, road_border_cost);
    return track_cost + heading_cost + lateral_distance_cost + lateral_boundary_cost +
           lateral_yaw_error_cost + remaining_distance_cost + path_overshoot_cost +
           drivable_area_cost + track_center_cost + corner_buffer_cost + obstacle_cost +
           road_border_cost;
  }
  return 0.0F;
}

template <class CLASS_T, int NUM_TIMESTEPS, class PARAMS_T, class DYN_PARAMS_T>
__host__ __device__ FirstOrderDubinsBicycleKinematicCost
FirstOrderDubinsBicycleCostImpl<CLASS_T, NUM_TIMESTEPS, PARAMS_T, DYN_PARAMS_T>::
  computeKinematicLimitCost(
    const float velocity, const float longitudinal_acceleration, const float longitudinal_jerk,
    const int timestep) const
{
  auto limits = kinematic_limits_;
  const int bounded_timestep =
    timestep < 0 ? 0 : (timestep >= NUM_TIMESTEPS ? NUM_TIMESTEPS - 1 : timestep);
  if (has_pointwise_velocity_limits_ && ref_velocity_limit_active_[bounded_timestep] != 0U) {
    limits.active_mask |= kVelocityLimitActive;
    limits.min_velocity = 0.0F;
    limits.max_velocity = ref_max_velocity_[bounded_timestep];
  }
  return computeCappedKinematicIntervalCost(
    limits, this->params_.overlimit_coeff, this->params_.crash_contact_penalty, velocity,
    longitudinal_acceleration, longitudinal_jerk);
}

template <class CLASS_T, int NUM_TIMESTEPS, class PARAMS_T, class DYN_PARAMS_T>
float FirstOrderDubinsBicycleCostImpl<CLASS_T, NUM_TIMESTEPS, PARAMS_T, DYN_PARAMS_T>::
  computeComfortCost(
    const Eigen::Ref<const control_array> & u, const Eigen::Ref<const output_array> & y,
    int timestep)
{
  float lateral_accel = 0.0F;
  float lateral_jerk = 0.0F;
  float longitudinal_jerk = 0.0F;
  float steer_rate = 0.0F;
  comfortTerms(
    this->params_, u.data(), y.data(), lateral_accel, lateral_jerk, longitudinal_jerk, steer_rate);
  const auto kinematic_cost = computeKinematicLimitCost(
    y(static_cast<int>(O::BASELINK_VEL_B_X)), y(static_cast<int>(O::ACCELERATION)),
    longitudinal_jerk, timestep);
  return this->params_.lateral_acceleration_coeff * lateral_accel * lateral_accel +
         this->params_.lateral_jerk_coeff * lateral_jerk * lateral_jerk +
         this->params_.longitudinal_jerk_coeff * longitudinal_jerk * longitudinal_jerk +
         this->params_.steer_rate_coeff * steer_rate * steer_rate + kinematic_cost.total;
}

template <class CLASS_T, int NUM_TIMESTEPS, class PARAMS_T, class DYN_PARAMS_T>
__device__ float
FirstOrderDubinsBicycleCostImpl<CLASS_T, NUM_TIMESTEPS, PARAMS_T, DYN_PARAMS_T>::computeComfortCost(
  float * u, float * y, int timestep)
{
  float lateral_accel = 0.0F;
  float lateral_jerk = 0.0F;
  float longitudinal_jerk = 0.0F;
  float steer_rate = 0.0F;
  comfortTerms(this->params_, u, y, lateral_accel, lateral_jerk, longitudinal_jerk, steer_rate);
  const auto kinematic_cost = computeKinematicLimitCost(
    y[static_cast<int>(O::BASELINK_VEL_B_X)], y[static_cast<int>(O::ACCELERATION)],
    longitudinal_jerk, timestep);
  return this->params_.lateral_acceleration_coeff * lateral_accel * lateral_accel +
         this->params_.lateral_jerk_coeff * lateral_jerk * lateral_jerk +
         this->params_.longitudinal_jerk_coeff * longitudinal_jerk * longitudinal_jerk +
         this->params_.steer_rate_coeff * steer_rate * steer_rate + kinematic_cost.total;
}

template <class CLASS_T, int NUM_TIMESTEPS, class PARAMS_T, class DYN_PARAMS_T>
float FirstOrderDubinsBicycleCostImpl<CLASS_T, NUM_TIMESTEPS, PARAMS_T, DYN_PARAMS_T>::
  computeRunningCost(
    const Eigen::Ref<const output_array> & y, const Eigen::Ref<const control_array> & u,
    int timestep, int * crash)
{
  const float state_cost = computeStateCost(y, timestep, crash);
  return state_cost + computeControlCost(u, timestep, crash) + computeComfortCost(u, y, timestep);
}

template <class CLASS_T, int NUM_TIMESTEPS, class PARAMS_T, class DYN_PARAMS_T>
__device__ float
FirstOrderDubinsBicycleCostImpl<CLASS_T, NUM_TIMESTEPS, PARAMS_T, DYN_PARAMS_T>::computeRunningCost(
  float * y, float * u, int timestep, float * theta_c, int * crash)
{
  if (threadIdx.y == 0) {
    const float state_cost = computeStateCost(y, timestep, theta_c, crash);
    return state_cost + computeControlCost(u, timestep, theta_c, crash) +
           computeComfortCost(u, y, timestep);
  }
  return 0.0F;
}

template <class CLASS_T, int NUM_TIMESTEPS, class PARAMS_T, class DYN_PARAMS_T>
constexpr int
  FirstOrderDubinsBicycleCostImpl<CLASS_T, NUM_TIMESTEPS, PARAMS_T, DYN_PARAMS_T>::kMaxObstacles;
template <class CLASS_T, int NUM_TIMESTEPS, class PARAMS_T, class DYN_PARAMS_T>
constexpr int FirstOrderDubinsBicycleCostImpl<
  CLASS_T, NUM_TIMESTEPS, PARAMS_T, DYN_PARAMS_T>::kMaxDrivablePolygonVertices;
template <class CLASS_T, int NUM_TIMESTEPS, class PARAMS_T, class DYN_PARAMS_T>
constexpr int FirstOrderDubinsBicycleCostImpl<
  CLASS_T, NUM_TIMESTEPS, PARAMS_T, DYN_PARAMS_T>::kMaxRoadBorderSegments;
template <class CLASS_T, int NUM_TIMESTEPS, class PARAMS_T, class DYN_PARAMS_T>
constexpr int FirstOrderDubinsBicycleCostImpl<
  CLASS_T, NUM_TIMESTEPS, PARAMS_T, DYN_PARAMS_T>::kMaxDrivableAreaSegments;
template <class CLASS_T, int NUM_TIMESTEPS, class PARAMS_T, class DYN_PARAMS_T>
constexpr int FirstOrderDubinsBicycleCostImpl<
  CLASS_T, NUM_TIMESTEPS, PARAMS_T, DYN_PARAMS_T>::kMaxLateralCorridorPoints;
template <class CLASS_T, int NUM_TIMESTEPS, class PARAMS_T, class DYN_PARAMS_T>
constexpr int FirstOrderDubinsBicycleCostImpl<
  CLASS_T, NUM_TIMESTEPS, PARAMS_T, DYN_PARAMS_T>::kStaticDistanceMapWidth;
template <class CLASS_T, int NUM_TIMESTEPS, class PARAMS_T, class DYN_PARAMS_T>
constexpr int FirstOrderDubinsBicycleCostImpl<
  CLASS_T, NUM_TIMESTEPS, PARAMS_T, DYN_PARAMS_T>::kStaticDistanceMapHeight;
template <class CLASS_T, int NUM_TIMESTEPS, class PARAMS_T, class DYN_PARAMS_T>
constexpr float FirstOrderDubinsBicycleCostImpl<
  CLASS_T, NUM_TIMESTEPS, PARAMS_T, DYN_PARAMS_T>::kStaticDistanceMapResolution;
template <class CLASS_T, int NUM_TIMESTEPS, class PARAMS_T, class DYN_PARAMS_T>
constexpr int FirstOrderDubinsBicycleCostImpl<
  CLASS_T, NUM_TIMESTEPS, PARAMS_T, DYN_PARAMS_T>::kObstacleDistanceMapWidth;
template <class CLASS_T, int NUM_TIMESTEPS, class PARAMS_T, class DYN_PARAMS_T>
constexpr int FirstOrderDubinsBicycleCostImpl<
  CLASS_T, NUM_TIMESTEPS, PARAMS_T, DYN_PARAMS_T>::kObstacleDistanceMapHeight;
template <class CLASS_T, int NUM_TIMESTEPS, class PARAMS_T, class DYN_PARAMS_T>
constexpr float FirstOrderDubinsBicycleCostImpl<
  CLASS_T, NUM_TIMESTEPS, PARAMS_T, DYN_PARAMS_T>::kObstacleDistanceMapResolution;
