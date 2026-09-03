/** CUDA distance-map texture state and geometry helpers. */
#pragma once

#ifndef MPPI_COST_FUNCTIONS_DUBINS_DISTANCE_MAP_TEXTURE_CUH_
#define MPPI_COST_FUNCTIONS_DUBINS_DISTANCE_MAP_TEXTURE_CUH_

#include <mppi/cost_functions/path_tracking_geometry.cuh>

#include <cuda_runtime.h>

#include <type_traits>

struct DistanceMapTextureGrid
{
  float origin_x = 0.0F;
  float origin_y = 0.0F;
  float resolution = 0.15F;
  int width = 0;
  int height = 0;
  int time_steps = 0;
};

class DistanceMapTextureVisualizer;
struct DistanceMapTextureTestAccess;

inline constexpr float kDistanceMapEmptyDistance = 1.0E8F;
inline constexpr int kEgoSpineCircleCount = 4;

/**
 * Trivially-copyable texture state embedded in the hybrid host/device cost object.
 * Device code reads the grids, texture handles, and validity flags. CUDA arrays, surfaces, and
 * the raw visualizer pointer are host-owned but remain POD so copying the cost object is safe.
 */
class DistanceMapTextureState
{
public:
  // BEGIN contiguous device-state block. distanceMapStateToDevice() uploads this range once.
  DistanceMapTextureGrid static_distance_map_grid_{};
  DistanceMapTextureGrid obstacle_distance_map_grid_{};
  cudaTextureObject_t static_distance_texture_ = 0;
  cudaTextureObject_t obstacle_distance_texture_ = 0;
  bool road_border_texture_valid_ = false;
  bool drivable_area_texture_valid_ = false;
  bool obstacle_texture_valid_ = false;
  bool obstacle_texture_has_obstacles_ = false;
  // END contiguous device-state block.

protected:
  friend struct DistanceMapTextureTestAccess;

  static constexpr int kStaticDistanceMapWidth = 1024;
  static constexpr int kStaticDistanceMapHeight = 1024;
  static constexpr float kStaticDistanceMapResolution = 0.15F;
  static constexpr int kObstacleDistanceMapWidth = 512;
  static constexpr int kObstacleDistanceMapHeight = 512;
  static constexpr float kObstacleDistanceMapResolution = 0.30F;

  cudaArray_t static_distance_array_ = nullptr;
  cudaArray_t obstacle_distance_array_ = nullptr;
  cudaSurfaceObject_t static_distance_surface_ = 0;
  cudaSurfaceObject_t obstacle_distance_surface_ = 0;
  DistanceMapTextureVisualizer * distance_map_visualizer_ = nullptr;
  bool static_distance_visualization_dirty_ = true;
};

static_assert(
  std::is_trivially_copyable<DistanceMapTextureState>::value,
  "DistanceMapTextureState is copied into device memory with the cost object");

__host__ void configureDistanceMapTextureVisualizer(
  DistanceMapTextureVisualizer *& visualizer, bool enable, int static_width, int static_height,
  int obstacle_width, int obstacle_height, int time_steps);

/** Returns true when a visualization frame was attempted. */
__host__ bool renderDistanceMapTextureVisualizer(
  DistanceMapTextureVisualizer *& visualizer, cudaTextureObject_t static_texture,
  cudaTextureObject_t obstacle_texture, bool road_border_valid, bool drivable_area_valid,
  bool obstacle_valid, bool obstacle_has_obstacles, float road_border_margin,
  float drivable_area_margin, float obstacle_margin, float static_resolution,
  float obstacle_resolution, cudaStream_t producer_stream, bool update_static);

__host__ __device__ inline void computeEgoSpineCircles(
  const float center_x, const float center_y, const float cos_yaw, const float sin_yaw,
  const float half_length, const float half_width, float circle_x[kEgoSpineCircleCount],
  float circle_y[kEgoSpineCircleCount], float & circle_radius)
{
  const float clamped_half_length = fmaxf(half_length, 0.0F);
  const float slice_half_length = clamped_half_length / static_cast<float>(kEgoSpineCircleCount);
  circle_radius = mppi::cost::detail::vectorLength(slice_half_length, fmaxf(half_width, 0.0F));
#pragma unroll
  for (int circle = 0; circle < kEgoSpineCircleCount; ++circle) {
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
  const float outside_distance = mppi::cost::detail::vectorLength(outside_x, outside_y);
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
        minimum, mppi::cost::detail::distancePointToSegment(
                   circle_x[circle], circle_y[circle], segment_x0[segment], segment_y0[segment],
                   segment_x1[segment], segment_y1[segment]) -
                   circle_radius);
    }
  }
  return signed_penetration ? minimum : fmaxf(minimum, 0.0F);
}

#endif  // MPPI_COST_FUNCTIONS_DUBINS_DISTANCE_MAP_TEXTURE_CUH_
