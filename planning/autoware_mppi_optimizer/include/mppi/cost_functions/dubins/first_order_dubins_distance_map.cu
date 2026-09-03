#include <mppi/cost_functions/dubins/distance_map_texture.cuh>
#include <mppi/cost_functions/dubins/first_order_dubins_bicycle_cost.cuh>

#include <algorithm>
#include <cmath>

namespace
{
using mppi::cost::detail::distancePointToSegment;

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
}  // namespace

template <class CLASS_T, int NUM_TIMESTEPS, class PARAMS_T, class DYN_PARAMS_T>
__host__ void FirstOrderDubinsBicycleCostImpl<CLASS_T, NUM_TIMESTEPS, PARAMS_T, DYN_PARAMS_T>::
  setDistanceMapTextureDebugEnabled(const bool enable)
{
  const bool visualizer_was_enabled = this->distance_map_visualizer_ != nullptr;
  configureDistanceMapTextureVisualizer(
    this->distance_map_visualizer_, enable, this->kStaticDistanceMapWidth,
    this->kStaticDistanceMapHeight, this->kObstacleDistanceMapWidth,
    this->kObstacleDistanceMapHeight, NUM_TIMESTEPS);
  if (enable && !visualizer_was_enabled && this->distance_map_visualizer_ != nullptr) {
    this->static_distance_visualization_dirty_ = true;
  }
}

template <class CLASS_T, int NUM_TIMESTEPS, class PARAMS_T, class DYN_PARAMS_T>
__host__ void FirstOrderDubinsBicycleCostImpl<
  CLASS_T, NUM_TIMESTEPS, PARAMS_T, DYN_PARAMS_T>::renderDistanceMapTextureDebug()
{
  const bool rendered = renderDistanceMapTextureVisualizer(
    this->distance_map_visualizer_, this->static_distance_texture_,
    this->obstacle_distance_texture_, this->road_border_texture_valid_,
    this->drivable_area_texture_valid_, this->obstacle_texture_valid_,
    this->obstacle_texture_has_obstacles_, this->params_.road_border_safe_margin,
    this->params_.drivable_area_safe_margin, this->params_.obstacle_safe_margin,
    this->static_distance_map_grid_.resolution, this->obstacle_distance_map_grid_.resolution,
    this->stream_, this->static_distance_visualization_dirty_);
  if (rendered) {
    this->static_distance_visualization_dirty_ = false;
  }
}

template <class CLASS_T, int NUM_TIMESTEPS, class PARAMS_T, class DYN_PARAMS_T>
__host__ bool FirstOrderDubinsBicycleCostImpl<CLASS_T, NUM_TIMESTEPS, PARAMS_T, DYN_PARAMS_T>::
  updateDistanceMapGrid(
    DistanceMapTextureGrid & grid, const int width, const int height, const float resolution)
{
  float minimum_x = this->ref_x_[0];
  float maximum_x = this->ref_x_[0];
  float minimum_y = this->ref_y_[0];
  float maximum_y = this->ref_y_[0];
  for (int timestep = 1; timestep < NUM_TIMESTEPS; ++timestep) {
    minimum_x = std::min(minimum_x, this->ref_x_[timestep]);
    maximum_x = std::max(maximum_x, this->ref_x_[timestep]);
    minimum_y = std::min(minimum_y, this->ref_y_[timestep]);
    maximum_y = std::max(maximum_y, this->ref_y_[timestep]);
  }

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
  if (this->static_distance_array_ != nullptr) {
    return;
  }

  const cudaChannelFormatDesc static_channel = cudaCreateChannelDesc<float2>();
  HANDLE_ERROR(cudaMallocArray(
    &this->static_distance_array_, &static_channel, this->kStaticDistanceMapWidth,
    this->kStaticDistanceMapHeight, cudaArraySurfaceLoadStore));
  const cudaChannelFormatDesc obstacle_channel = cudaCreateChannelDesc<float>();
  const cudaExtent obstacle_extent = make_cudaExtent(
    this->kObstacleDistanceMapWidth, this->kObstacleDistanceMapHeight, NUM_TIMESTEPS);
  HANDLE_ERROR(cudaMalloc3DArray(
    &this->obstacle_distance_array_, &obstacle_channel, obstacle_extent,
    cudaArraySurfaceLoadStore));

  cudaResourceDesc static_resource{};
  static_resource.resType = cudaResourceTypeArray;
  static_resource.res.array.array = this->static_distance_array_;
  HANDLE_ERROR(cudaCreateSurfaceObject(&this->static_distance_surface_, &static_resource));
  cudaTextureDesc static_texture{};
  static_texture.addressMode[0] = cudaAddressModeClamp;
  static_texture.addressMode[1] = cudaAddressModeClamp;
  static_texture.filterMode = cudaFilterModeLinear;
  static_texture.readMode = cudaReadModeElementType;
  static_texture.normalizedCoords = 0;
  HANDLE_ERROR(cudaCreateTextureObject(
    &this->static_distance_texture_, &static_resource, &static_texture, nullptr));

  cudaResourceDesc obstacle_resource{};
  obstacle_resource.resType = cudaResourceTypeArray;
  obstacle_resource.res.array.array = this->obstacle_distance_array_;
  HANDLE_ERROR(cudaCreateSurfaceObject(&this->obstacle_distance_surface_, &obstacle_resource));
  cudaTextureDesc obstacle_texture{};
  obstacle_texture.addressMode[0] = cudaAddressModeClamp;
  obstacle_texture.addressMode[1] = cudaAddressModeClamp;
  obstacle_texture.addressMode[2] = cudaAddressModeClamp;
  obstacle_texture.filterMode = cudaFilterModeLinear;
  obstacle_texture.readMode = cudaReadModeElementType;
  obstacle_texture.normalizedCoords = 0;
  HANDLE_ERROR(cudaCreateTextureObject(
    &this->obstacle_distance_texture_, &obstacle_resource, &obstacle_texture, nullptr));
}

template <class CLASS_T, int NUM_TIMESTEPS, class PARAMS_T, class DYN_PARAMS_T>
__host__ void FirstOrderDubinsBicycleCostImpl<
  CLASS_T, NUM_TIMESTEPS, PARAMS_T, DYN_PARAMS_T>::distanceMapStateToDevice()
{
  if (!this->GPUMemStatus_) {
    return;
  }
  const auto state_begin_address =
    reinterpret_cast<std::uintptr_t>(&this->static_distance_map_grid_);
  const auto state_end_address =
    reinterpret_cast<std::uintptr_t>(&this->obstacle_texture_has_obstacles_) +
    sizeof(this->obstacle_texture_has_obstacles_);
  const std::size_t state_size =
    static_cast<std::size_t>(state_end_address - state_begin_address);
  HANDLE_ERROR(cudaMemcpyAsync(
    &this->cost_d_->static_distance_map_grid_, &this->static_distance_map_grid_, state_size,
    cudaMemcpyHostToDevice, this->stream_));
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
    (this->static_distance_map_grid_.width + static_cast<int>(block.x) - 1) /
      static_cast<int>(block.x),
    (this->static_distance_map_grid_.height + static_cast<int>(block.y) - 1) /
      static_cast<int>(block.y),
    1);
  generateStaticDistanceMapKernel<<<grid, block, 0, this->stream_>>>(
    this->cost_d_, this->static_distance_surface_, this->static_distance_map_grid_,
    update_road_border, update_drivable_area);
  HANDLE_ERROR(cudaGetLastError());
  this->static_distance_visualization_dirty_ = true;
  this->road_border_texture_valid_ = this->road_border_texture_valid_ || update_road_border;
  this->drivable_area_texture_valid_ = this->drivable_area_texture_valid_ || update_drivable_area;
}

template <class CLASS_T, int NUM_TIMESTEPS, class PARAMS_T, class DYN_PARAMS_T>
__host__ void FirstOrderDubinsBicycleCostImpl<
  CLASS_T, NUM_TIMESTEPS, PARAMS_T, DYN_PARAMS_T>::rebuildObstacleDistanceTexture()
{
  if (!this->GPUMemStatus_) {
    return;
  }
  this->obstacle_texture_has_obstacles_ = this->num_obstacles_ > 0;
  if (!this->obstacle_texture_has_obstacles_) {
    this->obstacle_texture_valid_ = true;
    return;
  }
  ensureDistanceMapResources();
  const dim3 block(16, 16, 1);
  const dim3 grid(
    (this->obstacle_distance_map_grid_.width + static_cast<int>(block.x) - 1) /
      static_cast<int>(block.x),
    (this->obstacle_distance_map_grid_.height + static_cast<int>(block.y) - 1) /
      static_cast<int>(block.y),
    this->obstacle_distance_map_grid_.time_steps);
  generateObstacleDistanceMapKernel<<<grid, block, 0, this->stream_>>>(
    this->cost_d_, this->obstacle_distance_surface_, this->obstacle_distance_map_grid_);
  HANDLE_ERROR(cudaGetLastError());
  this->obstacle_texture_valid_ = true;
}

template <class CLASS_T, int NUM_TIMESTEPS, class PARAMS_T, class DYN_PARAMS_T>
__host__ void FirstOrderDubinsBicycleCostImpl<CLASS_T, NUM_TIMESTEPS, PARAMS_T, DYN_PARAMS_T>::
  refreshDistanceMapTextures(
    const bool obstacle_geometry_changed, const bool road_border_geometry_changed,
    const bool drivable_area_geometry_changed)
{
  distance_map_refresh_pending_ = true;
  obstacle_geometry_dirty_ = obstacle_geometry_dirty_ || obstacle_geometry_changed;
  road_border_geometry_dirty_ = road_border_geometry_dirty_ || road_border_geometry_changed;
  drivable_area_geometry_dirty_ =
    drivable_area_geometry_dirty_ || drivable_area_geometry_changed;
  if (data_update_active_) {
    return;
  }

  const bool pending_obstacle_geometry_changed = obstacle_geometry_dirty_;
  const bool pending_road_border_geometry_changed = road_border_geometry_dirty_;
  const bool pending_drivable_area_geometry_changed = drivable_area_geometry_dirty_;
  distance_map_refresh_pending_ = false;
  obstacle_geometry_dirty_ = false;
  road_border_geometry_dirty_ = false;
  drivable_area_geometry_dirty_ = false;
  refreshDistanceMapTexturesNow(
    pending_obstacle_geometry_changed, pending_road_border_geometry_changed,
    pending_drivable_area_geometry_changed);
}

template <class CLASS_T, int NUM_TIMESTEPS, class PARAMS_T, class DYN_PARAMS_T>
__host__ void FirstOrderDubinsBicycleCostImpl<CLASS_T, NUM_TIMESTEPS, PARAMS_T, DYN_PARAMS_T>::
  refreshDistanceMapTexturesNow(
    const bool obstacle_geometry_changed, const bool road_border_geometry_changed,
    const bool drivable_area_geometry_changed)
{
  if (!this->GPUMemStatus_) {
    return;
  }

  const bool static_grid_changed = updateDistanceMapGrid(
    this->static_distance_map_grid_, this->kStaticDistanceMapWidth, this->kStaticDistanceMapHeight,
    this->kStaticDistanceMapResolution);
  const bool obstacle_grid_changed = updateDistanceMapGrid(
    this->obstacle_distance_map_grid_, this->kObstacleDistanceMapWidth,
    this->kObstacleDistanceMapHeight, this->kObstacleDistanceMapResolution);

  if (static_grid_changed) {
    this->road_border_texture_valid_ = false;
    this->drivable_area_texture_valid_ = false;
    rebuildStaticDistanceTexture(true, true);
  } else if (road_border_geometry_changed || drivable_area_geometry_changed) {
    rebuildStaticDistanceTexture(road_border_geometry_changed, drivable_area_geometry_changed);
  }

  if (obstacle_grid_changed) {
    this->obstacle_texture_valid_ = false;
  }
  if (obstacle_grid_changed || obstacle_geometry_changed) {
    rebuildObstacleDistanceTexture();
  }
  if (
    static_grid_changed || obstacle_grid_changed || road_border_geometry_changed ||
    drivable_area_geometry_changed || obstacle_geometry_changed) {
    distanceMapStateToDevice();
  }
}

template <class CLASS_T, int NUM_TIMESTEPS, class PARAMS_T, class DYN_PARAMS_T>
__host__ void FirstOrderDubinsBicycleCostImpl<
  CLASS_T, NUM_TIMESTEPS, PARAMS_T, DYN_PARAMS_T>::releaseDistanceMapResources()
{
  if (
    this->static_distance_texture_ == 0 && this->obstacle_distance_texture_ == 0 &&
    this->static_distance_surface_ == 0 && this->obstacle_distance_surface_ == 0 &&
    this->static_distance_array_ == nullptr && this->obstacle_distance_array_ == nullptr) {
    return;
  }

  HANDLE_ERROR(cudaStreamSynchronize(this->stream_));
  if (this->static_distance_surface_ != 0) {
    HANDLE_ERROR(cudaDestroySurfaceObject(this->static_distance_surface_));
    this->static_distance_surface_ = 0;
  }
  if (this->obstacle_distance_surface_ != 0) {
    HANDLE_ERROR(cudaDestroySurfaceObject(this->obstacle_distance_surface_));
    this->obstacle_distance_surface_ = 0;
  }
  if (this->static_distance_texture_ != 0) {
    HANDLE_ERROR(cudaDestroyTextureObject(this->static_distance_texture_));
    this->static_distance_texture_ = 0;
  }
  if (this->obstacle_distance_texture_ != 0) {
    HANDLE_ERROR(cudaDestroyTextureObject(this->obstacle_distance_texture_));
    this->obstacle_distance_texture_ = 0;
  }
  if (this->static_distance_array_ != nullptr) {
    HANDLE_ERROR(cudaFreeArray(this->static_distance_array_));
    this->static_distance_array_ = nullptr;
  }
  if (this->obstacle_distance_array_ != nullptr) {
    HANDLE_ERROR(cudaFreeArray(this->obstacle_distance_array_));
    this->obstacle_distance_array_ = nullptr;
  }
  this->road_border_texture_valid_ = false;
  this->drivable_area_texture_valid_ = false;
  this->obstacle_texture_valid_ = false;
  this->obstacle_texture_has_obstacles_ = false;
}
