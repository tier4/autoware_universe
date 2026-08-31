#include "mppi/cost_functions/sat.cuh"

#include <mppi/cost_functions/dubins/first_order_dubins_bicycle_cost.cuh>
#include <mppi/cost_functions/path_tracking_geometry.cuh>
#include <mppi/utils/angle_utils.cuh>

#include <mppi/utils/math_utils.h>

#include <algorithm>
#include <array>
#include <cmath>

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
  refreshDistanceMapTextures(geometry_changed, false, false);
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
  refreshDistanceMapTextures(geometry_changed, false, false);
}

template <class CLASS_T, int NUM_TIMESTEPS, class PARAMS_T, class DYN_PARAMS_T>
void FirstOrderDubinsBicycleCostImpl<
  CLASS_T, NUM_TIMESTEPS, PARAMS_T, DYN_PARAMS_T>::clearObstacles()
{
  const bool geometry_changed = num_obstacles_ != 0;
  num_obstacles_ = 0;
  dataToDevice();
  refreshDistanceMapTextures(geometry_changed, false, false);
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
  refreshDistanceMapTextures(false, geometry_changed, false);
}

template <class CLASS_T, int NUM_TIMESTEPS, class PARAMS_T, class DYN_PARAMS_T>
void FirstOrderDubinsBicycleCostImpl<
  CLASS_T, NUM_TIMESTEPS, PARAMS_T, DYN_PARAMS_T>::clearRoadBorders()
{
  const bool geometry_changed = num_road_border_segments_ != 0;
  num_road_border_segments_ = 0;
  dataToDevice();
  refreshDistanceMapTextures(false, geometry_changed, false);
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
  refreshDistanceMapTextures(false, false, geometry_changed);
}

template <class CLASS_T, int NUM_TIMESTEPS, class PARAMS_T, class DYN_PARAMS_T>
void FirstOrderDubinsBicycleCostImpl<
  CLASS_T, NUM_TIMESTEPS, PARAMS_T, DYN_PARAMS_T>::clearDrivableAreaSegments()
{
  const bool geometry_changed = num_drivable_area_segments_ != 0;
  num_drivable_area_segments_ = 0;
  dataToDevice();
  refreshDistanceMapTextures(false, false, geometry_changed);
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
