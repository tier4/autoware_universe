/** Path-tracking and texture-accelerated environment cost for FirstOrderDubinsBicycle. */
#pragma once

#ifndef MPPI_COST_FUNCTIONS_FIRST_ORDER_DUBINS_BICYCLE_COST_CUH_
#define MPPI_COST_FUNCTIONS_FIRST_ORDER_DUBINS_BICYCLE_COST_CUH_

#include "autoware/mppi_optimizer/first_order_dubins_mppi_interface.hpp"

#include <mppi/cost_functions/cost.cuh>
#include <mppi/cost_functions/dubins/first_order_dubins_bicycle_kinematic_limits.cuh>
#include <mppi/dynamics/dubins/first_order_dubins_bicycle.cuh>

#include <cstdint>
#include <vector>

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

__host__ __device__ inline float computeSmoothBarrierCost(
  const float distance, const float safe_margin, const float precomputed_weight)
{
  const float violation = fmaxf(0.0f, safe_margin - distance);
  // At distance 0.0, this exactly equals max_penalty.
  // If distance < 0.0, it naturally grows > max_penalty, preserving the gradient!
  return precomputed_weight * violation * violation;
}

template <int NUM_TIMESTEPS>
struct FirstOrderDubinsBicycleCostParams : public CostParams<2>
{
  float speed_coeff = 500.0F;
  /** Pull toward ref position at each step: coeff * ||p - ref[t]||^2; 0 disables. */
  float track_coeff = 1000.0F;
  /** Multiplier on track_coeff * track_val in terminalCost (running state cost uses scale 1). */
  float track_terminal_scale = 10.0F;
  /** Pull toward ref heading at each horizon step: coeff * (yaw - ref_yaw[t])^2; 0 disables. */
  float heading_coeff = 500.0F;
  /** Spatial (closest-segment) cross-track error: coeff * e_lat^2; 0 disables. */
  float lateral_distance_coeff = 0.0F;
  /** Spatial yaw error vs closest-segment tangent: coeff * Δψ^2; 0 disables. */
  float lateral_yaw_error_coeff = 0.0F;
  /** Progress along corridor: coeff * (remaining chord length [m])^2; 0 disables. */
  float remaining_distance_coeff = 0.0F;
  /** Along-track distance past the corridor tip: coeff * (overshoot [m])^2; 0 disables. */
  float path_overshoot_coeff = 0.0F;
  /** Track ego footprint center vs ref[t]: coeff * ||center - ref[t]||^2; 0 disables. */
  float track_center_coeff = 0.0F;
  /** Quadratic soft cost for ego corners closer than corner_safe_margin to a boundary. */
  float corner_buffer_coeff = 0.0F;
  /** Desired minimum distance [m] from each ego corner to drivable-area boundary segments. */
  float corner_safe_margin = 0.3F;
  float boundary_threshold = 0.8F;
  /** Distance inside boundary_threshold at which the gradual lateral barrier activates. */
  float lateral_boundary_soft_margin = 0.2F;
  float lateral_boundary_barrier_weight = 0.0F;
  /** Beyond bound if signed lateral offset exceeds these (path-left = +); <0 falls back to
   * boundary_threshold. */
  float accel_cmd_coeff = 0.0F;
  float steer_cmd_coeff = 0.0F;
  /** Direct cost on steer rate [rad/s]: (steer_cmd - steer) / steer_time_constant. */
  float steer_rate_coeff = 0.0F;
  /** Shared cost weight for optional velocity, acceleration, and jerk interval violations. */
  float overlimit_coeff = 10000.0F;
  float lateral_acceleration_coeff = 300.0F;
  float lateral_jerk_coeff = 300.0F;
  float longitudinal_jerk_coeff = 10.0F;
  float wheel_base = 0.32F;
  float accel_time_constant = 0.15F;
  float steer_time_constant = 0.08F;
  /** From vehicle config steer_rate_lim (set in setup / fillGeometry). */
  float max_steer_rate = 5.0F;
  /** Ego OBB for parked-car collision (rear axle at pose; box center offset forward). */
  float ego_length = 0.55F * 1.5F;
  float ego_width = 0.28F * 1.5F;
  float ego_axle_to_box_center = 0.2F;
  /** Added to ego half-length/width in OBB collision test (~standoff to obstacle surfaces). */
  float obstacle_collision_margin = 0.2F;
  /** Added to the ego footprint when testing collision with road-border segments. */
  float road_border_collision_margin = 0.2F;
  float obstacle_safe_margin = 0.5F;
  float obstacle_barrier_weight = 0.0F;
  float road_border_safe_margin = 0.3F;
  float road_border_barrier_weight = 0.0F;
  float drivable_area_safe_margin = 0.0F;
  float drivable_area_barrier_weight = 2000.0F;
  float crash_contact_penalty = 100000.0F;
};

template <
  class CLASS_T, int NUM_TIMESTEPS,
  class PARAMS_T = FirstOrderDubinsBicycleCostParams<NUM_TIMESTEPS>,
  class DYN_PARAMS_T = FirstOrderDubinsBicycleParams>
class FirstOrderDubinsBicycleCostImpl : public Cost<CLASS_T, PARAMS_T, DYN_PARAMS_T>
{
public:
  static constexpr int kMaxObstacles = 64;
  static constexpr int kMaxDrivablePolygonVertices = 1024;
  static constexpr int kMaxRoadBorderSegments = 256;
  static constexpr int kMaxDrivableAreaSegments = 256;
  /** Full diffusion-path polyline for spatial lateral / crash. */
  static constexpr int kMaxLateralCorridorPoints = 256;

  /**
   * Block shared-memory layout in theta_c (floats):
   *   GRD (all samples share):
   *     [0] total corridor path length
   *     [1] num corridor points (sign encodes has_s)
   *     [2, 2+kMax) corridor_x, then corridor_y, corridor_s
   *     then ref_x/y/v/yaw [NUM_TIMESTEPS each]
   *   BLK (per sample, after float4-aligned GRD):
   *     [0] warm-start segment index for polyline projection (-1 = full scan)
   */
  static constexpr int kSharedTotalOffset = 0;
  static constexpr int kSharedNumCorridorOffset = 1;
  static constexpr int kSharedCorridorXOffset = 2;
  static constexpr int kSharedCorridorYOffset = kSharedCorridorXOffset + kMaxLateralCorridorPoints;
  static constexpr int kSharedCorridorSOffset = kSharedCorridorYOffset + kMaxLateralCorridorPoints;
  static constexpr int kSharedRefXOffset = kSharedCorridorSOffset + kMaxLateralCorridorPoints;
  static constexpr int kSharedRefYOffset = kSharedRefXOffset + NUM_TIMESTEPS;
  static constexpr int kSharedRefVOffset = kSharedRefYOffset + NUM_TIMESTEPS;
  static constexpr int kSharedRefYawOffset = kSharedRefVOffset + NUM_TIMESTEPS;
  static constexpr int kSharedNumFloats = kSharedRefYawOffset + NUM_TIMESTEPS;
  /** Per-sample BLK: previous closest-segment index for warm-started projection. */
  static constexpr int kSharedBlkHintFloats = 1;

  using PARENT_CLASS = Cost<CLASS_T, PARAMS_T, DYN_PARAMS_T>;
  using output_array = typename PARENT_CLASS::output_array;
  using control_array = typename PARENT_CLASS::control_array;

  __host__ FirstOrderDubinsBicycleCostImpl(cudaStream_t stream = 0);

  __host__ ~FirstOrderDubinsBicycleCostImpl() override;

  FirstOrderDubinsBicycleCostImpl(const FirstOrderDubinsBicycleCostImpl &) = delete;
  FirstOrderDubinsBicycleCostImpl & operator=(const FirstOrderDubinsBicycleCostImpl &) = delete;

  /** Stage corridor + time-aligned ref into block shared memory (theta_c). */
  __device__ void initializeCosts(
    float * output, float * control, float * theta_c, float t_0, float dt);

  /** Per-sample BLK slot: previous closest-segment index (-1 = full scan). */
  __device__ float * projectionHintSlot(float * theta_c) const;

  void paramsToDevice();

  void setReferenceTrajectory(
    const float * x, const float * y, const float * v, int count, const float * yaw = nullptr,
    const float * max_velocity = nullptr, const std::uint8_t * velocity_limit_active = nullptr);

  void setKinematicLimits(const FirstOrderDubinsBicycleKinematicLimitData & limits);

  /**
   * Spatial corridor for lateral_distance / lateral crash / lateral yaw error.
   * Prefer the full diffusion path. When unset (< 2 points), those checks fall back
   * to the time-aligned ref_ polyline.
   * @param s optional cumulative chord length [m] per vertex (same length as x/y); nullptr
   *          falls back to segment lengths from xy.
   */
  void setLateralCorridor(const float * x, const float * y, int count, const float * s = nullptr);

  void clearLateralCorridor();

  /** Static obstacles: same pose replicated at every MPPI horizon step. */
  void setOrientedBoxObstacles(
    const float * x, const float * y, const float * yaw, const float * half_length,
    const float * half_width, int count);

  /**
   * Time-varying obstacle poses. All trajectories participate in hard output validation and the
   * gradual obstacle barrier's 3D ESDF.
   */
  void setOrientedBoxObstacleTrajectories(
    const float * x, const float * y, const float * yaw, const float * half_length,
    const float * half_width, int obstacle_count, int num_timesteps);

  void clearObstacles();

  void setRoadBorderSegments(const std::vector<autoware::mppi_optimizer::Segment> & segments);

  void clearRoadBorders();

  void setDrivableAreaSegments(const std::vector<autoware::mppi_optimizer::Segment> & segments);

  void clearDrivableAreaSegments();

  void setDrivableAreaPolygon(const float * x, const float * y, int count);

  void clearDrivableArea();

  /** Enable or disable the CUDA-OpenGL distance-map debug window. */
  __host__ void setDistanceMapTextureDebugEnabled(bool enable);

  /** Refresh the debug window after all distance maps for the current control cycle are ready. */
  __host__ void renderDistanceMapTextureDebug();

  /** Euclidean position error squared: ||p - ref[t]||^2. */
  __host__ __device__ float computeTrackValue(
    float x, float y, int timestep, const float * theta_c = nullptr) const;

  __host__ __device__ float computeHeadingValue(
    float yaw, int timestep, const float * theta_c = nullptr) const;

  /**
   * Cross-track distance to the lateral corridor (or ref_ polyline). Returns signed
   * lateral offset (+ = left of segment tangent); use fabs for magnitude-based checks.
   */
  __host__ __device__ float computeLateralDistanceValue(
    float x, float y, float * theta_c = nullptr) const;

  /**
   * Unified closest-segment projection used when either lateral weight is active.
   * Also stores path length along the corridor chord-length array and remaining distance
   * to the polyline end (used by remaining_distance_coeff).
   * On device, warm-starts from / writes to the per-sample projection hint in theta_c BLK.
   */
  struct LateralPathMetrics
  {
    /** Signed cross-track error [m]; + = left of closest-segment tangent. */
    float lateral_distance = 0.0F;
    float lateral_yaw_error_sq = 0.0F;
    float path_length_s = 0.0F;
    float remaining_distance_s = 0.0F;
    /** Along-track extension past the corridor tip (0 if not past the end). */
    float overshoot_distance_s = 0.0F;
    /** Closest corridor/ref segment index (warm-start seed for the next query). */
    int best_segment_i = 0;
  };

  __host__ __device__ LateralPathMetrics
  computeLateralPathMetrics(float x, float y, float yaw, float * theta_c = nullptr) const;

  /** Squared distance from the ego footprint center to the time-aligned reference sample. */
  __host__ __device__ float computeTrackCenterValue(
    float x, float y, float yaw, int timestep, const float * theta_c = nullptr) const;

  /** Soft clearance cost for ego corners near drivable-area boundary segments. */
  __host__ __device__ float computeCornerBufferCost(float x, float y, float yaw) const;

  /** True if the lateral error exceeds boundary_threshold(_left/_right). */
  __host__ __device__ bool exceedsLateralBoundary(
    const float x, const float y, float * theta_c = nullptr) const;

  __host__ __device__ bool egoIntersectsObstacleAtStep(
    const float x, const float y, const float yaw, int timestep) const;

  /**
   * Signed clearance to the closest obstacle. Host replay uses exact OBB separation; GPU
   * rollout uses the trilinearly interpolated point ESDF minus the ego circumscribed radius.
   */
  __host__ __device__ float distanceToClosestObstacle(
    float x, float y, float yaw, int timestep) const;

  /** Placeholder for ego-footprint collision against static road-border segments. */
  __host__ __device__ bool egoIntersectsRoadBorder(
    const float x, const float y, const float yaw) const;

  /**
   * Ego clearance to the closest road border. The GPU texture query conservatively models the ego
   * footprint by subtracting its circumscribed radius from the point ESDF.
   */
  __host__ __device__ float distanceToRoadBorder(float x, float y, float yaw) const;

  /**
   * Signed clearance to drivable-area segments. GPU rollout uses the point ESDF minus the ego
   * circumscribed radius; host replay retains the exact oriented-box calculation.
   */
  __host__ __device__ float distanceToDrivableArea(float x, float y, float yaw) const;

  __host__ __device__ void computeGradualCrashCosts(
    float x, float y, float yaw, int timestep, float & drivable_area_cost, float & obstacle_cost,
    float & road_border_cost) const;

  float computeStateCost(
    const Eigen::Ref<const output_array> & y, int timestep, int * crash_status);

  __device__ float computeStateCost(float * y, int timestep, float * theta_c, int * crash_status);

  float computeControlCost(const Eigen::Ref<const control_array> & u, int timestep, int * crash);

  __device__ float computeControlCost(float * u, int timestep, float * theta_c, int * crash);

  float computeComfortCost(
    const Eigen::Ref<const control_array> & u, const Eigen::Ref<const output_array> & y,
    int timestep);

  /** Cost components for one host-replayed running step, including gradual static constraints. */
  autoware::mppi_optimizer::FirstOrderDubinsMppiCostBreakdown computeRunningCostBreakdown(
    const Eigen::Ref<const output_array> & y, const Eigen::Ref<const control_array> & u,
    int timestep, int * crash_status) const;

  /** Cost components for the terminal output, before division by the horizon length. */
  autoware::mppi_optimizer::FirstOrderDubinsMppiCostBreakdown computeTerminalCostBreakdown(
    const Eigen::Ref<const output_array> & y) const;

  __device__ float computeComfortCost(float * u, float * y, int timestep);

  __host__ __device__ FirstOrderDubinsBicycleKinematicCost computeKinematicLimitCost(
    float velocity, float longitudinal_acceleration, float longitudinal_jerk, int timestep) const;

  __device__ float terminalCost(float * y, float * theta_c);

  float computeRunningCost(
    const Eigen::Ref<const output_array> & y, const Eigen::Ref<const control_array> & u,
    int timestep, int * crash);

  __device__ float computeRunningCost(
    float * y, float * u, int timestep, float * theta_c, int * crash);

  float ref_x_[NUM_TIMESTEPS] = {};
  float ref_y_[NUM_TIMESTEPS] = {};
  float ref_v_[NUM_TIMESTEPS] = {};
  float ref_yaw_[NUM_TIMESTEPS] = {};
  float ref_max_velocity_[NUM_TIMESTEPS] = {};
  std::uint8_t ref_velocity_limit_active_[NUM_TIMESTEPS] = {};
  bool has_pointwise_velocity_limits_{false};
  FirstOrderDubinsBicycleKinematicLimitData kinematic_limits_{};
  int num_lateral_corridor_points_ = 0;
  float lateral_corridor_x_[kMaxLateralCorridorPoints] = {};
  float lateral_corridor_y_[kMaxLateralCorridorPoints] = {};
  /** Cumulative chord length [m] along lateral_corridor_* (s[0]=0); valid when count >= 1. */
  float lateral_corridor_s_[kMaxLateralCorridorPoints] = {};
  /** Final path length = s[n-1]; staged into shared memory with corridor/ref on GPU. */
  float lateral_corridor_total_length_s_ = 0.0F;
  bool lateral_corridor_has_s_ = false;
  int num_obstacles_ = 0;
  float obs_x_[kMaxObstacles][NUM_TIMESTEPS] = {};
  float obs_y_[kMaxObstacles][NUM_TIMESTEPS] = {};
  float obs_yaw_[kMaxObstacles][NUM_TIMESTEPS] = {};
  float obs_half_length_[kMaxObstacles] = {};
  float obs_half_width_[kMaxObstacles] = {};
  /** True when the obstacle pose is invariant across the supplied horizon. */
  bool obs_is_static_[kMaxObstacles] = {};
  int num_road_border_segments_ = 0;
  float road_border_x0_[kMaxRoadBorderSegments] = {};
  float road_border_y0_[kMaxRoadBorderSegments] = {};
  float road_border_x1_[kMaxRoadBorderSegments] = {};
  float road_border_y1_[kMaxRoadBorderSegments] = {};
  int num_drivable_area_segments_ = 0;
  float drivable_area_x0_[kMaxDrivableAreaSegments] = {};
  float drivable_area_y0_[kMaxDrivableAreaSegments] = {};
  float drivable_area_x1_[kMaxDrivableAreaSegments] = {};
  float drivable_area_y1_[kMaxDrivableAreaSegments] = {};

  /**
   * GPU ESDF texture state. The 2D float2 texture stores road-border distance in x and
   * drivable-boundary distance in y. The 3D texture stores obstacle distance over the horizon.
   * The host-owned CUDA arrays are surface-written by generation kernels and sampled through
   * texture handles copied into cost_d_.
   */
  DistanceMapTextureGrid static_distance_map_grid_{};
  DistanceMapTextureGrid obstacle_distance_map_grid_{};
  cudaTextureObject_t static_distance_texture_ = 0;
  cudaTextureObject_t obstacle_distance_texture_ = 0;
  bool road_border_texture_valid_ = false;
  bool drivable_area_texture_valid_ = false;
  bool obstacle_texture_valid_ = false;
  bool obstacle_texture_has_obstacles_ = false;

private:
  void dataToDevice();
  __host__ bool updateDistanceMapGrid(
    DistanceMapTextureGrid & grid, int width, int height, float resolution);
  __host__ void ensureDistanceMapResources();
  __host__ void rebuildStaticDistanceTexture(bool update_road_border, bool update_drivable_area);
  __host__ void rebuildObstacleDistanceTexture();
  __host__ void distanceMapStateToDevice();
  __host__ void releaseDistanceMapResources();

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

template <int NUM_TIMESTEPS>
class FirstOrderDubinsBicycleCost
: public FirstOrderDubinsBicycleCostImpl<FirstOrderDubinsBicycleCost<NUM_TIMESTEPS>, NUM_TIMESTEPS>
{
public:
  FirstOrderDubinsBicycleCost(cudaStream_t stream = 0)
  : FirstOrderDubinsBicycleCostImpl<FirstOrderDubinsBicycleCost<NUM_TIMESTEPS>, NUM_TIMESTEPS>(
      stream)
  {
  }
};

#if __CUDACC__
#include "first_order_dubins_bicycle_cost.cu"
#endif

#endif  // MPPI_COST_FUNCTIONS_FIRST_ORDER_DUBINS_BICYCLE_COST_CUH_
