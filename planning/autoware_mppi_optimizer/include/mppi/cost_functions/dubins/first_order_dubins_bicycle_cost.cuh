/**
 * Analytic path-tracking cost for FirstOrderDubinsBicycle (reference polyline + parked-car OBBs).
 */
#pragma once

#ifndef MPPI_COST_FUNCTIONS_FIRST_ORDER_DUBINS_BICYCLE_COST_CUH_
#define MPPI_COST_FUNCTIONS_FIRST_ORDER_DUBINS_BICYCLE_COST_CUH_

#include "autoware/mppi_optimizer/first_order_dubins_mppi_interface.hpp"

#include <mppi/cost_functions/cost.cuh>
#include <mppi/dynamics/dubins/first_order_dubins_bicycle.cuh>

template <int NUM_TIMESTEPS>
struct FirstOrderDubinsBicycleCostParams : public CostParams<2>
{
  float desired_speed = 2.5F;
  float speed_coeff = 500.0F;
  float track_coeff = 1000.0F;
  /** Multiplier on track_coeff * track_val in terminalCost (running state cost uses scale 1). */
  float track_terminal_scale = 10.0F;
  /** Pull toward ref heading at each horizon step: coeff * (yaw - ref_yaw[t])^2; 0 disables. */
  float heading_coeff = 500.0F;
  /** Spatial (closest-segment) distance to the reference polyline; 0 disables. */
  float lateral_distance_coeff = 0.0F;
  /** Spatial yaw error vs closest-segment tangent: coeff * Δψ^2; 0 disables. */
  float lateral_yaw_error_coeff = 0.0F;
  /** Per-violation crash penalty; latched crash_status counts violations (1=lateral bound or hit,
   * 2=both). */
  float crash_coeff = 100000.0F;
  float boundary_threshold = 0.8F;
  /** Beyond bound if signed lateral offset exceeds these (path-left = +); <0 falls back to
   * boundary_threshold. */
  float boundary_threshold_left = -1.0F;
  float boundary_threshold_right = -1.0F;
  float accel_cmd_coeff = 0.0F;
  float steer_cmd_coeff = 0.0F;
  /** Deadband-quadratic kinematic constraint limits and weights. */
  float max_velocity = 10.0F;
  float max_lon_accel = 2.0F;
  float min_lon_accel = -2.0F;
  float max_lon_jerk = 3.0F;
  float max_lat_accel = 2.0F;
  float max_lat_jerk = 3.0F;
  /** W_overspeed, independent from speed_coeff tracking against ref_v_[t]. */
  float overspeed_coeff = 10000.0F;
  float longitudinal_acceleration_coeff = 10000.0F;
  /** Deadband-quadratic cost beyond the configured steering-rate limit. */
  float steer_rate_coeff = 0.0F;
  float lateral_acceleration_coeff = 10000.0F;
  float lateral_jerk_coeff = 10000.0F;
  float longitudinal_jerk_coeff = 10000.0F;
  float longitudinal_recovery_coeff = 20000.0F;
  float longitudinal_recovery_time_constant = 0.5F;
  float wheel_base = 0.32F;
  float accel_time_constant = 0.15F;
  float steer_time_constant = 0.08F;
  float max_steer_rate = 3.0F;
  /** Ego OBB for parked-car collision (rear axle at pose; box center offset forward). */
  float ego_length = 0.55F * 1.5F;
  float ego_width = 0.28F * 1.5F;
  float ego_axle_to_box_center = 0.2F;
  /** Added to ego half-length/width in OBB collision test (~standoff to obstacle surfaces). */
  float obstacle_collision_margin = 0.2F;
  /** Added to the ego footprint when testing collision with road-border segments. */
  float road_border_collision_margin = 0.2F;
  /** Per-timestep soft cost when the ego footprint crosses a drivable-area boundary. */
  float drivable_area_crossing_coeff = 10000.0F;
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

  using PARENT_CLASS = Cost<CLASS_T, PARAMS_T, DYN_PARAMS_T>;
  using output_array = typename PARENT_CLASS::output_array;
  using control_array = typename PARENT_CLASS::control_array;

  FirstOrderDubinsBicycleCostImpl(cudaStream_t stream = 0);

  void paramsToDevice();

  void setReferenceTrajectory(
    const float * x, const float * y, const float * v, int count, const float * yaw = nullptr);

  /**
   * Set the host-precomputed per-step effective limits and comfort profile.
   *
   * Weight entries are multipliers in [0, 1]. A zero longitudinal multiplier is a
   * safety-stop exemption; an attenuated lateral multiplier is a high-curvature exemption.
   * Effective limits have already resolved static, external, and map sources on the host.
   */
  void setKinematicConstraintProfile(
    const float * lon_comfort_weight, const float * lat_comfort_weight,
    const float * max_velocity_limit, const float * min_lon_accel_limit,
    const float * max_lon_accel_limit, const float * max_lon_jerk_limit,
    const float * max_lat_accel_limit, const float * max_lat_jerk_limit,
    const float * accel_recovery_limit, const float * accel_recovery_weight, int count);

  /** Upload the constraint profile once after all host preparation is complete. */
  void uploadKinematicConstraintProfile();

  /** Static obstacles: same pose replicated at every MPPI horizon step. */
  void setOrientedBoxObstacles(
    const float * x, const float * y, const float * yaw, const float * half_length,
    const float * half_width, int count);

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

  /** Euclidean position error to the time-aligned reference sample ref[t]. */
  __host__ __device__ float computeTrackValue(float x, float y, int timestep) const;

  __host__ __device__ float computeHeadingValue(float yaw, int timestep) const;

  /**
   * Pre time-indexed tracking: min Euclidean distance from (x,y) to the reference
   * polyline (closest segment). Used by lateral_distance_coeff.
   */
  __host__ __device__ float computeLateralDistanceValue(float x, float y) const;

  /**
   * Pre time-indexed heading: squared yaw error vs the tangent of the closest
   * reference segment. Used by lateral_yaw_error_coeff.
   */
  __host__ __device__ float computeLateralYawErrorValue(float x, float y, float yaw) const;

  /** Signed lateral error from ref[t], resolved in ref_yaw[t] (+ = reference-left). */
  __host__ __device__ float computeSignedLateralOffset(float x, float y, int timestep) const;

  /** True if the time-aligned lateral error exceeds boundary_threshold(_left/_right). */
  __host__ __device__ bool exceedsLateralBoundary(const float x, const float y, int timestep) const;

  __host__ __device__ bool egoIntersectsObstacleAtStep(
    const float x, const float y, const float yaw, int timestep) const;

  /** Placeholder for ego-footprint collision against static road-border segments. */
  __host__ __device__ bool egoIntersectsRoadBorder(
    const float x, const float y, const float yaw) const;

  /** Placeholder for ego-footprint crossing of drivable-area boundary segments. */
  __host__ __device__ bool egoCrossesDrivableAreaBoundary(
    const float x, const float y, const float yaw) const;

  __host__ __device__ bool isCrashLatched(const int * crash_status) const;

  __host__ __device__ bool detectAndLatchCrash(
    const float x, const float y, const float yaw, int timestep, int * crash_status) const;

  __host__ __device__ float latchedCrashCost(const int * crash_status) const;

  float computeStateCost(
    const Eigen::Ref<const output_array> & y, int timestep, int * crash_status);

  __device__ float computeStateCost(float * y, int timestep, float * theta_c, int * crash_status);

  float computeControlCost(const Eigen::Ref<const control_array> & u, int timestep, int * crash);

  __device__ float computeControlCost(float * u, int timestep, float * theta_c, int * crash);

  float computeComfortCost(
    const Eigen::Ref<const control_array> & u, const Eigen::Ref<const output_array> & y,
    int timestep);

  __device__ float computeComfortCost(float * u, float * y, int timestep);

  __host__ __device__ float computeKinematicBarrierCost(
    float velocity, float longitudinal_accel, float longitudinal_jerk, float lateral_accel,
    float lateral_jerk, float steer_rate, int timestep) const;

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
  struct KinematicConstraintProfileData
  {
    float lon_comfort_weight[NUM_TIMESTEPS] = {};
    float lat_comfort_weight[NUM_TIMESTEPS] = {};
    float max_velocity_limit[NUM_TIMESTEPS] = {};
    float min_lon_accel_limit[NUM_TIMESTEPS] = {};
    float max_lon_accel_limit[NUM_TIMESTEPS] = {};
    float max_lon_jerk_limit[NUM_TIMESTEPS] = {};
    float max_lat_accel_limit[NUM_TIMESTEPS] = {};
    float max_lat_jerk_limit[NUM_TIMESTEPS] = {};
    float accel_recovery_limit[NUM_TIMESTEPS] = {};
    float accel_recovery_weight[NUM_TIMESTEPS] = {};
  };
  KinematicConstraintProfileData kinematic_constraint_profile_{};
  int num_obstacles_ = 0;
  float obs_x_[kMaxObstacles][NUM_TIMESTEPS] = {};
  float obs_y_[kMaxObstacles][NUM_TIMESTEPS] = {};
  float obs_yaw_[kMaxObstacles][NUM_TIMESTEPS] = {};
  float obs_half_length_[kMaxObstacles] = {};
  float obs_half_width_[kMaxObstacles] = {};
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
  int num_drivable_vertices_ = 0;
  float drivable_poly_x_[kMaxDrivablePolygonVertices] = {};
  float drivable_poly_y_[kMaxDrivablePolygonVertices] = {};

private:
  void referenceDataToDevice();
  void kinematicConstraintProfileDataToDevice();
  void obstacleDataToDevice();
  void roadBorderDataToDevice();
  void drivableAreaSegmentDataToDevice();
  void drivableAreaPolygonDataToDevice();
  bool kinematic_constraint_profile_dirty_{true};
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
