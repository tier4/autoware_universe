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

#ifndef PATH_OPTIMIZER_HPP_
#define PATH_OPTIMIZER_HPP_

#include "acados_interface_time.hpp"

#include <autoware/trajectory/trajectory_point.hpp>
#include <autoware/vehicle_info_utils/vehicle_info_utils.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_planning_msgs/msg/trajectory_point.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <array>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace autoware::minimum_rule_based_planner
{
using autoware_planning_msgs::msg::TrajectoryPoint;
using nav_msgs::msg::Odometry;
using TrajectoryPoints = std::vector<TrajectoryPoint>;

// 2D segment used to feed uncrossable_boundary geometry from the planner side
// (which owns the cached lanelet linestrings) into the optimizer. Only the
// (x, y) of the endpoints is used; z is ignored.
struct UncrossableSegment
{
  geometry_msgs::msg::Point p_a;
  geometry_msgs::msg::Point p_b;
};

// 3-circle vehicle footprint approximation used for boundary collision
// distances. Offsets are along the body x axis from base_link (= rear axle in
// the kinematic bicycle model used here). All three circles share the same
// radius so the same `r + margin` clearance applies uniformly.
struct VehicleCircle
{
  double lon_offset_m{0.0};
  double radius_m{0.0};
};

struct MinRuleBasedPathOptimizerParams
{
  // Two-stage time grid: stages [0, near_stage_count) advance dt_near_s
  // seconds each (dense near-field sampling), stages [near_stage_count, N)
  // advance dt_far_s seconds each (coarse far-field sampling). Total physical
  // horizon is `near_stage_count · dt_near_s + (N − near_stage_count) · dt_far_s`.
  double dt_near_s{0.05};
  double dt_far_s{0.4};
  int near_stage_count{20};

  // Velocity reference floor [m/s]. Used when the input trajectory's average
  // longitudinal_velocity_mps is below this value (or zero), to avoid
  // degenerate v_ref sampling.
  double v_ref_floor_mps{0.5};

  // Input box on |dδ/dt| [rad/s] (steering rate, physical units).
  double u_delta_max_rad_per_s{1.0};

  // Input box on longitudinal velocity v [m/s].
  double v_min_mps{0.0};
  double v_max_mps{20.0};

  // Margin subtracted from vehicle_info.max_steer_angle_rad for numerical safety.
  double steer_margin_rad{0.02};

  // ---- Cost weights (diagonal) ----
  double w_stage_position_xy{0.05};
  double w_stage_yaw{0.01};
  double w_stage_delta{0.5};
  double w_stage_steer_rate{5.0};  // weight on u_delta = dδ/dt
  double w_stage_velocity{0.1};    // weight on (v - v_ref)

  double w_terminal_position_xy{10.0};
  double w_terminal_yaw{2.0};
  double w_terminal_delta{0.1};

  // Warm-start reset thresholds.
  bool warm_start_enable{true};
  double reset_pose_threshold_m{5.0};
  double reset_yaw_threshold_rad{0.785};

  // Validation: discard solution when lateral / yaw deviation from input is too large.
  double max_lat_error_from_input_m{3.0};
  double max_yaw_error_from_input_rad{0.7};

  // Soft monitoring.
  double elapsed_time_warn_threshold_ms{30.0};
  bool publish_replay_fixture_on_failure{false};
  bool enable_debug_topics{false};
};

// Uncrossable_boundary collision-avoidance constraint selection parameters.
// At this stage the parameters drive only the per-stage active-set selection
// and its visualization; the acados OCP itself is not yet aware of these
// constraints. See my_docs/min_rule_based_path_optimizer_uncrossable_boundary_constraints.md.
struct UncrossableBoundaryParams
{
  bool enable{true};
  // Drop threshold on min-circle perpendicular clearance. Segments where even
  // the closest circle has more than this clearance are pruned (cannot go
  // active over the SQP step). This is the single retention radius.
  double clearance_drop_threshold_m{2.0};
  // Number of azimuthal bins around the stage centre = the maximum number of
  // segments per stage. Each bin keeps its closest segment.
  int K_max_per_stage{4};
  // Extra margin added to the circle radius when forming the half-space
  // clearance (the visualisation half-plane line is offset by this).
  double margin_m{0.10};
};

/**
 * @brief Time-axis kinematic-feasibility path optimizer (acados, full SQP).
 *
 * Modifies path shape (x, y, yaw) only. Velocity/acceleration/time of the output
 * trajectory points are passed through. The model integrates over physical time
 * with two free inputs — steering rate u_delta = dδ/dt and longitudinal speed v —
 * which lets the solver choose the actual arc-length traversed; this fixes the
 * earlier failure mode where a fixed arc-length parametrisation prevented the
 * path from reaching the target pose when significant reshaping was required.
 *
 * On solver failure or large deviation, the previous successful output is
 * reused (cropped from ego onward) when ego is still on it; otherwise
 * traj_points is left unchanged so downstream falls back to the input.
 */
class PathOptimizer
{
public:
  PathOptimizer(const std::string & name, rclcpp::Node * node_ptr);
  ~PathOptimizer() = default;

  void optimize_trajectory(
    TrajectoryPoints & traj_points, const Odometry & odometry,
    const std::vector<UncrossableSegment> & uncrossable_segments);

  rcl_interfaces::msg::SetParametersResult on_parameter(
    const std::vector<rclcpp::Parameter> & parameters);

  const std::string & get_name() const { return name_; }

private:
  void set_up_params();

  // Result of per-stage active-set selection. At this stage consumed only by
  // the debug visualiser; will be the source of acados con_h parameters once
  // the OCP-side wiring is added.
  struct SelectedHalfSpace
  {
    geometry_msgs::msg::Point p_a;     // segment endpoint A
    geometry_msgs::msg::Point p_b;     // segment endpoint B
    geometry_msgs::msg::Point anchor;  // closest point on segment to the
                                       // most-active circle centre
    double n_x{0.0};                   // inward normal (toward warm-start)
    double n_y{0.0};
    double min_clearance_m{0.0};  // (min over circles) of perp distance,
                                  // already shrunk by r (vehicle circle).
    int azimuth_bin{-1};          // bin index in [0, K_max_per_stage)
  };

  // Replaces traj_points with the previous successful output cropped from the
  // point nearest to ego. Returns true if applied, false if no usable previous
  // output (none stored, or ego is more than the threshold away from any
  // point on it).
  bool try_apply_previous_output_fallback(
    TrajectoryPoints & traj_points, const geometry_msgs::msg::Pose & ego_pose) const;

  // Per-stage active-set selection (Layers 3-6 of the uncrossable_boundary
  // pipeline). `warm_start_circles` are the 3 vehicle-circle centres in map
  // frame at the warm-start state of one stage; the middle circle is used as
  // the stage centre for azimuth binning. Returns up to
  // uncrossable_params_.K_max_per_stage half-spaces; pads no dummies.
  std::vector<SelectedHalfSpace> select_uncrossable_active_set_for_stage(
    const std::array<geometry_msgs::msg::Point, 3> & warm_start_circles,
    const std::vector<UncrossableSegment> & segments) const;

  std::string name_;
  rclcpp::Node * node_ptr_{nullptr};

  MinRuleBasedPathOptimizerParams plugin_params_;
  UncrossableBoundaryParams uncrossable_params_;
  autoware::vehicle_info_utils::VehicleInfo vehicle_info_;
  // 3-circle vehicle footprint approximation. Computed once in set_up_params
  // from vehicle_info; uniform tiling so the same radius covers each third of
  // the vehicle length plus the half-width.
  std::array<VehicleCircle, 3> vehicle_circles_{};

  std::unique_ptr<time_mpt::AcadosInterfaceTime> acados_;

  // Warm-start state from previous solve.
  bool has_last_solution_{false};
  time_mpt::AcadosTimeSolution last_solution_;
  // Anchor of the previous solve in map frame, for jump detection.
  double last_x0_world_x_{0.0};
  double last_x0_world_y_{0.0};
  double last_x0_world_yaw_{0.0};
  // dt used in the previous solve; a significant change forces a warm-start reset
  // because last_solution_'s xtraj is sampled on the previous time grid.
  double last_dt_runtime_{0.0};

  // Last successfully produced output, as a Trajectory<TrajectoryPoint>
  // (poses from the optimizer + velocities from the input it was built
  // against). Single source of truth for two things:
  //   - warm-start anchor: project ego onto it to get (X0, Y0, ψ0) that is
  //     consistent with the path we just produced, avoiding the
  //     "snap-to-input-projection" jitter when the optimizer wants to
  //     deviate from the input geometry.
  //   - failure fallback: if ego is still on it within
  //     kFallbackMaxDistFromPrevSolM, crop it from the nearest projection
  //     onward and emit that instead of the raw input.
  std::optional<autoware::experimental::trajectory::Trajectory<TrajectoryPoint>> last_output_traj_;

  // Debug visualization: anchor (X0, Y0) as a green sphere and per-stage
  // (Xref_k, Yref_k) reference points as red spheres. Only populated and
  // published when `debug.enable_debug_topics` is true.
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_debug_markers_;
};

}  // namespace autoware::minimum_rule_based_planner

#endif  // PATH_OPTIMIZER_HPP_
