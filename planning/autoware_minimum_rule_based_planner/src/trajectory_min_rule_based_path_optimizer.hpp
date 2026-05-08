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

#ifndef TRAJECTORY_MIN_RULE_BASED_PATH_OPTIMIZER_HPP_
#define TRAJECTORY_MIN_RULE_BASED_PATH_OPTIMIZER_HPP_

#include "acados_interface_time.hpp"

#include <autoware/trajectory/trajectory_point.hpp>
#include <autoware/trajectory_optimizer/trajectory_optimizer_plugins/trajectory_optimizer_plugin_base.hpp>
#include <autoware/vehicle_info_utils/vehicle_info_utils.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_planning_msgs/msg/trajectory_point.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <memory>
#include <optional>
#include <vector>

namespace autoware::trajectory_optimizer::plugin
{
using autoware_planning_msgs::msg::TrajectoryPoint;
using TrajectoryPoints = std::vector<TrajectoryPoint>;

struct MinRuleBasedPathOptimizerParams
{
  // Maximum physical time per stage [s]. The runtime dt is the smaller of
  // dt_max_s and (s_max / (N * v_ref)) so that the horizon roughly ends at
  // the input's last point under the input's nominal speed.
  double dt_max_s{0.1};

  // Velocity reference floor [m/s]. Used when the input trajectory's average
  // longitudinal_velocity_mps is below this value (or zero), to avoid
  // degenerate dt sizing.
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
 * On solver failure or large deviation, traj_points is left unchanged.
 */
class TrajectoryMinRuleBasedPathOptimizer : public TrajectoryOptimizerPluginBase
{
public:
  TrajectoryMinRuleBasedPathOptimizer() = default;
  ~TrajectoryMinRuleBasedPathOptimizer() override = default;

  void optimize_trajectory(
    TrajectoryPoints & traj_points, const TrajectoryOptimizerParams & params,
    TrajectoryOptimizerData & data) override;

  void set_up_params() override;

  rcl_interfaces::msg::SetParametersResult on_parameter(
    const std::vector<rclcpp::Parameter> & parameters) override;

private:
  MinRuleBasedPathOptimizerParams plugin_params_;
  autoware::vehicle_info_utils::VehicleInfo vehicle_info_;

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

  // World-frame trajectory of the previous successful solve. We use this as
  // the anchor source for the next cycle: projecting the current ego onto it
  // gives a (X0, Y0, ψ0) that is consistent with the path we just produced,
  // which avoids the "snap-to-input-projection" jitter when the optimizer
  // wants to deviate from the input geometry. Falls back to the input
  // projection if absent or if ego has drifted too far from this path.
  std::optional<autoware::experimental::trajectory::Trajectory<TrajectoryPoint>>
    last_world_sol_traj_;

  // Debug visualization: anchor (X0, Y0) as a green sphere and per-stage
  // (Xref_k, Yref_k) reference points as red spheres. Only populated and
  // published when `debug.enable_debug_topics` is true.
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_debug_markers_;
};

}  // namespace autoware::trajectory_optimizer::plugin

#endif  // TRAJECTORY_MIN_RULE_BASED_PATH_OPTIMIZER_HPP_
