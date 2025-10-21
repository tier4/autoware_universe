// Copyright 2025 TIER IV, Inc.
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

#include "autoware/trajectory_optimizer/trajectory_optimizer_plugins/trajectory_mpt_optimizer.hpp"

#include "autoware/trajectory_optimizer/utils.hpp"

#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils_math/unit_conversion.hpp>
#include <autoware_utils_rclcpp/parameter.hpp>

#include <cmath>
#include <memory>
#include <string>
#include <vector>

namespace autoware::trajectory_optimizer::plugin
{

TrajectoryMPTOptimizer::TrajectoryMPTOptimizer(
  const std::string name, rclcpp::Node * node_ptr,
  const std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper,
  const TrajectoryOptimizerParams & params)
: TrajectoryOptimizerPluginBase(name, node_ptr, time_keeper, params)
{
  // Get vehicle info
  vehicle_info_ = autoware::vehicle_info_utils::VehicleInfoUtils(*node_ptr).getVehicleInfo();

  // Initialize debug data
  debug_data_ptr_ = std::make_shared<autoware::path_optimizer::DebugData>();

  // Set up parameters
  set_up_params();

  // Initialize MPT optimizer
  mpt_optimizer_ptr_ = std::make_shared<autoware::path_optimizer::MPTOptimizer>(
    node_ptr, mpt_params_.enable_debug_info, ego_nearest_param_, vehicle_info_, traj_param_,
    debug_data_ptr_, time_keeper);

  // Subscribe to lanelet map (within plugin, not main node)
  map_sub_ = node_ptr->create_subscription<autoware_map_msgs::msg::LaneletMapBin>(
    "/map/vector_map", rclcpp::QoS{1}.transient_local(),
    std::bind(&TrajectoryMPTOptimizer::on_map, this, std::placeholders::_1));
}

void TrajectoryMPTOptimizer::set_up_params()
{
  auto node_ptr = get_node_ptr();
  using autoware_utils_rclcpp::get_or_declare_parameter;

  // MPT-specific parameters
  mpt_params_.bounds_lateral_offset_m =
    get_or_declare_parameter<double>(*node_ptr, "trajectory_mpt_optimizer.bounds_lateral_offset_m");
  mpt_params_.enable_debug_info =
    get_or_declare_parameter<bool>(*node_ptr, "trajectory_mpt_optimizer.enable_debug_info");

  // Trajectory parameters (required by MPT optimizer)
  traj_param_.output_delta_arc_length = get_or_declare_parameter<double>(
    *node_ptr, "trajectory_mpt_optimizer.output_delta_arc_length_m");
  traj_param_.output_backward_traj_length = get_or_declare_parameter<double>(
    *node_ptr, "trajectory_mpt_optimizer.output_backward_traj_length_m");

  // Ego nearest parameters (required by MPT optimizer)
  ego_nearest_param_.dist_threshold = get_or_declare_parameter<double>(
    *node_ptr, "trajectory_mpt_optimizer.ego_nearest_dist_threshold_m");
  const double ego_nearest_yaw_threshold_deg = get_or_declare_parameter<double>(
    *node_ptr, "trajectory_mpt_optimizer.ego_nearest_yaw_threshold_deg");
  ego_nearest_param_.yaw_threshold = autoware_utils_math::deg2rad(ego_nearest_yaw_threshold_deg);
}

rcl_interfaces::msg::SetParametersResult TrajectoryMPTOptimizer::on_parameter(
  const std::vector<rclcpp::Parameter> & parameters)
{
  using autoware_utils_rclcpp::update_param;

  update_param(
    parameters, "trajectory_mpt_optimizer.bounds_lateral_offset_m",
    mpt_params_.bounds_lateral_offset_m);
  update_param(
    parameters, "trajectory_mpt_optimizer.enable_debug_info", mpt_params_.enable_debug_info);

  update_param(
    parameters, "trajectory_mpt_optimizer.output_delta_arc_length_m",
    traj_param_.output_delta_arc_length);
  update_param(
    parameters, "trajectory_mpt_optimizer.output_backward_traj_length_m",
    traj_param_.output_backward_traj_length);

  update_param(
    parameters, "trajectory_mpt_optimizer.ego_nearest_dist_threshold_m",
    ego_nearest_param_.dist_threshold);

  // Handle yaw threshold with deg2rad conversion
  double ego_nearest_yaw_threshold_deg = 0.0;
  if (update_param(
        parameters, "trajectory_mpt_optimizer.ego_nearest_yaw_threshold_deg",
        ego_nearest_yaw_threshold_deg)) {
    ego_nearest_param_.yaw_threshold = autoware_utils_math::deg2rad(ego_nearest_yaw_threshold_deg);
  }

  // Propagate parameter updates to MPT optimizer
  if (mpt_optimizer_ptr_) {
    mpt_optimizer_ptr_->onParam(parameters);
  }

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  return result;
}

void TrajectoryMPTOptimizer::on_map(const autoware_map_msgs::msg::LaneletMapBin::ConstSharedPtr msg)
{
  lanelet_map_ptr_ = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::fromBinMsg(*msg, lanelet_map_ptr_);
  RCLCPP_DEBUG(
    get_node_ptr()->get_logger(), "TrajectoryMPTOptimizer: Received lanelet map with %zu lanelets",
    lanelet_map_ptr_->laneletLayer.size());
}

BoundsPair TrajectoryMPTOptimizer::generate_bounds_from_lanelet_map(
  const TrajectoryPoints & traj_points) const
{
  BoundsPair bounds;

  if (!lanelet_map_ptr_) {
    return bounds;
  }

  bounds.left_bound.reserve(traj_points.size());
  bounds.right_bound.reserve(traj_points.size());

  // Get all lanelets once
  const auto all_lanelets = lanelet::utils::query::laneletLayer(lanelet_map_ptr_);

  // Optimize: Query lanelet once, reuse for nearby consecutive points
  lanelet::ConstLanelet current_lanelet;
  bool has_valid_lanelet = false;
  size_t points_since_last_query = 0;
  constexpr size_t max_points_per_lanelet = 20;  // Re-query every 20 points as safety

  for (const auto & point : traj_points) {
    // Re-query lanelet periodically or if we don't have one yet
    if (!has_valid_lanelet || points_since_last_query >= max_points_per_lanelet) {
      if (!lanelet::utils::query::getClosestLanelet(all_lanelets, point.pose, &current_lanelet)) {
        RCLCPP_WARN_THROTTLE(
          get_node_ptr()->get_logger(), *get_node_ptr()->get_clock(), 5000,
          "MPT Optimizer: Could not find closest lanelet, using fallback");
        return BoundsPair{};
      }
      has_valid_lanelet = true;
      points_since_last_query = 0;
    }
    points_since_last_query++;

    // Get bounds from current lanelet
    const auto left_bound_3d = current_lanelet.leftBound();
    const auto right_bound_3d = current_lanelet.rightBound();

    // Use middle point of each bound for simplicity and speed
    const size_t left_idx = left_bound_3d.size() / 2;
    const size_t right_idx = right_bound_3d.size() / 2;

    bounds.left_bound.push_back(lanelet::utils::conversion::toGeomMsgPt(left_bound_3d[left_idx]));
    bounds.right_bound.push_back(
      lanelet::utils::conversion::toGeomMsgPt(right_bound_3d[right_idx]));
  }

  return bounds;
}

void TrajectoryMPTOptimizer::optimize_trajectory(
  TrajectoryPoints & traj_points, const TrajectoryOptimizerParams & params,
  const TrajectoryOptimizerData & data)
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *get_time_keeper());

  // Skip if MPT optimizer is disabled
  if (!params.use_mpt_optimizer) {
    return;
  }

  // Minimum points required for optimization
  constexpr size_t min_points_for_optimization = 10;
  if (traj_points.size() < min_points_for_optimization) {
    RCLCPP_DEBUG_THROTTLE(
      get_node_ptr()->get_logger(), *get_node_ptr()->get_clock(), 5000,
      "MPT Optimizer: Trajectory too short (< %zu points), skipping optimization",
      min_points_for_optimization);
    return;
  }

  // Generate bounds - use lanelet map if available, otherwise use perpendicular offset
  const auto bounds = [&]() {
    if (lanelet_map_ptr_) {
      auto lanelet_bounds = generate_bounds_from_lanelet_map(traj_points);
      if (!lanelet_bounds.left_bound.empty() && !lanelet_bounds.right_bound.empty()) {
        RCLCPP_DEBUG(get_node_ptr()->get_logger(), "MPT Optimizer: Using lanelet-based bounds");
        return lanelet_bounds;
      }
      RCLCPP_WARN_THROTTLE(
        get_node_ptr()->get_logger(), *get_node_ptr()->get_clock(), 5000,
        "MPT Optimizer: Lanelet bounds extraction failed, falling back to perpendicular bounds");
    }
    return generate_bounds_from_trajectory(traj_points, mpt_params_.bounds_lateral_offset_m);
  }();

  // Create planner data
  const auto planner_data = create_planner_data(traj_points, bounds, data);

  // Debug logging (only at DEBUG level)
  RCLCPP_DEBUG(
    get_node_ptr()->get_logger(),
    "MPT Optimizer Input: traj_points=%zu, left_bound=%zu, right_bound=%zu, ego_vel=%.2f m/s",
    planner_data.traj_points.size(), planner_data.left_bound.size(),
    planner_data.right_bound.size(), planner_data.ego_vel);

  // Store original size for validation
  const size_t original_size = traj_points.size();

  // Reset previous optimization data since diffusion planner generates new trajectories each cycle
  // Without this, MPT warm start uses stale data from previous (different) trajectories
  mpt_optimizer_ptr_->resetPreviousData();

  // Run MPT optimization
  const auto optimized_traj = mpt_optimizer_ptr_->optimizeTrajectory(planner_data);

  // Apply optimized trajectory if successful with validation
  if (optimized_traj) {
    // Validate optimized trajectory
    if (optimized_traj->empty()) {
      RCLCPP_WARN_THROTTLE(
        get_node_ptr()->get_logger(), *get_node_ptr()->get_clock(), 5000,
        "MPT Optimizer: Returned empty trajectory, keeping original");
      return;
    }

    if (optimized_traj->size() < min_points_for_optimization) {
      RCLCPP_WARN_THROTTLE(
        get_node_ptr()->get_logger(), *get_node_ptr()->get_clock(), 5000,
        "MPT Optimizer: Returned trajectory too short (%zu < %zu), keeping original",
        optimized_traj->size(), min_points_for_optimization);
      return;
    }

    // Log output trajectory info at DEBUG level
    RCLCPP_DEBUG(
      get_node_ptr()->get_logger(), "MPT Output: Successfully optimized %zu → %zu points",
      original_size, optimized_traj->size());

    traj_points = *optimized_traj;

    // CRITICAL: MPT does not set acceleration field, recalculate from velocities
    // Without this, acceleration will be 0.0 and vehicle won't move!
    autoware::trajectory_optimizer::utils::recalculate_longitudinal_acceleration(
      traj_points, false);
  } else {
    RCLCPP_DEBUG_THROTTLE(
      get_node_ptr()->get_logger(), *get_node_ptr()->get_clock(), 5000,
      "MPT Optimizer: Optimization failed, keeping original trajectory");
  }
}

BoundsPair TrajectoryMPTOptimizer::generate_bounds_from_trajectory(
  const TrajectoryPoints & traj_points, const double lateral_offset_m) const
{
  BoundsPair bounds;
  bounds.left_bound.reserve(traj_points.size());
  bounds.right_bound.reserve(traj_points.size());

  for (size_t i = 0; i < traj_points.size(); ++i) {
    const auto & point = traj_points[i];
    const auto & pose = point.pose;

    // Get yaw from quaternion
    const double yaw = tf2::getYaw(pose.orientation);

    // Calculate perpendicular offset (90 degrees to the left and right)
    const double left_yaw = yaw + M_PI_2;   // Left is +90 degrees
    const double right_yaw = yaw - M_PI_2;  // Right is -90 degrees

    // Left bound point
    geometry_msgs::msg::Point left_point;
    left_point.x = pose.position.x + lateral_offset_m * std::cos(left_yaw);
    left_point.y = pose.position.y + lateral_offset_m * std::sin(left_yaw);
    left_point.z = pose.position.z;
    bounds.left_bound.push_back(left_point);

    // Right bound point
    geometry_msgs::msg::Point right_point;
    right_point.x = pose.position.x + lateral_offset_m * std::cos(right_yaw);
    right_point.y = pose.position.y + lateral_offset_m * std::sin(right_yaw);
    right_point.z = pose.position.z;
    bounds.right_bound.push_back(right_point);
  }

  return bounds;
}

autoware::path_optimizer::PlannerData TrajectoryMPTOptimizer::create_planner_data(
  const TrajectoryPoints & traj_points, const BoundsPair & bounds,
  const TrajectoryOptimizerData & data) const
{
  autoware::path_optimizer::PlannerData planner_data;

  // Create header from odometry frame (ego state is in this frame)
  planner_data.header.stamp = get_node_ptr()->now();
  planner_data.header.frame_id = data.current_odometry.header.frame_id;

  // Set trajectory points
  planner_data.traj_points = traj_points;

  // Set bounds
  planner_data.left_bound = bounds.left_bound;
  planner_data.right_bound = bounds.right_bound;

  // Set ego state
  planner_data.ego_pose = data.current_odometry.pose.pose;
  planner_data.ego_vel = data.current_odometry.twist.twist.linear.x;

  return planner_data;
}

}  // namespace autoware::trajectory_optimizer::plugin
