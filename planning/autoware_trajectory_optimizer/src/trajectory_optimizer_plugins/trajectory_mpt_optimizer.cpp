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

#include <autoware_utils/geometry/boost_geometry.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils_math/unit_conversion.hpp>
#include <autoware_utils_rclcpp/parameter.hpp>

#include <boost/geometry/algorithms/disjoint.hpp>
#include <boost/geometry/algorithms/envelope.hpp>

#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/primitives/BoundingBox.h>

#include <cmath>
#include <limits>
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

  if (!lanelet_map_ptr_ || traj_points.empty()) {
    return bounds;
  }

  constexpr double overlap_threshold = 0.01;

  // Step 1: Create trajectory linestring for spatial query
  autoware_utils::LineString2d trajectory_ls;
  trajectory_ls.reserve(traj_points.size());
  for (const auto & point : traj_points) {
    trajectory_ls.emplace_back(point.pose.position.x, point.pose.position.y);
  }

  // Step 2: Use R-tree spatial index to get candidate lanelets (FAST!)
  const auto candidate_lanelets = lanelet_map_ptr_->laneletLayer.search(
    boost::geometry::return_envelope<lanelet::BoundingBox2d>(trajectory_ls));

  if (candidate_lanelets.empty()) {
    RCLCPP_WARN_THROTTLE(
      get_node_ptr()->get_logger(), *get_node_ptr()->get_clock(), 5000,
      "MPT Optimizer: No candidate lanelets found for trajectory");
    return bounds;
  }

  // Step 3: Find the primary lanelet (closest to trajectory start that intersects)
  lanelet::ConstLanelet primary_lanelet;
  bool found_primary = false;
  double min_dist = std::numeric_limits<double>::max();

  const auto & start_pose = traj_points.front().pose;

  for (const auto & lanelet : candidate_lanelets) {
    // Check subtype = "road"
    const bool is_road =
      lanelet.hasAttribute(lanelet::AttributeName::Subtype) &&
      lanelet.attribute(lanelet::AttributeName::Subtype) == lanelet::AttributeValueString::Road;
    if (!is_road) {
      continue;
    }

    // Check if trajectory intersects lanelet polygon
    if (boost::geometry::disjoint(trajectory_ls, lanelet.polygon2d().basicPolygon())) {
      continue;
    }

    // Calculate distance from trajectory start to lanelet centerline
    const auto centerline = lanelet.centerline();
    for (const auto & cl_point : centerline) {
      const double dx = cl_point.x() - start_pose.position.x;
      const double dy = cl_point.y() - start_pose.position.y;
      const double dist = std::hypot(dx, dy);
      if (dist < min_dist) {
        min_dist = dist;
        primary_lanelet = lanelet;
        found_primary = true;
      }
    }
  }

  if (!found_primary) {
    RCLCPP_WARN_THROTTLE(
      get_node_ptr()->get_logger(), *get_node_ptr()->get_clock(), 5000,
      "MPT Optimizer: No primary lanelet found for trajectory");
    return bounds;
  }

  // Step 4: Build DrivableLanes structure (like behavior planner does)
  // Start with primary lanelet as both left and right
  lanelet::ConstLanelet left_lanelet = primary_lanelet;
  lanelet::ConstLanelet right_lanelet = primary_lanelet;

  // Step 5: Try to find adjacent lanelets that share boundaries
  for (const auto & lanelet : candidate_lanelets) {
    if (lanelet.id() == primary_lanelet.id()) {
      continue;
    }

    const bool is_road =
      lanelet.hasAttribute(lanelet::AttributeName::Subtype) &&
      lanelet.attribute(lanelet::AttributeName::Subtype) == lanelet::AttributeValueString::Road;
    if (!is_road) {
      continue;
    }

    // Check if this lanelet is left neighbor (shares right bound with primary's left bound)
    if (lanelet.rightBound().id() == primary_lanelet.leftBound().id()) {
      left_lanelet = lanelet;
    } else if (lanelet.leftBound().id() == primary_lanelet.rightBound().id()) {
      right_lanelet = lanelet;
    }
  }

  // Step 6: Extract bounds from DrivableLanes (leftmost left bound, rightmost right bound)
  const auto left_bound_3d = left_lanelet.leftBound();
  for (const auto & bound_point : left_bound_3d) {
    const auto point = lanelet::utils::conversion::toGeomMsgPt(bound_point);
    if (
      bounds.left_bound.empty() ||
      std::hypot(point.x - bounds.left_bound.back().x, point.y - bounds.left_bound.back().y) >
        overlap_threshold) {
      bounds.left_bound.push_back(point);
    }
  }

  const auto right_bound_3d = right_lanelet.rightBound();
  for (const auto & bound_point : right_bound_3d) {
    const auto point = lanelet::utils::conversion::toGeomMsgPt(bound_point);
    if (
      bounds.right_bound.empty() ||
      std::hypot(point.x - bounds.right_bound.back().x, point.y - bounds.right_bound.back().y) >
        overlap_threshold) {
      bounds.right_bound.push_back(point);
    }
  }

  RCLCPP_DEBUG(
    get_node_ptr()->get_logger(),
    "MPT Optimizer: Generated bounds from primary lanelet %ld (left: %ld, right: %ld), left=%zu "
    "pts, right=%zu pts",
    primary_lanelet.id(), left_lanelet.id(), right_lanelet.id(), bounds.left_bound.size(),
    bounds.right_bound.size());

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

  // Generate bounds from lanelet map
  const auto bounds = generate_bounds_from_lanelet_map(traj_points);

  // If bounds extraction failed, skip optimization
  if (bounds.left_bound.empty() || bounds.right_bound.empty()) {
    RCLCPP_WARN_THROTTLE(
      get_node_ptr()->get_logger(), *get_node_ptr()->get_clock(), 5000,
      "MPT Optimizer: Failed to generate bounds from lanelet map, skipping optimization");
    return;
  }

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
