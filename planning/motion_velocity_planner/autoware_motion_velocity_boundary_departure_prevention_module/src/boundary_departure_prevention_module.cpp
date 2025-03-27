// Copyright 2025 TIER IV, Inc. All rights reserved.
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

#include "boundary_departure_prevention_module.hpp"
#include <autoware_utils/ros/update_param.hpp>

#include "autoware/motion_utils/marker/marker_helper.hpp"
#include "debug.hpp"
#include "utils.hpp"

#include <autoware/boundary_departure_checker/utils.hpp>
#include <magic_enum.hpp>

#include <fmt/format.h>

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::motion_velocity_planner
{

void BoundaryDeparturePreventionModule::init(
  rclcpp::Node & node, [[maybe_unused]] const std::string & module_name)
{
  // fmt::print("Running Boundary Departure Prevention Module\n");
  module_name_ = module_name;
  clock_ptr_ = node.get_clock();
  logger_ = node.get_logger();

  node_param_ = param::NodeParam(node);
  subscribe_topics(node);
  publish_topics(node);

  updater_ptr_ = std::make_unique<diagnostic_updater::Updater>(&node);
  updater_ptr_->setHardwareID("motion_velocity_boundary_departure_prevention");
  updater_ptr_->add(
    "boundary_departure", [this](diagnostic_updater::DiagnosticStatusWrapper & stat) {
      using DiagStatus = diagnostic_msgs::msg::DiagnosticStatus;
      int8_t lvl{DiagStatus::OK};
      std::string msg{"OK"};

      if (bdp_output_.is_critical_departing) {
        lvl = DiagStatus::ERROR;
        msg = "vehicle will leave lane";
      }

      stat.summary(lvl, msg);
    });
}

void BoundaryDeparturePreventionModule::update_parameters(
  const std::vector<rclcpp::Parameter> & parameters)
{
  using autoware_utils::update_param;
  auto & pp = node_param_;

  const std::string module_name{"boundary_departure_prevention."};
  update_param(parameters, module_name + "th_data_timeout_s", pp.th_data_timeout_s);
  update_param(parameters, module_name + "boundary_types_to_detect", pp.boundary_types_to_detect);
  update_param(
    parameters, module_name + "th_max_lateral_dist_query_m", pp.th_max_lateral_dist_query_m);

  auto boundary_behaviour_trigger_param = [&module_name,
                                           &parameters](const std::string & trigger_type_str) {
    BoundaryThreshold th_dist_to_boundary_m;
    const std::string ns{module_name + trigger_type_str + "."};
    const std::string ns_bound = ns + "th_dist_to_boundary_m.";
    update_param(parameters, ns_bound + "left", th_dist_to_boundary_m.left);
    update_param(parameters, ns_bound + "right", th_dist_to_boundary_m.right);

    param::BehaviorTriggerThreshold th_behavior;
    const std::string ns_behavior{ns + "th_trigger."};
    update_param(parameters, ns_behavior + "decel_mp2", th_behavior.decel_mp2);
    update_param(parameters, ns_behavior + "brake_delay_s", th_behavior.brake_delay_s);
    update_param(parameters, ns_behavior + "dist_error_m", th_behavior.dist_error_m);

    param::BoundaryBehaviorTrigger trigger;

    update_param(parameters, ns + "enable", trigger.enable);
    trigger.th_dist_to_boundary_m = th_dist_to_boundary_m;
    trigger.th_trigger = th_behavior;

    return trigger;
  };
  pp.slow_down_before_departure = boundary_behaviour_trigger_param("slow_down_before_departure");
  pp.stop_before_departure = boundary_behaviour_trigger_param("stop_before_departure");

  auto slow_down_behaviour_trigger_param = [&](
                                             const std::string & trigger_type_str,
                                             const param::SlowDownNearBoundaryTrigger slow) {
    param::SlowDownNearBoundaryTrigger trigger(boundary_behaviour_trigger_param(trigger_type_str));
    param::SlowDownBehavior slow_behavior;
    const std::string ns{module_name + trigger_type_str + "."};
    const std::string ns_slow = ns + "slow_down_behavior.";

    auto velocity_kmh = slow.slow_down_behavior.velocity_mps * 3.6;
    update_param(parameters, ns_slow + "velocity_kmh", velocity_kmh);
    slow_behavior.velocity_mps = velocity_kmh / 3.6;
    return trigger;
  };
  pp.slow_down_near_boundary =
    slow_down_behaviour_trigger_param("slow_down_near_boundary", pp.slow_down_near_boundary);
}

void BoundaryDeparturePreventionModule::subscribe_topics(rclcpp::Node & node)
{
  // fmt::print("Subscribing\n");
  sub_ego_pred_traj_ = node.create_subscription<Trajectory>(
    "/control/trajectory_follower/lateral/predicted_trajectory", rclcpp::QoS{1},
    [&](const Trajectory::ConstSharedPtr msg) { ego_pred_traj_ptr_ = msg; });

  sub_op_mode_state_ = node.create_subscription<OperationModeState>(
    "~/api/operation_mode/state", rclcpp::QoS{1},
    [this](const OperationModeState::ConstSharedPtr msg) { op_mode_state_ptr_ = msg; });
}

void BoundaryDeparturePreventionModule::publish_topics(rclcpp::Node & node)
{
  const std::string ns = "boundary_departure_prevention";
  debug_publisher_ =
    node.create_publisher<visualization_msgs::msg::MarkerArray>("~/" + ns + "/debug_markers", 1);
}

VelocityPlanningResult BoundaryDeparturePreventionModule::plan(
  [[maybe_unused]] const TrajectoryPoints & raw_trajectory_points,
  [[maybe_unused]] const TrajectoryPoints & smoothed_trajectory_points,
  const std::shared_ptr<const PlannerData> planner_data)
{
  // fmt::print("Running Boundary Departure Prevention Module is running\n");

  const auto & curr_pose = planner_data->current_odometry.pose;
  const auto & curr_twist = planner_data->current_odometry.twist.twist;
  const auto & vehicle_info = planner_data->vehicle_info_;

  if (!ego_pred_traj_ptr_) {
    // fmt::print("Invalid ego pred path ptr\n");
    return {};
  }

  constexpr double min_velocity = 0.01;
  const auto & raw_abs_velocity = std::abs(curr_twist.linear.x);
  const auto abs_velocity = raw_abs_velocity < min_velocity ? 0.0 : raw_abs_velocity;
  const auto output_opt = plan(
    curr_pose, abs_velocity, ego_pred_traj_ptr_->points, vehicle_info,
    node_param_.pred_path_footprint.scale, *planner_data->route_handler->getLaneletMapPtr(),
    node_param_.boundary_types_to_detect, node_param_);
  if (!output_opt) {
    // fmt::print("Invalid output\n");
    return {};
  }

  auto slow_down_traj_points = raw_trajectory_points;

  const auto dist_to_ego = std::invoke([&]() {
    const auto idx = planner_data->find_segment_index(raw_trajectory_points, curr_pose.pose);
    using autoware::motion_utils::calcSignedArcLength;
    return calcSignedArcLength(slow_down_traj_points, 0, curr_pose.pose.position, idx);
  });

  const auto insert_to_traj = [&](const double lon_dist) -> std::optional<size_t> {
    using autoware::motion_utils::insertTargetPoint;
    if (const auto inserted_idx_opt = insertTargetPoint(0, lon_dist, slow_down_traj_points)) {
      if (*inserted_idx_opt + 2 <= slow_down_traj_points.size()) {
        slow_down_traj_points.at(*inserted_idx_opt).longitudinal_velocity_mps =
          slow_down_traj_points.at(*inserted_idx_opt + 1).longitudinal_velocity_mps;
      }
      return *inserted_idx_opt;
    }
    return std::nullopt;
  };

  const auto & [left_dpt_stat, right_dpt_stat] = output_opt->departure_statuses;
  const auto is_critical_departing_from_left =
    std::any_of(left_dpt_stat.begin(), left_dpt_stat.end(), [](const auto & stat) {
      const auto & [status, idx] = stat;
      return status == param::DepartureStatus::CRITICAL_DEPARTURE;
    });

  const auto is_critical_departing_from_right =
    std::any_of(right_dpt_stat.begin(), right_dpt_stat.end(), [](const auto & stat) {
      const auto & [status, idx] = stat;
      return status == param::DepartureStatus::CRITICAL_DEPARTURE;
    });
  bdp_output_.is_critical_departing =
    is_critical_departing_from_left || is_critical_departing_from_right;

  updater_ptr_->force_update();
  if (bdp_output_.is_critical_departing) {
    // fmt::print("have critical departure");
    return {};
  }

  auto slow_down_candidate_idx = utils::get_traj_indices_candidates(
    left_dpt_stat, output_opt->output.ego_sides_from_footprints, vehicle_info.vehicle_length_m);
  const auto right_candidates = utils::get_traj_indices_candidates(
    right_dpt_stat, output_opt->output.ego_sides_from_footprints, vehicle_info.vehicle_length_m);
  slow_down_candidate_idx.insert(
    slow_down_candidate_idx.end(), right_candidates.begin(), right_candidates.end());

  std::vector<SlowdownInterval> slowdown_intervals;
  int i = 0;
  for (const auto & [start_idx, end_idx] : slow_down_candidate_idx) {
    if (
      start_idx >= output_opt->output.ego_sides_from_footprints.size() ||
      end_idx >= output_opt->output.ego_sides_from_footprints.size()) {
      continue;
    }
    const auto start_dist =
      output_opt->output.ego_sides_from_footprints.at(start_idx).dist_from_start;
    const auto end_dist = output_opt->output.ego_sides_from_footprints.at(end_idx).dist_from_start;

    const auto slow_down_start_idx = insert_to_traj(start_dist + dist_to_ego);
    const auto slow_down_end_idx =
      (start_dist < end_dist) ? insert_to_traj(end_dist + dist_to_ego) : std::nullopt;

    if (!slow_down_start_idx) {
      continue;
    }

    slowdown_intervals.emplace_back(
      slow_down_traj_points.at(slow_down_start_idx ? *slow_down_start_idx : 0).pose.position,
      slow_down_traj_points.at(*slow_down_end_idx).pose.position,
      node_param_.slow_down_near_boundary.slow_down_behavior.velocity_mps);

    if (slow_down_start_idx && slow_down_end_idx) {
      const auto slow_down_wall_idx = std::invoke([&]() {
        const auto ego_idx = planner_data->find_index(slow_down_traj_points, curr_pose.pose);
        if (ego_idx < *slow_down_start_idx) return *slow_down_start_idx;
        if (ego_idx < *slow_down_end_idx) return ego_idx;
        return *slow_down_end_idx;
      });

      slow_down_wall_marker_.markers.clear();
      const auto marker = autoware::motion_utils::createSlowDownVirtualWallMarker(
        slow_down_traj_points.at(slow_down_wall_idx).pose, "boundary departure", clock_ptr_->now(),
        ++i);
      autoware_utils::append_marker_array(marker, &slow_down_wall_marker_);
    }
  }
  VelocityPlanningResult result;
  result.slowdown_intervals = slowdown_intervals;

  return result;
}

std::optional<param::BDPOutput> BoundaryDeparturePreventionModule::plan(
  const PoseWithCovariance & pose_with_covariance, const double abs_velocity,
  const TrajectoryPoints & ego_pred_traj, const VehicleInfo & vehicle_info,
  const double footprint_margin_scale, const lanelet::LaneletMap & lanelet_map,
  const std::vector<std::string> & boundary_types_to_detect, const param::NodeParam & param)
{
  param::BDPOutput bdp_output;

  auto & output = bdp_output.output;

  autoware_utils::StopWatch<std::chrono::milliseconds> stop_watch;
  const auto footprints_with_pose = utils::create_vehicle_footprints(
    pose_with_covariance, ego_pred_traj, vehicle_info, footprint_margin_scale);
  output.processing_time_map["create_vehicle_footprint"] = stop_watch.toc(true);

  if (!footprints_with_pose) {
    return std::nullopt;
  }

  output.ego_sides_from_footprints = utils::get_ego_sides_from_footprints(*footprints_with_pose);
  output.processing_time_map["get_ego_sides_from_footprints"] = stop_watch.toc(true);

  output.side_to_bound_projections = lane_departure_checker::utils::get_closest_boundary_from_side(
    lanelet_map, output.ego_sides_from_footprints, boundary_types_to_detect,
    param.th_max_lateral_dist_query_m);

  bdp_output.departure_statuses =
    utils::check_departure_status(output.side_to_bound_projections, param);

  const auto is_far = [&](const auto & param, const auto dist_to_start) {
    const auto braking_dist = utils::calc_braking_distance(
      abs_velocity, param.th_trigger.decel_mp2, param.th_trigger.brake_delay_s,
      param.th_trigger.dist_error_m);
    return (braking_dist > dist_to_start);
  };

  const auto cond = [&](const param::DepartureStatusIdx & side) {
    const auto [status, idx] = side;
    const auto dist_to_start = output.ego_sides_from_footprints.at(idx).dist_from_start;
    if (status == param::DepartureStatus::CRITICAL_DEPARTURE) {
      return is_far(param.stop_before_departure, dist_to_start);
    }

    if (status == param::DepartureStatus::APPROACHING_DEPARTURE) {
      return is_far(param.slow_down_before_departure, dist_to_start);
    }

    if (status == param::DepartureStatus::NEAR_BOUNDARY) {
      return is_far(param.slow_down_near_boundary, dist_to_start);
    }
    return true;
  };

  bdp_output.departure_statuses.left.erase(
    std::remove_if(
      bdp_output.departure_statuses.left.begin(), bdp_output.departure_statuses.left.end(), cond),
    bdp_output.departure_statuses.left.end());

  bdp_output.departure_statuses.right.erase(
    std::remove_if(
      bdp_output.departure_statuses.right.begin(), bdp_output.departure_statuses.right.end(), cond),
    bdp_output.departure_statuses.right.end());

  if (debug_publisher_) {
    autoware_utils::append_marker_array(
      debug::create_debug_marker_array(output, clock_ptr_, pose_with_covariance.pose.position.z),
      &slow_down_wall_marker_);
    debug_publisher_->publish(slow_down_wall_marker_);
  }
  return bdp_output;
}

bool BoundaryDeparturePreventionModule::is_data_ready(
  [[maybe_unused]] std::unordered_map<std::string, double> & processing_times)
{
  if (!ego_pred_traj_ptr_) {
    RCLCPP_INFO_THROTTLE(
      logger_, *clock_ptr_, throttle_duration_ms, "waiting for predicted trajectory...");
    return false;
  }

  if (!op_mode_state_ptr_) {
    RCLCPP_INFO_THROTTLE(
      logger_, *clock_ptr_, throttle_duration_ms, "waiting for operation mode state...");
    return false;
  }

  return true;
}

bool BoundaryDeparturePreventionModule::is_data_valid() const
{
  if (ego_pred_traj_ptr_->points.empty()) {
    RCLCPP_INFO_THROTTLE(
      logger_, *clock_ptr_, throttle_duration_ms, "empty predicted trajectory...");
    return false;
  }
  return true;
}

bool BoundaryDeparturePreventionModule::is_data_timeout(const Odometry & odom) const
{
  const auto now = clock_ptr_->now();
  const auto time_diff_s = (rclcpp::Time(odom.header.stamp) - now).seconds();
  constexpr auto th_pose_timeout_s = 1.0;

  if (time_diff_s > th_pose_timeout_s) {
    RCLCPP_INFO_THROTTLE(logger_, *clock_ptr_, throttle_duration_ms, "pose timeout...");
    return true;
  }

  return false;
}
}  // namespace autoware::motion_velocity_planner

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::motion_velocity_planner::BoundaryDeparturePreventionModule,
  autoware::motion_velocity_planner::PluginModuleInterface)
