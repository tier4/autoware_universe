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

#include "run_out_module.hpp"

#include "collision.hpp"
#include "debug.hpp"
#include "decision.hpp"
#include "footprints.hpp"
#include "map_data.hpp"
#include "objects_filtering.hpp"
#include "parameters.hpp"
#include "slowdown.hpp"

#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/universe_utils/geometry/geometry.hpp>
#include <rclcpp/duration.hpp>

#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <geometry_msgs/msg/detail/pose__struct.hpp>

#include <lanelet2_core/primitives/BoundingBox.h>
#include <lanelet2_core/primitives/Point.h>

#include <iostream>
#include <memory>
#include <string>
#include <vector>

namespace autoware::motion_velocity_planner
{

using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

void RunOutModule::init_parameters(rclcpp::Node & node)
{
  params_.initialize(node, ns_);
}

void RunOutModule::update_parameters(const std::vector<rclcpp::Parameter> & parameters)
{
  params_.update(parameters, ns_);
}

void RunOutModule::init(rclcpp::Node & node, const std::string & module_name)
{
  diagnostic_updater_.emplace(&node);
  module_name_ = module_name;
  logger_ = node.get_logger();
  clock_ = node.get_clock();

  debug_publisher_ =
    node.create_publisher<visualization_msgs::msg::MarkerArray>("~/" + ns_ + "/debug_markers", 1);
  virtual_wall_publisher_ =
    node.create_publisher<visualization_msgs::msg::MarkerArray>("~/" + ns_ + "/virtual_walls", 1);
  processing_diag_publisher_ = std::make_shared<universe_utils::ProcessingTimePublisher>(
    &node, "~/debug/" + ns_ + "/processing_time_ms_diag");
  processing_time_publisher_ =
    node.create_publisher<autoware_internal_debug_msgs::msg::Float64Stamped>(
      "~/debug/" + ns_ + "/processing_time_ms", 1);
  debug_trajectory_publisher_ = node.create_publisher<autoware_planning_msgs::msg::Trajectory>(
    "~/debug/" + ns_ + "/trajectory", 1);
  timekeeper_publisher_ = node.create_publisher<autoware::universe_utils::ProcessingTimeDetail>(
    "~/" + ns_ + "processing_time", 1);
  time_keeper_ =
    std::make_shared<autoware::universe_utils::TimeKeeper>(timekeeper_publisher_, &std::cerr);

  init_parameters(node);
  diagnostic_updater_->setHardwareID("mvp_run_out");
  diagnostic_updater_->add(
    "unavoidable_run_out_collision", this, &RunOutModule::update_unavoidable_collision_status);
}

double calculate_comfortable_time_to_stop(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & ego_trajectory,
  const std::optional<double> & distance_to_stop)
{
  if (!distance_to_stop || *distance_to_stop <= 1e-3) {
    return 0.0;
  }
  auto s = 0.0;
  auto i = 0UL;
  auto s_delta = 0.0;
  for (; i + 1 < ego_trajectory.size() && s < *distance_to_stop; ++i) {
    s_delta = universe_utils::calcDistance2d(ego_trajectory[i], ego_trajectory[i + 1]);
    s += s_delta;
  }
  const auto s_diff = s_delta - (s - *distance_to_stop);
  const auto ratio = s_diff / s_delta;
  const auto t_from = rclcpp::Duration(ego_trajectory[i].time_from_start);
  const auto t_to = rclcpp::Duration(ego_trajectory[i + 1].time_from_start);
  const auto t_delta = (t_to - t_from).seconds();
  const auto t_diff = ratio * t_delta;
  // TODO(Maxime): double check
  return t_from.seconds() + t_diff;
}

void RunOutModule::update_unavoidable_collision_status(
  diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  if (unavoidable_collision_.has_value()) {
    const std::string error_msg = "[RunOut]: Unavoidable collision";
    const auto diag_level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    stat.summary(diag_level, error_msg);
    stat.addf("Time to collision", "%.2f", unavoidable_collision_->time_to_collision);
    stat.addf("Comfortable time to stop", "%.2f", unavoidable_collision_->comfortable_time_to_stop);
  } else {
    const std::string error_msg = "[RunOut]: Nominal";
    const auto diag_level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    stat.summary(diag_level, error_msg);
  }
}

void RunOutModule::ignore_unavoidable_collision(const double time_to_stop)
{
  for (auto & [_, history] : decisions_tracker_.history_per_object) {
    if (history.decisions.empty()) {
      continue;
    }
    auto & current_decision = history.decisions.back();
    if (current_decision.type == run_out::DecisionType::stop) {
      if (time_to_stop > current_decision.collision->ego_time_interval.from) {
        unavoidable_collision_.emplace();
        unavoidable_collision_->comfortable_time_to_stop = time_to_stop;
        unavoidable_collision_->time_to_collision =
          current_decision.collision->ego_time_interval.from;
        current_decision.type = run_out::DecisionType::nothing;
      }
    }
  }
  unavoidable_collision_.reset();
}

VelocityPlanningResult RunOutModule::plan(
  [[maybe_unused]] const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> &,
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & smoothed_trajectory_points,
  const std::shared_ptr<const PlannerData> planner_data)
{
  const auto now = clock_->now();
  time_keeper_->start_track("plan()");
  time_keeper_->start_track("calc_ego_footprint()");
  const auto ego_footprint = calculate_trajectory_corner_footprint(
    smoothed_trajectory_points, planner_data->vehicle_info_, params_);
  time_keeper_->end_track("calc_ego_footprint()");
  time_keeper_->start_track("filter_objects()");
  const auto filtering_data = run_out::calculate_filtering_data(
    planner_data->route_handler->getLaneletMapPtr(), ego_footprint, planner_data->objects, params_);
  auto filtered_objects = run_out::prepare_dynamic_objects(
    planner_data->objects, ego_footprint, decisions_tracker_, filtering_data, params_);
  time_keeper_->end_track("filter_objects()");
  time_keeper_->start_track("calc_rtree()");
  const auto footprint_rtree = run_out::prepare_trajectory_footprint_rtree(ego_footprint);
  time_keeper_->end_track("calc_rtree()");
  time_keeper_->start_track("calc_collisions()");
  run_out::calculate_collisions(
    filtered_objects, footprint_rtree, smoothed_trajectory_points, params_);
  time_keeper_->end_track("calc_collisions()");
  time_keeper_->start_track("calc_decisions()");
  const auto ego_is_stopped = planner_data->current_odometry.twist.twist.linear.x < 1e-2;
  run_out::calculate_decisions(decisions_tracker_, filtered_objects, now, ego_is_stopped, params_);
  time_keeper_->end_track("calc_decisions()");
  time_keeper_->start_track("calc_slowdowns()");
  const auto comfortable_time_to_stop = calculate_comfortable_time_to_stop(
    smoothed_trajectory_points, planner_data->calculate_min_deceleration_distance(0.0));
  if (params_.enable_deceleration_limit) {
    ignore_unavoidable_collision(comfortable_time_to_stop);
  }
  const auto result =
    run_out::calculate_slowdowns(decisions_tracker_, smoothed_trajectory_points, params_);
  time_keeper_->end_track("calc_slowdowns()");

  time_keeper_->start_track("publish_debug()");
  virtual_wall_marker_creator.add_virtual_walls(run_out::create_virtual_walls(
    result, smoothed_trajectory_points, planner_data->vehicle_info_.max_longitudinal_offset_m));
  virtual_wall_publisher_->publish(virtual_wall_marker_creator.create_markers(now));
  debug_publisher_->publish(run_out::make_debug_footprint_markers(ego_footprint, filtered_objects));
  debug_publisher_->publish(run_out::make_debug_object_markers(filtered_objects));
  debug_publisher_->publish(run_out::make_debug_decisions_markers(decisions_tracker_));
  debug_publisher_->publish(
    run_out::make_debug_min_stop_marker(smoothed_trajectory_points, comfortable_time_to_stop));
  debug_publisher_->publish(run_out::make_debug_filtering_data_marker(filtering_data));
  time_keeper_->end_track("publish_debug()");
  time_keeper_->end_track("plan()");
  diagnostic_updater_->force_update();
  autoware_planning_msgs::msg::Trajectory debug_trajectory;
  debug_trajectory.header.frame_id = "map";
  debug_trajectory.header.stamp = clock_->now();
  debug_trajectory.points = smoothed_trajectory_points;
  for (const auto & stop_point : result.stop_points) {
    const auto length = motion_utils::calcSignedArcLength(debug_trajectory.points, 0, stop_point);
    motion_utils::insertStopPoint(length, debug_trajectory.points);
  }
  for (const auto & slowdown_point : result.slowdown_intervals) {
    (void)slowdown_point;
    // TODO(Maxime)
  }
  debug_trajectory_publisher_->publish(debug_trajectory);
  return result;
}

}  // namespace autoware::motion_velocity_planner

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::motion_velocity_planner::RunOutModule,
  autoware::motion_velocity_planner::PluginModuleInterface)
