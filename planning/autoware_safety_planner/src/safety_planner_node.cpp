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

#include "safety_planner_node.hpp"

#include "utils/sl_view_utils.hpp"
#include "utils/trajectory_conversion.hpp"

#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils_visualization/marker_helper.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace autoware::safety_planner
{

SafetyPlannerNode::SafetyPlannerNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("safety_planner_node", options),
  generator_uuid_(autoware_utils_uuid::generate_uuid()),
  vehicle_info_(vehicle_info_utils::VehicleInfoUtils(*this).getVehicleInfo())
{
  param_listener_ =
    std::make_shared<::safety_planner::ParamListener>(get_node_parameters_interface());
  params_ = param_listener_->get_params();

  pub_debug_trajectory_ = this->create_publisher<Trajectory>("~/debug/trajectory", 1);
  pub_debug_rough_trajectory_ = this->create_publisher<Trajectory>("~/debug/rough_trajectory", 1);
  pub_candidate_trajectories_ =
    this->create_publisher<CandidateTrajectories>("~/output/candidate_trajectories", 1);
  pub_debug_marker_ = this->create_publisher<MarkerArray>("~/debug/debug_marker", 1);
  pub_debug_rough_planner_marker_ =
    this->create_publisher<MarkerArray>("~/debug/rough_planner_marker", 1);

  debug_processing_time_detail_pub_ =
    this->create_publisher<autoware_utils_debug::ProcessingTimeDetail>(
      "~/debug/processing_time_detail_ms", 1);
  time_keeper_ =
    std::make_shared<autoware_utils_debug::TimeKeeper>(debug_processing_time_detail_pub_);

  planner_ = std::make_unique<SafetyPlanner>(params_, time_keeper_);
  for (const auto & name : planner_->get_constraint_generator_plugin_names()) {
    constraint_debug_marker_publishers_[name] =
      this->create_publisher<MarkerArray>("~/debug/constraints/" + name, 1);
  }

  const auto planning_freq = rclcpp::Rate(10.0);
  timer_ = rclcpp::create_timer(
    this, get_clock(), planning_freq.period(), std::bind(&SafetyPlannerNode::on_timer, this));
}

bool SafetyPlannerNode::is_data_ready(const InputData & input_data)
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);
  const auto notify_waiting = [this](const std::string & name) {
    RCLCPP_INFO_SKIPFIRST_THROTTLE(
      get_logger(), *get_clock(), 5000, "waiting for %s", name.c_str());
  };

  // TODO(odashima): check topic timeout

  if (!input_data.lanelet_map_bin_ptr) {
    notify_waiting("lanelet map");
    return false;
  }
  if (!input_data.route_ptr) {
    notify_waiting("route");
    return false;
  }
  if (!input_data.lanelet_map_bin_ptr) {
    notify_waiting("lanelet map");
    return false;
  }
  if (!input_data.odometry_ptr) {
    notify_waiting("odometry");
    return false;
  }
  if (!input_data.acceleration_ptr) {
    notify_waiting("acceleration");
    return false;
  }
  if (!input_data.steering_ptr) {
    notify_waiting("steering");
    return false;
  }

  return true;
}

SafetyPlannerNode::InputData SafetyPlannerNode::take_data()
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);
  InputData input_data;

  if (const auto msg = route_subscriber_.take_data()) {
    if (!msg->segments.empty()) {
      route_ptr_ = msg;
    } else {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "input route is empty, ignoring...");
    }
  }
  input_data.route_ptr = route_ptr_;

  if (const auto msg = vector_map_subscriber_.take_data()) {
    lanelet_map_bin_ptr_ = msg;
  }
  input_data.lanelet_map_bin_ptr = lanelet_map_bin_ptr_;

  if (const auto msg = odometry_subscriber_.take_data()) {
    odometry_ptr_ = msg;
  }
  input_data.odometry_ptr = odometry_ptr_;

  if (const auto msg = acceleration_subscriber_.take_data()) {
    acceleration_ptr_ = msg;
  }
  input_data.acceleration_ptr = acceleration_ptr_;

  if (const auto msg = objects_subscriber_.take_data()) {
    predicted_objects_ptr_ = msg;
  }
  input_data.predicted_objects_ptr = predicted_objects_ptr_;

  if (const auto msg = steering_subscriber_.take_data()) {
    steering_ptr_ = msg;
  }
  input_data.steering_ptr = steering_ptr_;

  if (const auto msg = pointcloud_subscriber_.take_data()) {
    obstacle_pointcloud_ptr_ = msg;
  }
  input_data.obstacle_pointcloud_ptr = obstacle_pointcloud_ptr_;

  return input_data;
}

void SafetyPlannerNode::on_timer()
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);

  const auto input_data = take_data();
  if (!is_data_ready(input_data)) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "Waiting for necessary data to plan trajectories.");
    return;
  }

  if (!update_input(input_data)) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "Failed to update input. Skipping planning cycle.");
    return;
  }

  const auto planned = planner_->plan(input_);
  if (!planned) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000, "%s. Skipping this cycle.",
      planned.error().c_str());
    return;
  }
  const auto & result = planned.value();

  // publish (normal 側のみ。cautial 側の consumer は未実装)
  if (result.normal_trajectory) {
    publish_trajectory(*result.normal_trajectory);
  }
  publish_constraints_debug_markers(result.debug.constraint_generator_outputs);
  publish_rough_plan_trajectory(result.debug.rough_plan_result);
  publish_rough_plan_markers(result.debug.rough_plan_result);
  publish_debug_markers(result.debug);

  // どの制約が経路に影響を与えたかの情報をpublishする(どうやって検出する？)
  // publish_planning_factors();
}

bool SafetyPlannerNode::update_route_manager(const InputData & input_data)
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);

  const auto & current_pose = input_data.odometry_ptr->pose.pose;

  // route / map が差し替わったら追従は意味を持たないので作り直す
  const bool needs_create = !input_.route_manager ||
                            route_uuid_of_route_manager_ != input_data.route_ptr->uuid ||
                            map_ptr_of_route_manager_ != input_data.lanelet_map_bin_ptr;

  if (!needs_create) {
    try {
      input_.route_manager = std::move(*input_.route_manager)
                               .update_current_pose(
                                 current_pose, params_.ego_nearest_lanelet.dist_threshold_m,
                                 params_.ego_nearest_lanelet.yaw_threshold_rad);
    } catch (const std::exception & e) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000, "update_current_pose threw: %s", e.what());
      input_.route_manager = std::nullopt;
    }
    if (input_.route_manager) {
      return true;
    }
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "Failed to track current pose on the route. Re-creating RouteManager.");
  }

  input_.route_manager =
    RouteManager::create(*input_data.lanelet_map_bin_ptr, *input_data.route_ptr, current_pose);

  if (!input_.route_manager) {
    route_uuid_of_route_manager_.reset();
    map_ptr_of_route_manager_.reset();
    return false;
  }

  route_uuid_of_route_manager_ = input_data.route_ptr->uuid;
  map_ptr_of_route_manager_ = input_data.lanelet_map_bin_ptr;
  return true;
}

bool SafetyPlannerNode::update_input(const InputData & input_data)
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);

  input_.vehicle_info = vehicle_info_;

  if (!update_route_manager(input_data)) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000, "Failed to update RouteManager.");
    return false;
  }

  input_.odometry = *input_data.odometry_ptr;
  input_.acceleration = *input_data.acceleration_ptr;
  input_.steering = *input_data.steering_ptr;
  input_.goal_pose = input_data.route_ptr->goal_pose;
  input_.predicted_objects = input_data.predicted_objects_ptr;

  return true;
}

void SafetyPlannerNode::publish_trajectory(const Trajectory & trajectory) const
{
  // 本線出力: CandidateTrajectories (Selector が generator_id / generator_name で識別する)
  CandidateTrajectories candidate_trajectories;
  auto & candidate = candidate_trajectories.candidate_trajectories.emplace_back();
  candidate.header = trajectory.header;
  candidate.generator_id = generator_uuid_;
  candidate.points = trajectory.points;
  auto & generator_info = candidate_trajectories.generator_info.emplace_back();
  generator_info.generator_id = generator_uuid_;
  generator_info.generator_name.data = "SafetyPlanner_Normal";
  pub_candidate_trajectories_->publish(candidate_trajectories);

  // デバッグ用
  pub_debug_trajectory_->publish(trajectory);
}

void SafetyPlannerNode::publish_rough_plan_trajectory(const RoughPlanResult & rough_plan_result)
{
  if (rough_plan_result.plans.empty()) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000, "rough_plan candidates are empty.");
    return;
  }

  // 当面 consumer は先頭候補のみ消費する (rough_planner.hpp の運用 K = 1)
  const auto & plan = rough_plan_result.plans.front();
  if (plan.points.empty()) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "the adopted rough_plan has no points. Skipping trajectory publish.");
    return;
  }

  Trajectory trajectory;
  trajectory.header.frame_id = "map";
  trajectory.header.stamp = this->now();
  trajectory.points.reserve(plan.points.size());
  const double z = input_.odometry.pose.pose.position.z;
  const double wheel_base_m = input_.vehicle_info.wheel_base_m;
  for (const auto & rough_point : plan.points) {
    trajectory.points.push_back(to_trajectory_point(rough_point, z, wheel_base_m));
  }

  pub_debug_rough_trajectory_->publish(trajectory);
}

void SafetyPlannerNode::publish_rough_plan_markers(const RoughPlanResult & rough_plan_result) const
{
  // 候補本数が周期で変わる (candidate_<i> の ns が減る) と前周期のマーカーが残るので、
  // DELETEALL を先頭に挟んでから今周期分を積む
  MarkerArray marker_array;
  Marker delete_all;
  delete_all.action = Marker::DELETEALL;
  marker_array.markers.push_back(delete_all);

  const auto & markers = rough_plan_result.debug.debug_markers.markers;
  marker_array.markers.insert(marker_array.markers.end(), markers.begin(), markers.end());
  pub_debug_rough_planner_marker_->publish(marker_array);
}

void SafetyPlannerNode::publish_debug_markers(const SafetyPlannerResult::Debug & debug) const
{
  using autoware_utils_visualization::create_default_marker;
  using autoware_utils_visualization::create_marker_color;
  using autoware_utils_visualization::create_marker_scale;

  const auto now = this->now();
  MarkerArray marker_array;

  // -------------------- コンテキストの可視化 --------------------
  // current_pose
  {
    auto marker = create_default_marker(
      "map", now, "current_pose", 0, Marker::ARROW, create_marker_scale(2.0, 0.5, 0.5),
      create_marker_color(0.0, 1.0, 0.0, 0.999));
    marker.pose = input_.odometry.pose.pose;
    marker_array.markers.push_back(marker);
  }

  // goal_pose
  {
    auto marker = create_default_marker(
      "map", now, "goal_pose", 0, Marker::ARROW, create_marker_scale(2.0, 0.5, 0.5),
      create_marker_color(1.0, 0.0, 0.0, 0.999));
    marker.pose = input_.goal_pose;
    marker_array.markers.push_back(marker);
  }

  // reference_pathの可視化
  {
    auto marker = create_default_marker(
      "map", now, "reference_path", 0, Marker::LINE_STRIP, create_marker_scale(0.2, 0.0, 0.0),
      create_marker_color(0.0, 0.5, 1.0, 0.999));
    // 基底点 (waypoint、数 m 間隔) だけだと折れ線に見えるので、補間形状が分かるように 1 m 以下で
    // サンプルする (終端も含める)
    constexpr double MARKER_INTERVAL_M = 1.0;
    const auto & reference_path = debug.reference_path;
    for (double s = 0.0; s < reference_path.length(); s += MARKER_INTERVAL_M) {
      marker.points.push_back(reference_path.compute(s).point.pose.position);
    }
    if (reference_path.length() > 0.0) {
      marker.points.push_back(reference_path.compute(reference_path.length()).point.pose.position);
    }
    if (marker.points.size() >= 2) {
      marker_array.markers.push_back(marker);
    }
  }

  // 横境界 (射影ビュー) の可視化: reference_path 上の一定間隔ごとに、中心線から各境界までを
  // 法線方向の細線で結ぶ。どの s にどちら側の境界が効いているかを見るためのもの
  {
    constexpr double INTERVAL_M = 2.0;
    auto hard_marker = create_default_marker(
      "map", now, "lateral_bounds_hard", 0, Marker::LINE_LIST, create_marker_scale(0.05, 0.0, 0.0),
      create_marker_color(1.0, 0.2, 0.0, 0.8));
    auto soft_marker = create_default_marker(
      "map", now, "lateral_bounds_soft", 0, Marker::LINE_LIST, create_marker_scale(0.05, 0.0, 0.0),
      create_marker_color(1.0, 0.8, 0.0, 0.5));
    const auto & reference_path = debug.reference_path;
    const auto & compiled = debug.compiled_constraints;
    const double z = input_.odometry.pose.pose.position.z;
    for (double s = 0.0; s <= reference_path.length(); s += INTERVAL_M) {
      for (const auto & bound : compiled.lateral_bounds) {
        if (
          bound.polyline.size() < 2 || s < bound.polyline.front().s ||
          s > bound.polyline.back().s) {
          continue;
        }
        const double l_bound = interpolate_boundary_l(bound.polyline, s);
        auto & marker = compiled.raw_constraints[bound.raw_index].hardness == Hardness::HARD
                          ? hard_marker
                          : soft_marker;
        for (const double l : {0.0, l_bound}) {
          const auto pose = to_world_pose(reference_path, s, l);
          geometry_msgs::msg::Point q;
          q.x = pose.position.x();
          q.y = pose.position.y();
          q.z = z;
          marker.points.push_back(q);
        }
      }
    }
    for (auto & marker : {hard_marker, soft_marker}) {
      if (!marker.points.empty()) {
        marker_array.markers.push_back(marker);
      }
    }
  }

  // -------------------- IRの可視化 --------------------
  (void)debug.compiled_constraints;

  pub_debug_marker_->publish(marker_array);
}

void SafetyPlannerNode::publish_constraints_debug_markers(
  const std::map<std::string, ConstraintGeneratorOutput> & constraints) const
{
  for (const auto & [plugin_name, output] : constraints) {
    if (!output.debug_markers) {
      continue;
    }
    const auto it = constraint_debug_marker_publishers_.find(plugin_name);
    if (it == constraint_debug_marker_publishers_.end()) {
      RCLCPP_WARN_ONCE(
        get_logger(), "no debug marker publisher for plugin '%s'", plugin_name.c_str());
      continue;
    }
    it->second->publish(*output.debug_markers);
  }
}

}  // namespace autoware::safety_planner

RCLCPP_COMPONENTS_REGISTER_NODE(autoware::safety_planner::SafetyPlannerNode)
