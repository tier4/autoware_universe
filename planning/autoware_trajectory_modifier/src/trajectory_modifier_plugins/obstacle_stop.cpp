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

#include "autoware/trajectory_modifier/trajectory_modifier_plugins/obstacle_stop.hpp"

#include "autoware/trajectory_modifier/trajectory_modifier_plugins/velocity_limits.hpp"
#include "autoware/trajectory_modifier/trajectory_modifier_utils/obstacle_stop_utils.hpp"
#include "autoware/trajectory_modifier/trajectory_modifier_utils/utils.hpp"

#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_utils/ros/marker_helper.hpp>
#include <autoware_utils_geometry/geometry.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/logging.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace autoware::trajectory_modifier::plugin
{
using utils::obstacle_stop::build_trajectory_footprint_index;
using utils::obstacle_stop::compute_cruise_velocity;
using utils::obstacle_stop::evaluate_object_states;
using utils::obstacle_stop::get_nearest_object_collision;
using utils::obstacle_stop::get_nearest_pcd_collision;
using utils::obstacle_stop::PointCloud;
using utils::obstacle_stop::PointCloud2;
using utils::obstacle_stop::select_cruise_target;
using utils::obstacle_stop::TargetObject;
using utils::obstacle_stop::TargetObjects;

namespace
{
void ensure_increasing_timestamps(TrajectoryPoints & points)
{
  if (points.size() < 2) return;

  auto is_strictly_increasing = [&]() {
    auto prev = rclcpp::Duration(points.front().time_from_start).seconds();
    if (!std::isfinite(prev) || prev < 0.0) return false;
    for (std::size_t i = 1; i < points.size(); ++i) {
      const auto t = rclcpp::Duration(points[i].time_from_start).seconds();
      if (!std::isfinite(t) || t <= prev) return false;
      prev = t;
    }
    return true;
  };
  if (is_strictly_increasing()) return;

  constexpr double min_dt = 1e-3;
  points.front().time_from_start = rclcpp::Duration::from_seconds(0.0);
  auto t = 0.0;
  for (std::size_t i = 1; i < points.size(); ++i) {
    const auto ds = autoware_utils_geometry::calc_distance2d(points[i - 1].pose, points[i].pose);
    const auto speed =
      std::max(static_cast<double>(std::abs(points[i - 1].longitudinal_velocity_mps)), min_dt);
    t += std::max(ds / speed, min_dt);
    points[i].time_from_start = rclcpp::Duration::from_seconds(t);
  }
}
}  // namespace

void ObstacleStop::on_initialize(const TrajectoryModifierParams & params)
{
  const auto node_ptr = get_node_ptr();
  planning_factor_interface_ =
    std::make_unique<autoware::planning_factor_interface::PlanningFactorInterface>(
      node_ptr, "modifier_obstacle_stop");

  pub_filtered_pointcloud_ =
    node_ptr->create_publisher<PointCloud2>("~/obstacle_stop/debug/filtered_points", 1);
  debug_viz_pub_ = node_ptr->create_publisher<visualization_msgs::msg::MarkerArray>(
    "~/obstacle_stop/debug/marker", 1);
  pub_debug_text_ = node_ptr->create_publisher<StringStamped>("~/obstacle_stop/debug/text", 1);

  params_ = params.obstacle_stop;
  stopping_params_ = params.stopping_constraints;
  enabled_ = params.use_obstacle_stop;
  trajectory_time_step_ = params.trajectory_time_step;

  verify_params();

  {
    const auto & p = params_.objects;
    object_filter_ = std::make_unique<utils::obstacle_stop::ObjectFilter>(
      p.target_objects.bbox, p.target_objects.polygon, p.stopped_velocity_th,
      p.max_lateral_velocity_th, p.safety_buffer, p.low_velocity_th);
    pointcloud_filter_ =
      std::make_unique<utils::obstacle_stop::PointCloudFilter>(p.target_objects.pointcloud);

    lateral_margin_map_ = {
      {utils::obstacle_stop::ObjectType::CAR, {p.lateral_margin.car, p.low_velocity_lateral_margin.car}},
      {utils::obstacle_stop::ObjectType::TRUCK, {p.lateral_margin.truck, p.low_velocity_lateral_margin.truck}},
      {utils::obstacle_stop::ObjectType::BUS, {p.lateral_margin.bus, p.low_velocity_lateral_margin.bus}},
      {utils::obstacle_stop::ObjectType::TRAILER, {p.lateral_margin.trailer, p.low_velocity_lateral_margin.trailer}},
      {utils::obstacle_stop::ObjectType::MOTORCYCLE, {p.lateral_margin.motorcycle, p.low_velocity_lateral_margin.motorcycle}},
      {utils::obstacle_stop::ObjectType::BICYCLE, {p.lateral_margin.bicycle, p.low_velocity_lateral_margin.bicycle}},
      {utils::obstacle_stop::ObjectType::PEDESTRIAN, {p.lateral_margin.pedestrian, p.low_velocity_lateral_margin.pedestrian}},
      {utils::obstacle_stop::ObjectType::HAZARD, {p.lateral_margin.hazard, p.low_velocity_lateral_margin.hazard}},
      {utils::obstacle_stop::ObjectType::STRUCTURE, {p.lateral_margin.structure, p.low_velocity_lateral_margin.structure}},
      {utils::obstacle_stop::ObjectType::VEGETATION, {p.lateral_margin.vegetation, p.low_velocity_lateral_margin.vegetation}},
      {utils::obstacle_stop::ObjectType::UNKNOWN, {p.lateral_margin.unknown, p.low_velocity_lateral_margin.unknown}},
    };
  }

  {
    const auto & p = params_.obstacle_tracking;
    obstacle_tracker_ = std::make_unique<utils::obstacle_stop::ObstacleTracker>(
      p.on_time_buffer, p.off_time_buffer, p.object_distance_th, p.object_yaw_th, p.pcd_distance_th,
      p.grace_period);
  }

  {
    const auto & p = params_.rss_params;
    object_decel_map_ = {
      {utils::obstacle_stop::ObjectType::CAR, p.object_decel.car},
      {utils::obstacle_stop::ObjectType::TRUCK, p.object_decel.truck},
      {utils::obstacle_stop::ObjectType::BUS, p.object_decel.bus},
      {utils::obstacle_stop::ObjectType::TRAILER, p.object_decel.trailer},
      {utils::obstacle_stop::ObjectType::MOTORCYCLE, p.object_decel.motorcycle},
      {utils::obstacle_stop::ObjectType::BICYCLE, p.object_decel.bicycle},
      {utils::obstacle_stop::ObjectType::PEDESTRIAN, p.object_decel.pedestrian},
      {utils::obstacle_stop::ObjectType::ANIMAL, p.object_decel.animal}};
  }
}

void ObstacleStop::update_params(const TrajectoryModifierParams & params)
{
  params_ = params.obstacle_stop;
  stopping_params_ = params.stopping_constraints;
  enabled_ = params.use_obstacle_stop;
  trajectory_time_step_ = params.trajectory_time_step;

  verify_params();

  {
    const auto & p = params_.objects;
    object_filter_->set_params(
      p.target_objects.bbox, p.target_objects.polygon, p.stopped_velocity_th,
      p.max_lateral_velocity_th, p.safety_buffer, p.low_velocity_th);
    pointcloud_filter_->set_params(p.target_objects.pointcloud);

    lateral_margin_map_ = {
      {utils::obstacle_stop::ObjectType::CAR, {p.lateral_margin.car, p.low_velocity_lateral_margin.car}},
      {utils::obstacle_stop::ObjectType::TRUCK, {p.lateral_margin.truck, p.low_velocity_lateral_margin.truck}},
      {utils::obstacle_stop::ObjectType::BUS, {p.lateral_margin.bus, p.low_velocity_lateral_margin.bus}},
      {utils::obstacle_stop::ObjectType::TRAILER, {p.lateral_margin.trailer, p.low_velocity_lateral_margin.trailer}},
      {utils::obstacle_stop::ObjectType::MOTORCYCLE, {p.lateral_margin.motorcycle, p.low_velocity_lateral_margin.motorcycle}},
      {utils::obstacle_stop::ObjectType::BICYCLE, {p.lateral_margin.bicycle, p.low_velocity_lateral_margin.bicycle}},
      {utils::obstacle_stop::ObjectType::PEDESTRIAN, {p.lateral_margin.pedestrian, p.low_velocity_lateral_margin.pedestrian}},
      {utils::obstacle_stop::ObjectType::HAZARD, {p.lateral_margin.hazard, p.low_velocity_lateral_margin.hazard}},
      {utils::obstacle_stop::ObjectType::STRUCTURE, {p.lateral_margin.structure, p.low_velocity_lateral_margin.structure}},
      {utils::obstacle_stop::ObjectType::VEGETATION, {p.lateral_margin.vegetation, p.low_velocity_lateral_margin.vegetation}},
      {utils::obstacle_stop::ObjectType::UNKNOWN, {p.lateral_margin.unknown, p.low_velocity_lateral_margin.unknown}},
    };
  }

  {
    const auto & p = params_.obstacle_tracking;
    obstacle_tracker_->set_params(
      p.on_time_buffer, p.off_time_buffer, p.object_distance_th, p.object_yaw_th, p.pcd_distance_th,
      p.grace_period);
  }

  {
    const auto & p = params_.rss_params;
    object_decel_map_ = {
      {utils::obstacle_stop::ObjectType::CAR, p.object_decel.car},
      {utils::obstacle_stop::ObjectType::TRUCK, p.object_decel.truck},
      {utils::obstacle_stop::ObjectType::BUS, p.object_decel.bus},
      {utils::obstacle_stop::ObjectType::TRAILER, p.object_decel.trailer},
      {utils::obstacle_stop::ObjectType::MOTORCYCLE, p.object_decel.motorcycle},
      {utils::obstacle_stop::ObjectType::BICYCLE, p.object_decel.bicycle},
      {utils::obstacle_stop::ObjectType::PEDESTRIAN, p.object_decel.pedestrian},
      {utils::obstacle_stop::ObjectType::ANIMAL, p.object_decel.animal}};
  }
}

void ObstacleStop::verify_params()
{
  auto & rss = params_.rss_params;
  rss.ego_decel = std::clamp(
    rss.ego_decel, stopping_params_.nominal_deceleration, stopping_params_.maximum_deceleration);

  // cruise must stay softer than the RSS stop check, otherwise the two keep handing over control
  auto & cruise = params_.cruise;
  if (cruise.ego_decel > rss.ego_decel) {
    RCLCPP_WARN(
      get_node_ptr()->get_logger(),
      "[TM ObstacleStop] cruise.ego_decel (%.2f) exceeds rss_params.ego_decel (%.2f), clamping",
      cruise.ego_decel, rss.ego_decel);
    cruise.ego_decel = rss.ego_decel;
  }
  if (cruise.min_distance < params_.stop_margin) {
    RCLCPP_WARN(
      get_node_ptr()->get_logger(),
      "[TM ObstacleStop] cruise.min_distance (%.2f) is below stop_margin (%.2f), clamping",
      cruise.min_distance, params_.stop_margin);
    cruise.min_distance = params_.stop_margin;
  }
}

void ObstacleStop::reset_cycle()
{
  debug_data_ = DebugData();
  safety_factors_ = SafetyFactorArray{};
  cruise_report_.reset();
  nearest_collision_point_ = std::nullopt;
  object_rss_states_.clear();
}

void ObstacleStop::build_trajectory_shape(
  const TrajectoryPoints & traj_points, const TrajectoryModifierData & input)
{
  autoware_utils_debug::ScopedTimeTrack st(
    "ObstacleStop::build_trajectory_footprint_index", *get_time_keeper());

  debug_data_.trajectory_shape = build_trajectory_footprint_index(
    traj_points, input.current_odometry->pose.pose, context_->vehicle_info,
    input.current_odometry->twist.twist.linear.x, input.current_acceleration->accel.accel.linear.x,
    stopping_params_.nominal_deceleration, stopping_params_.jerk_limit, params_.stop_margin,
    stopping_params_.delay_response_time);
}

bool ObstacleStop::is_trajectory_modification_required(
  const TrajectoryPoints & traj_points, const TrajectoryModifierData & input)
{
  reset_cycle();

  if (traj_points.empty()) {
    return false;
  }

  build_trajectory_shape(traj_points, input);
  detect_objects(traj_points, input);
  object_rss_states_ = evaluate_object_states(
    debug_data_.target_objects, traj_points, debug_data_.trajectory_shape, object_decel_map_,
    params_.objects.stopped_velocity_th);
  const auto collision_point_pcd = check_pointcloud(traj_points, input);
  const auto collision_point_objects = evaluate_object_collision(traj_points);
  check_obstacles(collision_point_objects, collision_point_pcd);

  debug_data_.ego_z = input.current_odometry->pose.pose.position.z;
  publish_debug_string(nearest_collision_point_ == std::nullopt);
  return nearest_collision_point_ != std::nullopt;
}

ProcessingResult ObstacleStop::process(
  TrajectoryPoints & traj_points, TrajectoryModifierData & input)
{
  autoware_utils_debug::ScopedTimeTrack st("ObstacleStop::process", *get_time_keeper());

  if (!enabled_ || traj_points.size() < 2) return ProcessingResult::Unchanged;

  auto trajectory = traj_points;
  utils::obstacle_stop::trim_trajectory_and_remove_duplicates(trajectory);
  if (trajectory.size() < 2) return ProcessingResult::Unchanged;

  reset_cycle();
  build_trajectory_shape(trajectory, input);
  detect_objects(trajectory, input);
  object_rss_states_ = evaluate_object_states(
    debug_data_.target_objects, trajectory, debug_data_.trajectory_shape, object_decel_map_,
    params_.objects.stopped_velocity_th);

  const bool cruise_modified = apply_cruise(trajectory, input);

  const auto collision_point_pcd = check_pointcloud(trajectory, input);
  const auto collision_point_objects = evaluate_object_collision(trajectory);
  check_obstacles(collision_point_objects, collision_point_pcd);

  debug_data_.ego_z = input.current_odometry->pose.pose.position.z;
  publish_debug_string(nearest_collision_point_ == std::nullopt);

  const auto stop_modified = nearest_collision_point_ ? set_stop_point(trajectory, input) : false;

  if (!cruise_modified && !stop_modified) {
    return ProcessingResult::Unchanged;
  }

  if (!stop_modified && cruise_report_) {
    add_cruise_planning_factor(cruise_report_.value(), input.current_odometry->pose.pose);
  }

  traj_points = std::move(trajectory);
  return ProcessingResult::Modified;
}

bool ObstacleStop::apply_cruise(
  TrajectoryPoints & traj_points, const TrajectoryModifierData & input)
{
  autoware_utils_debug::ScopedTimeTrack st("ObstacleStop::apply_cruise", *get_time_keeper());

  if (!params_.cruise.enable || traj_points.size() < 2 || !input.current_odometry) {
    return false;
  }

  const auto target = select_cruise_target(object_rss_states_);
  if (!target) {
    return false;
  }

  const auto & cruise = params_.cruise;
  const auto margin = std::max(cruise.min_distance, cruise.time_headway * target->lon_vel);
  const auto v_cruise = compute_cruise_velocity(
    target->gap, cruise.ego_decel, target->stopping_distance, stopping_params_.delay_response_time,
    margin);
  cruise_report_ = CruiseReport{*target, v_cruise};

  ensure_increasing_timestamps(traj_points);

  const detail::VelocityLimitConstraints constraints{0.0, cruise.max_deceleration, cruise.max_jerk};
  const auto result = detail::apply_velocity_limits(
    traj_points, constraints,
    [v_cruise](const geometry_msgs::msg::Point &) { return std::optional<double>{v_cruise}; },
    detail::make_velocity_limit_options(input));

  if (!result.error.empty()) {
    RCLCPP_WARN_THROTTLE(
      get_node_ptr()->get_logger(), *get_clock(), 1000,
      "[TM ObstacleStop] Cruise velocity limit failed for candidate %zu: %s", input.candidate_index,
      result.error.c_str());
  }
  if (result.status != ProcessingResult::Modified) {
    return false;
  }
  return true;
}

void ObstacleStop::add_cruise_planning_factor(const CruiseReport & report, const geometry_msgs::msg::Pose & ego_pose)
{
  if (report.target.index >= debug_data_.target_objects.size()) {
    return;
  }
  const auto & object = debug_data_.target_objects[report.target.index].object;
  const auto & pose = object.kinematics.initial_pose_with_covariance.pose;

  SafetyFactor safety_factor;
  safety_factor.type = SafetyFactor::OBJECT;
  safety_factor.object_id = object.object_id;
  safety_factor.points = {pose.position};
  safety_factor.is_safe = false;

  SafetyFactorArray safety_factors;
  safety_factors.is_safe = false;
  safety_factors.factors.push_back(safety_factor);

  planning_factor_interface_->add(
    report.gap(), ego_pose, PlanningFactor::SLOW_DOWN, safety_factors, true, report.cruise_velocity);
}

bool ObstacleStop::set_stop_point(
  TrajectoryPoints & traj_points, const TrajectoryModifierData & input)
{
  autoware_utils_debug::ScopedTimeTrack st("ObstacleStop::set_stop_point", *get_time_keeper());

  const auto stop_margin =
    nearest_collision_point_->is_dynamic ? params_.minimum_stop_margin : params_.stop_margin;
  const auto ego_longitudinal_offset = context_->vehicle_info.max_longitudinal_offset_m;
  const auto target_stop_margin = stop_margin + ego_longitudinal_offset;
  const auto target_stop_point_arc_length = utils::clamp_stop_point_arc_length(
    nearest_collision_point_->arc_length - target_stop_margin,
    debug_data_.trajectory_shape.trajectory_length, input.current_odometry->twist.twist.linear.x,
    input.current_acceleration->accel.accel.linear.x, stopping_params_.maximum_deceleration,
    stopping_params_.jerk_limit, stopping_params_.delay_response_time);

  // actual stop margin from ego front to collision point
  const auto actual_stop_margin =
    nearest_collision_point_->arc_length - (target_stop_point_arc_length + ego_longitudinal_offset);
  const auto overlap_th =
    actual_stop_margin > params_.minimum_stop_margin ? params_.duplicate_check_threshold : 0.0;
  if (utils::stop_point_exists(traj_points, target_stop_point_arc_length, overlap_th)) {
    RCLCPP_WARN_THROTTLE(
      get_node_ptr()->get_logger(), *get_clock(), 1000,
      "[TM ObstacleStop] Preceding (or duplicate) stop point exists, skip inserting stop point");
    return false;
  }

  const auto ego_arc_length = debug_data_.trajectory_shape.ego_arc_length();
  const auto ego_to_stop_arc_length = target_stop_point_arc_length - ego_arc_length;

  if (
    ego_to_stop_arc_length < stopping_params_.arrived_distance_threshold ||
    !utils::insert_stop_point(traj_points, target_stop_point_arc_length, trajectory_time_step_)) {
    utils::replace_trajectory_with_stop_point(
      traj_points, input.current_odometry->pose.pose, trajectory_time_step_);
  }

  const auto & stop_pose = traj_points.back().pose;
  const auto & ego_pose = input.current_odometry->pose.pose;
  auto distance =
    motion_utils::calcSignedArcLength(traj_points, ego_pose.position, stop_pose.position);
  if (std::isnan(distance) || distance < 1e-3) distance = 0.0;
  planning_factor_interface_->add(
    distance, stop_pose, autoware_internal_planning_msgs::msg::PlanningFactor::STOP,
    safety_factors_);

  RCLCPP_WARN_THROTTLE(
    get_node_ptr()->get_logger(), *get_clock(), 1000,
    "[TM ObstacleStop] Inserted stop point at arc length %f m", target_stop_point_arc_length);
  return true;
}

void ObstacleStop::check_obstacles(
  const std::optional<CollisionPoint> & collision_point_objects,
  const std::optional<CollisionPoint> & collision_point_pcd)
{
  autoware_utils_debug::ScopedTimeTrack st("ObstacleStop::check_obstacles", *get_time_keeper());

  auto get_safety_factor = [&](
                             const geometry_msgs::msg::Point & point,
                             const SafetyFactor::_type_type type) -> SafetyFactor {
    SafetyFactor safety_factor;
    safety_factor.type = type;
    safety_factor.points.emplace_back(point);
    safety_factor.is_safe = false;
    return safety_factor;
  };

  if (collision_point_objects) {
    RCLCPP_WARN_THROTTLE(
      get_node_ptr()->get_logger(), *get_clock(), 1000,
      "[TM ObstacleStop] Detected collision with object at arc length %f m",
      collision_point_objects->arc_length);
    if (debug_data_.colliding_object) {
      auto safety_factor = get_safety_factor(
        debug_data_.colliding_object->kinematics.initial_pose_with_covariance.pose.position,
        SafetyFactor::OBJECT);
      safety_factor.object_id = debug_data_.colliding_object->object_id;
      safety_factors_.factors.push_back(safety_factor);
    }
  }

  if (collision_point_pcd) {
    RCLCPP_WARN_THROTTLE(
      get_node_ptr()->get_logger(), *get_clock(), 1000,
      "[TM ObstacleStop] Detected collision with pointcloud at arc length %f m",
      collision_point_pcd->arc_length);
    auto safety_factor = get_safety_factor(collision_point_pcd->point, SafetyFactor::POINTCLOUD);
    safety_factors_.factors.push_back(safety_factor);
  }

  nearest_collision_point_ = std::invoke([&]() -> std::optional<CollisionPoint> {
    const auto is_collision_point_pcd = params_.enable_stop_for_pointcloud && collision_point_pcd;
    const auto is_collision_point_objects =
      params_.enable_stop_for_objects && collision_point_objects;
    if (!is_collision_point_pcd && !is_collision_point_objects) return std::nullopt;
    if (!is_collision_point_pcd) return collision_point_objects.value();
    if (!is_collision_point_objects) return collision_point_pcd.value();
    return collision_point_pcd->arc_length < collision_point_objects->arc_length
             ? collision_point_pcd.value()
             : collision_point_objects.value();
  });
}

void ObstacleStop::detect_objects(
  const TrajectoryPoints & traj_points, const TrajectoryModifierData & input)
{
  autoware_utils_debug::ScopedTimeTrack st("ObstacleStop::detect_objects", *get_time_keeper());
  if (!params_.use_objects || !input.predicted_objects) return;

  debug_data_.filtered_objects = *input.predicted_objects;

  object_filter_->filter_objects(debug_data_.filtered_objects);

  obstacle_tracker_->update_objects(
    debug_data_.filtered_objects, debug_data_.target_objects, get_clock()->now());

  object_filter_->filter_by_target_area(
    debug_data_.target_objects, traj_points, context_->vehicle_info, debug_data_.trajectory_shape,
    lateral_margin_map_);
}

std::optional<CollisionPoint> ObstacleStop::evaluate_object_collision(
  const TrajectoryPoints & traj_points)
{
  autoware_utils_debug::ScopedTimeTrack st(
    "ObstacleStop::evaluate_object_collision", *get_time_keeper());
  if (debug_data_.target_objects.empty()) return std::nullopt;

  auto collision_point = get_nearest_object_collision(
    debug_data_.target_objects, traj_points, context_->vehicle_info, object_decel_map_,
    params_.rss_params.ego_decel, stopping_params_.delay_response_time, params_.stop_margin,
    params_.rss_params.safety_margin, params_.objects.stopped_velocity_th,
    params_.rss_params.lookahead_horizon, params_.rss_params.enable);

  if (collision_point) {
    auto it = std::find_if(
      debug_data_.target_objects.begin(), debug_data_.target_objects.end(),
      [&](const TargetObject & object) { return !object.is_safe; });
    if (it != debug_data_.target_objects.end()) {
      debug_data_.colliding_object = it->object;
    }
  }

  return collision_point;
}

std::optional<CollisionPoint> ObstacleStop::check_pointcloud(
  [[maybe_unused]] const TrajectoryPoints & traj_points, const TrajectoryModifierData & input)
{
  autoware_utils_debug::ScopedTimeTrack st("ObstacleStop::check_pointcloud", *get_time_keeper());
  if (!params_.use_pointcloud || !input.obstacle_pointcloud) return std::nullopt;

  PointCloud::Ptr filtered_pointcloud(new PointCloud);
  pcl::fromROSMsg(*input.obstacle_pointcloud, *filtered_pointcloud);

  {
    geometry_msgs::msg::TransformStamped transform_stamped;
    try {
      transform_stamped = context_->tf_buffer.lookupTransform(
        "map", input.obstacle_pointcloud->header.frame_id, tf2::TimePointZero);
    } catch (tf2::TransformException & e) {
      RCLCPP_WARN(get_node_ptr()->get_logger(), "no transform found for pointcloud: %s", e.what());
      return std::nullopt;
    }

    Eigen::Affine3f isometry = tf2::transformToEigen(transform_stamped.transform).cast<float>();
    for (auto & p : filtered_pointcloud->points) {
      const Eigen::Vector3f q = isometry * Eigen::Vector3f(p.x, p.y, p.z);
      p.x = q.x();
      p.y = q.y();
      p.z = q.z();
    }
  }

  {
    autoware_utils_debug::ScopedTimeTrack stt(
      "ObstacleStop::filter_pointcloud", *get_time_keeper());
    const auto & bbox = debug_data_.trajectory_shape.bounding_box;
    constexpr double buffer = 1.0;
    const auto [min_x, max_x] = std::minmax(bbox.min_corner().x(), bbox.max_corner().x());
    const auto [min_y, max_y] = std::minmax(bbox.min_corner().y(), bbox.max_corner().y());
    const auto ego_z = input.current_odometry->pose.pose.position.z;
    const auto min_z = ego_z + params_.objects.pointcloud_min_height;
    const auto max_z =
      ego_z + context_->vehicle_info.vehicle_height_m + params_.objects.pointcloud_height_buffer;
    pointcloud_filter_->filter_pointcloud(
      filtered_pointcloud, min_x - buffer, max_x + buffer, min_y - buffer, max_y + buffer, min_z,
      max_z);
  }

  {
    const auto filtered_pointcloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(*filtered_pointcloud, *filtered_pointcloud_msg);
    filtered_pointcloud_msg->header.stamp = input.obstacle_pointcloud->header.stamp;
    filtered_pointcloud_msg->header.frame_id = "map";
    debug_data_.filtered_points = filtered_pointcloud_msg;
  }

  if (input.predicted_objects && !input.predicted_objects->objects.empty()) {
    autoware_utils_debug::ScopedTimeTrack stt(
      "ObstacleStop::filter_pointcloud_by_object", *get_time_keeper());
    pointcloud_filter_->filter_pointcloud_by_object(filtered_pointcloud, *input.predicted_objects);
  }

  PointCloud::Ptr active_points(new PointCloud);
  obstacle_tracker_->update_points(filtered_pointcloud, active_points, get_clock()->now());

  std::optional<CollisionPoint> collision_point;
  {
    autoware_utils_debug::ScopedTimeTrack stt(
      "ObstacleStop::get_nearest_pcd_collision", *get_time_keeper());
    collision_point = get_nearest_pcd_collision(
      debug_data_.trajectory_shape, active_points, lateral_margin_map_,
      params_.objects.low_velocity_th, debug_data_.target_pcd_points);
  }

  return collision_point;
}

void ObstacleStop::publish_debug_string(bool is_safe) const
{
  const auto filtered_pcd_size =
    debug_data_.filtered_points ? debug_data_.filtered_points->data.size() : 0;
  std::ostringstream ss;
  ss << std::fixed << std::setprecision(2) << std::boolalpha;
  ss << "OBSTACLE STOP MODIFIER: "
     << "\n";
  ss << "\t\t"
     << "SAFE: " << is_safe << "\n";
  ss << "\t\t"
     << "OBJECTS: " << debug_data_.filtered_objects.objects.size() << " --> "
     << debug_data_.target_objects.size() << "\n";
  ss << "\t\t" << "POINTCLOUD: " << filtered_pcd_size << " --> "
     << debug_data_.target_pcd_points.size() << "\n";
  if (nearest_collision_point_) {
    ss << "\t\t"
       << "DISTANCE TO COLLISION: " << nearest_collision_point_->arc_length << " m"
       << "\n";
    ss << "\t\t"
       << "OBSTACLE TYPE: " << (nearest_collision_point_->is_dynamic ? "DYNAMIC" : "STATIC")
       << "\n";
  }
  if (cruise_report_) {
    ss << "\t\t"
       << "CRUISE: gap " << cruise_report_->gap() << " m, target " << cruise_report_->target_velocity()
       << " m/s, v_cruise " << cruise_report_->cruise_velocity << " m/s"
       << "\n";
  }

  StringStamped string_stamp;
  string_stamp.stamp = get_clock()->now();
  string_stamp.data = ss.str();
  pub_debug_text_->publish(string_stamp);
}

void ObstacleStop::publish_debug_data(const std::string & ns) const
{
  autoware_utils_debug::ScopedTimeTrack st("ObstacleStop::publish_debug_data", *get_time_keeper());
  if (debug_data_.filtered_points) pub_filtered_pointcloud_->publish(*debug_data_.filtered_points);

  MarkerArray marker_array;
  const auto ego_z = debug_data_.ego_z;
  const auto white = autoware_utils::create_marker_color(1.0, 1.0, 1.0, 1.0);
  const auto yellow = autoware_utils::create_marker_color(1.0, 1.0, 0.0, 1.0);
  const auto magenta = autoware_utils::create_marker_color(1.0, 0.0, 1.0, 1.0);

  auto add_line_list_marker = [&](
                                const std::vector<geometry_msgs::msg::Point> & segments,
                                const std::string & ns, const int id,
                                const std_msgs::msg::ColorRGBA & color) {
    Marker marker = autoware_utils::create_default_marker(
      "map", get_clock()->now(), ns, id, Marker::LINE_LIST,
      autoware_utils::create_marker_scale(0.1, 0.1, 0.1), color);
    marker.lifetime = rclcpp::Duration::from_seconds(0.2);
    marker.points = segments;
    marker_array.markers.push_back(marker);
  };

  int id = 0;
  {
    using autoware_utils_geometry::calc_offset_pose;
    const auto & shape = debug_data_.trajectory_shape;
    const auto half_width = shape.ego_half_width;
    const auto front_offset = std::abs(shape.ego_front_offset);
    const auto back_offset = std::abs(shape.ego_back_offset);
    std::vector<geometry_msgs::msg::Point> segments;
    auto dist = 0.0;
    for (const auto & footprint : shape.footprints) {
      if (footprint.arc_length - dist < 1.0) continue;
      dist = footprint.arc_length;
      auto front_left = calc_offset_pose(footprint.pose, front_offset, -half_width, 0.0).position;
      auto front_right = calc_offset_pose(footprint.pose, front_offset, half_width, 0.0).position;
      auto rear_right = calc_offset_pose(footprint.pose, -back_offset, half_width, 0.0).position;
      auto rear_left = calc_offset_pose(footprint.pose, -back_offset, -half_width, 0.0).position;
      segments.emplace_back(rear_left);
      segments.emplace_back(front_left);
      segments.emplace_back(front_left);
      segments.emplace_back(front_right);
      segments.emplace_back(front_right);
      segments.emplace_back(rear_right);
      segments.emplace_back(rear_right);
      segments.emplace_back(rear_left);
    }
    add_line_list_marker(segments, ns + "/traj_polygon", id, yellow);
    id++;
  }

  auto add_polygon_marker = [&](
                              const autoware_utils_geometry::Polygon2d & polygon,
                              const std::string & ns, const int id,
                              const std_msgs::msg::ColorRGBA & color) {
    Marker marker = autoware_utils::create_default_marker(
      "map", get_clock()->now(), ns, id, Marker::LINE_STRIP,
      autoware_utils::create_marker_scale(0.1, 0.1, 0.1), color);
    marker.lifetime = rclcpp::Duration::from_seconds(0.2);

    for (const auto & p : polygon.outer()) {
      marker.points.push_back(autoware_utils_geometry::create_point(p.x(), p.y(), ego_z));
    }
    if (!marker.points.empty()) {
      marker.points.push_back(marker.points.front());
    }
    marker_array.markers.push_back(marker);
  };

  auto add_text_marker = [&](
                           const std::string & text, const geometry_msgs::msg::Pose & pose,
                           const std::string & ns, const int id,
                           const std_msgs::msg::ColorRGBA & color, const double scale = 0.4) {
    Marker marker = autoware_utils::create_default_marker(
      "map", get_clock()->now(), ns, id, Marker::TEXT_VIEW_FACING,
      autoware_utils::create_marker_scale(scale, scale, scale), color);
    marker.lifetime = rclcpp::Duration::from_seconds(0.2);
    marker.pose = pose;
    marker.pose.position.z += 1.0;
    marker.text = text;
    marker_array.markers.push_back(marker);
  };

  {
    const auto & bounding_box = debug_data_.trajectory_shape.bounding_box;
    Polygon2d polygon;
    polygon.outer().emplace_back(bounding_box.min_corner());
    polygon.outer().emplace_back(bounding_box.min_corner().x(), bounding_box.max_corner().y());
    polygon.outer().emplace_back(bounding_box.max_corner());
    polygon.outer().emplace_back(bounding_box.max_corner().x(), bounding_box.min_corner().y());
    add_polygon_marker(polygon, ns + "/traj_bounding_box", id, white);
    id++;
  }

  for (const auto & obj : debug_data_.target_objects) {
    add_polygon_marker(obj.polygon, ns + "/target_objects", id, magenta);
    std::stringstream ss;
    ss << std::fixed << std::setprecision(2);
    ss << "safe: " << (obj.is_safe ? "true" : "false") << "\n";
    ss << "rss_safe_dist: " << obj.safe_distance << " m" << "\n";
    ss << "dist_from_ego: " << obj.distance_from_ego << " m";
    add_text_marker(
      ss.str(), obj.object.kinematics.initial_pose_with_covariance.pose,
      ns + "/target_objects_text", id, white);
    if (!obj.is_safe)
      add_polygon_marker(obj.ego_footprint, ns + "/overlapping_ego_footprint", id, magenta);
    id++;
  }

  auto add_point_marker = [&](
                            const geometry_msgs::msg::Point & point, const std::string & ns,
                            const int id, const std_msgs::msg::ColorRGBA & color,
                            const double scale = 0.1) {
    Marker marker = autoware_utils::create_default_marker(
      "map", get_clock()->now(), ns, id, Marker::SPHERE,
      autoware_utils::create_marker_scale(scale, scale, scale), color);
    marker.lifetime = rclcpp::Duration::from_seconds(0.2);
    marker.pose.position = point;
    marker_array.markers.push_back(marker);
  };

  for (const auto & target_pcd_point : debug_data_.target_pcd_points) {
    add_point_marker(target_pcd_point, ns + "/target_pcd", id, magenta, 0.25);
    id++;
  }

  debug_viz_pub_->publish(marker_array);
}

}  // namespace autoware::trajectory_modifier::plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::trajectory_modifier::plugin::ObstacleStop,
  autoware::trajectory_modifier::plugin::TrajectoryModifierPluginBase)
