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

#include "autoware/diffusion_planner/diffusion_planner_core.hpp"

#include "autoware/diffusion_planner/conversion/agent.hpp"
#include "autoware/diffusion_planner/dimensions.hpp"
#include "autoware/diffusion_planner/postprocessing/postprocessing_utils.hpp"
#include "autoware/diffusion_planner/preprocessing/preprocessing_utils.hpp"
#include "autoware/diffusion_planner/utils/utils.hpp"

#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_utils_geometry/geometry.hpp>

#include <autoware_internal_planning_msgs/msg/candidate_trajectory.hpp>
#include <autoware_internal_planning_msgs/msg/generator_info.hpp>

#include <lanelet2_core/geometry/LaneletMap.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace autoware::diffusion_planner
{

namespace
{

// Mimic of autoware_perfect_tracker's Z interpolation along the followed trajectory.
double interpolate_z_from_trajectory(
  const Trajectory & trajectory, const double x, const double y, const double fallback_z)
{
  if (trajectory.points.size() < 2) {
    return fallback_z;
  }

  double min_dist_sq = std::numeric_limits<double>::max();
  size_t closest_idx = 0;
  for (size_t i = 0; i < trajectory.points.size(); ++i) {
    const auto & p = trajectory.points[i];
    const double dx = p.pose.position.x - x;
    const double dy = p.pose.position.y - y;
    const double d2 = dx * dx + dy * dy;
    if (d2 < min_dist_sq) {
      min_dist_sq = d2;
      closest_idx = i;
    }
  }

  size_t prev_idx = closest_idx;
  size_t next_idx = closest_idx + 1;
  if (next_idx >= trajectory.points.size()) {
    next_idx = closest_idx;
    prev_idx = (closest_idx > 0) ? closest_idx - 1 : 0;
  } else if (closest_idx > 0) {
    const auto & p_curr = trajectory.points[closest_idx];
    const auto & p_next = trajectory.points[next_idx];
    const double dx = p_next.pose.position.x - p_curr.pose.position.x;
    const double dy = p_next.pose.position.y - p_curr.pose.position.y;
    const double vx = x - p_curr.pose.position.x;
    const double vy = y - p_curr.pose.position.y;
    if ((dx * vx + dy * vy) < 0.0) {
      next_idx = closest_idx;
      prev_idx = closest_idx - 1;
    }
  }

  const auto & p1 = trajectory.points[prev_idx];
  const auto & p2 = trajectory.points[next_idx];
  const double dx = p2.pose.position.x - p1.pose.position.x;
  const double dy = p2.pose.position.y - p1.pose.position.y;
  const double dz = p2.pose.position.z - p1.pose.position.z;
  const double len_sq = dx * dx + dy * dy;
  if (len_sq < 1e-6) {
    return p1.pose.position.z;
  }
  const double vx = x - p1.pose.position.x;
  const double vy = y - p1.pose.position.y;
  double t = (vx * dx + vy * dy) / len_sq;
  t = std::clamp(t, 0.0, 1.0);
  return p1.pose.position.z + t * dz;
}

// Mimic of autoware_perfect_tracker's pitch lookup from the nearest lanelet centerline.
double calculate_pitch_from_map(
  const lanelet::LaneletMap & lanelet_map, const geometry_msgs::msg::Pose & pose)
{
  const lanelet::BasicPoint2d search_point(pose.position.x, pose.position.y);
  const auto nearest_lanelets =
    lanelet::geometry::findNearest(lanelet_map.laneletLayer, search_point, 1);
  if (nearest_lanelets.empty()) {
    return 0.0;
  }

  const lanelet::ConstLanelet ego_lanelet = nearest_lanelets[0].second;
  std::vector<geometry_msgs::msg::Point> centerline_points;
  for (const auto & point : ego_lanelet.centerline()) {
    geometry_msgs::msg::Point cp;
    cp.x = point.basicPoint().x();
    cp.y = point.basicPoint().y();
    cp.z = point.basicPoint().z();
    centerline_points.push_back(cp);
  }
  if (centerline_points.size() < 2) {
    return 0.0;
  }

  const size_t seg_idx =
    autoware::motion_utils::findNearestSegmentIndex(centerline_points, pose.position);
  const auto & prev_point = centerline_points.at(seg_idx);
  const auto & next_point = centerline_points.at(seg_idx + 1);

  const double road_yaw =
    std::atan2(next_point.y - prev_point.y, next_point.x - prev_point.x);
  const double car_yaw = tf2::getYaw(pose.orientation);
  const double yaw_diff = car_yaw - road_yaw;
  const double diff_z = next_point.z - prev_point.z;
  const double diff_xy = std::hypot(next_point.x - prev_point.x, next_point.y - prev_point.y);
  const double projected_run = diff_xy / std::cos(yaw_diff);
  const bool reverse_sign = std::cos(yaw_diff) < 0.0;
  return reverse_sign ? -std::atan2(-diff_z, -projected_run)
                      : -std::atan2(diff_z, projected_run);
}

// One tick of autoware_perfect_tracker's update algorithm applied to `odom` in place.
// Snaps to the closest trajectory point, Euler-integrates X/Y with the current yaw,
// snaps yaw to the trajectory, then updates Z and pitch from the trajectory and map.
void apply_perfect_tracker_step(
  nav_msgs::msg::Odometry & odom, geometry_msgs::msg::AccelWithCovarianceStamped & accel,
  const Trajectory & trajectory, const lanelet::LaneletMap * lanelet_map_ptr, const double dt)
{
  if (trajectory.points.empty()) {
    return;
  }

  const double cx = odom.pose.pose.position.x;
  const double cy = odom.pose.pose.position.y;

  double min_dist_sq = std::numeric_limits<double>::max();
  size_t closest_idx = 0;
  for (size_t i = 0; i < trajectory.points.size(); ++i) {
    const auto & p = trajectory.points[i];
    const double dx = cx - p.pose.position.x;
    const double dy = cy - p.pose.position.y;
    const double d2 = dx * dx + dy * dy;
    if (d2 < min_dist_sq) {
      min_dist_sq = d2;
      closest_idx = i;
    }
  }
  const auto & target_pt = trajectory.points[closest_idx];

  // Set dynamics targets from the trajectory point (same as perfect_tracker).
  odom.twist.twist.linear.x = target_pt.longitudinal_velocity_mps;
  odom.twist.twist.angular.z = target_pt.heading_rate_rps;
  accel.accel.accel.linear.x = target_pt.acceleration_mps2;

  // Euler integration on X/Y using the OLD yaw and the target longitudinal velocity.
  const double current_yaw = tf2::getYaw(odom.pose.pose.orientation);
  const double v_target = target_pt.longitudinal_velocity_mps;
  odom.pose.pose.position.x = cx + v_target * std::cos(current_yaw) * dt;
  odom.pose.pose.position.y = cy + v_target * std::sin(current_yaw) * dt;

  // Snap yaw to the trajectory point yaw.
  const double next_yaw = tf2::getYaw(target_pt.pose.orientation);

  // Z from trajectory interpolation at the NEW (x, y).
  odom.pose.pose.position.z = interpolate_z_from_trajectory(
    trajectory, odom.pose.pose.position.x, odom.pose.pose.position.y,
    odom.pose.pose.position.z);

  // Pitch from map at the NEW (x, y); roll is forced to zero like perfect_tracker.
  // Perfect_tracker evaluates pitch before applying next_yaw, so the pose here still
  // carries the pre-update yaw — match that behavior.
  double pitch = 0.0;
  if (lanelet_map_ptr != nullptr) {
    pitch = calculate_pitch_from_map(*lanelet_map_ptr, odom.pose.pose);
  }

  odom.pose.pose.orientation =
    autoware_utils_geometry::create_quaternion_from_rpy(0.0, pitch, next_yaw);
}

}  // namespace

DiffusionPlannerCore::DiffusionPlannerCore(
  const DiffusionPlannerParams & params, const VehicleInfo & vehicle_info)
: params_(params),
  vehicle_spec_(vehicle_info),
  turn_indicator_manager_(
    rclcpp::Duration::from_seconds(params.turn_indicator_hold_duration),
    params.turn_indicator_keep_offset)
{
}

void DiffusionPlannerCore::load_model()
{
  tensorrt_inference_.reset();
  utils::check_weight_version(params_.args_path);
  normalization_map_ = utils::load_normalization_stats(params_.args_path);
  tensorrt_inference_ = std::make_unique<TensorrtInference>(
    params_.model_path, params_.plugins_path, params_.batch_size);
}

void DiffusionPlannerCore::update_params(const DiffusionPlannerParams & params)
{
  params_ = params;
  turn_indicator_manager_.set_hold_duration(
    rclcpp::Duration::from_seconds(params_.turn_indicator_hold_duration));
  turn_indicator_manager_.set_keep_offset(params_.turn_indicator_keep_offset);
}

void DiffusionPlannerCore::set_map(
  const std::shared_ptr<const lanelet::LaneletMap> & lanelet_map_ptr)
{
  lanelet_map_ptr_ = lanelet_map_ptr;
  lane_segment_context_ = std::make_unique<preprocess::LaneSegmentContext>(
    lanelet_map_ptr, params_.line_string_max_step_m);
}

std::optional<FrameContext> DiffusionPlannerCore::create_frame_context(
  const std::shared_ptr<const Odometry> & ego_kinematic_state,
  const std::shared_ptr<const AccelWithCovarianceStamped> & ego_acceleration,
  const std::shared_ptr<const TrackedObjects> & objects,
  const std::vector<std::shared_ptr<const autoware_perception_msgs::msg::TrafficLightGroupArray>> &
    traffic_signals,
  const std::shared_ptr<const TurnIndicatorsReport> & turn_indicators,
  const LaneletRoute::ConstSharedPtr & route_ptr, const rclcpp::Time & current_time)
{
  route_ptr_ = (!route_ptr_ || route_ptr) ? route_ptr : route_ptr_;

  TrackedObjects empty_object_list;
  auto effective_objects = objects;

  if (params_.ignore_neighbors) {
    effective_objects = std::make_shared<TrackedObjects>(empty_object_list);
  }

  if (!effective_objects || !ego_kinematic_state || !ego_acceleration || !turn_indicators) {
    return std::nullopt;
  }

  if (!route_ptr_) {
    return std::nullopt;
  }

  // --- Internal "perfect tracker" -----------------------------------------------------------
  // Drive a virtual ego state forward using the previous output trajectory, and use that
  // virtual state in place of the observed kinematic state. This mirrors what
  // autoware_perfect_tracker would do externally, but keeps everything self-contained so that
  // inference is conditioned on a world in which control is assumed to follow perfectly.
  if (!virtual_ego_state_) {
    virtual_ego_state_ = *ego_kinematic_state;
    virtual_ego_acceleration_ = *ego_acceleration;
  } else if (last_output_trajectory_ && !last_output_trajectory_->points.empty()) {
    const double dt = (params_.planning_frequency_hz > 0.0)
                        ? 1.0 / params_.planning_frequency_hz
                        : 0.1;
    apply_perfect_tracker_step(
      *virtual_ego_state_, *virtual_ego_acceleration_, *last_output_trajectory_,
      lanelet_map_ptr_.get(), dt);
  }

  // Safety fallback: if the virtual state has drifted more than 1 m from the real EKF
  // position, snap it back to the real state so the model never sees a grossly stale pose.
  constexpr double kVirtualStateMaxDriftM = 1.0;
  const double drift_dx =
    virtual_ego_state_->pose.pose.position.x - ego_kinematic_state->pose.pose.position.x;
  const double drift_dy =
    virtual_ego_state_->pose.pose.position.y - ego_kinematic_state->pose.pose.position.y;
  const double drift_dz =
    virtual_ego_state_->pose.pose.position.z - ego_kinematic_state->pose.pose.position.z;
  if (
    (drift_dx * drift_dx + drift_dy * drift_dy + drift_dz * drift_dz) >
    (kVirtualStateMaxDriftM * kVirtualStateMaxDriftM)) {
    virtual_ego_state_ = *ego_kinematic_state;
    virtual_ego_acceleration_ = *ego_acceleration;
  }
  // Preserve incoming header/frame ids but swap in the virtual pose, twist, and acceleration.
  auto effective_ego_kinematic_state = std::make_shared<Odometry>(*ego_kinematic_state);
  effective_ego_kinematic_state->pose.pose = virtual_ego_state_->pose.pose;
  effective_ego_kinematic_state->twist.twist = virtual_ego_state_->twist.twist;
  auto effective_ego_acceleration =
    std::make_shared<AccelWithCovarianceStamped>(*ego_acceleration);
  effective_ego_acceleration->accel.accel = virtual_ego_acceleration_->accel.accel;

  Odometry kinematic_state = *effective_ego_kinematic_state;
  if (params_.shift_x) {
    kinematic_state.pose.pose =
      utils::shift_x(kinematic_state.pose.pose, vehicle_spec_.base_link_to_center);
  }

  // Get transforms
  const geometry_msgs::msg::Pose & pose_base_link = kinematic_state.pose.pose;
  const Eigen::Matrix4d ego_to_map_transform = utils::pose_to_matrix4d(pose_base_link);
  const Eigen::Matrix4d map_to_ego_transform = utils::inverse(ego_to_map_transform);

  // Update ego history
  ego_history_.push_back(kinematic_state);
  if (ego_history_.size() > static_cast<size_t>(EGO_HISTORY_SHAPE[1])) {
    ego_history_.pop_front();
  }

  // Update turn indicators history
  turn_indicators_history_.push_back(*turn_indicators);
  if (turn_indicators_history_.size() > static_cast<size_t>(TURN_INDICATORS_SHAPE[1])) {
    turn_indicators_history_.pop_front();
  }

  // Update neighbor agent data
  agent_data_.update_histories(*effective_objects, params_.ignore_unknown_neighbors);
  const auto processed_neighbor_histories =
    agent_data_.transformed_and_trimmed_histories(map_to_ego_transform, NEIGHBOR_SHAPE[1]);

  // Update traffic light map
  const auto & traffic_light_msg_timeout_s = params_.traffic_light_group_msg_timeout_seconds;
  preprocess::process_traffic_signals(
    traffic_signals, traffic_light_id_map_, current_time, traffic_light_msg_timeout_s);

  // Create frame context — note we use the virtual (perfect-tracker-driven) ego state here
  // so that downstream consumers (base position for trajectory, twist checks, etc.) see the
  // same state the model was conditioned on.
  const rclcpp::Time frame_time(ego_kinematic_state->header.stamp);
  const FrameContext frame_context{
    *effective_ego_kinematic_state, *effective_ego_acceleration, ego_to_map_transform,
    processed_neighbor_histories, frame_time};

  return frame_context;
}

InputDataMap DiffusionPlannerCore::create_input_data(const FrameContext & frame_context)
{
  InputDataMap input_data_map;

  const geometry_msgs::msg::Pose & pose_center =
    params_.shift_x
      ? utils::shift_x(
          frame_context.ego_kinematic_state.pose.pose, vehicle_spec_.base_link_to_center)
      : frame_context.ego_kinematic_state.pose.pose;
  const Eigen::Matrix4d ego_to_map_transform = utils::pose_to_matrix4d(pose_center);
  const Eigen::Matrix4d map_to_ego_transform = utils::inverse(ego_to_map_transform);
  const auto & center_x = static_cast<float>(pose_center.position.x);
  const auto & center_y = static_cast<float>(pose_center.position.y);
  const auto & center_z = static_cast<float>(pose_center.position.z);

  // random sample trajectories
  int64_t delay_step = 0;
  {
    const int64_t copy_steps = std::min<int64_t>(params_.delay_step, OUTPUT_T);
    const bool has_previous_output = !last_agent_poses_map_.empty();

    for (int64_t b = 0; b < params_.batch_size; b++) {
      std::vector<float> sampled_trajectories =
        preprocess::create_sampled_trajectories(params_.temperature_list[b]);

      if (has_previous_output) {
        constexpr int64_t agent_idx = 0;
        delay_step = params_.delay_step;
        for (int64_t t = 0; t <= copy_steps; ++t) {
          const size_t dst_base = agent_idx * (OUTPUT_T + 1) * POSE_DIM + (t)*POSE_DIM;
          const Eigen::Matrix4d pose_ego =
            map_to_ego_transform * last_agent_poses_map_[b][agent_idx][t];
          const float shifted_x = static_cast<float>(pose_ego(0, 3));
          const float shifted_y = static_cast<float>(pose_ego(1, 3));
          const auto [shifted_cos, shifted_sin] =
            utils::rotation_matrix_to_cos_sin(pose_ego.block<3, 3>(0, 0));

          sampled_trajectories[dst_base + 0] = (shifted_x - 10.0f) / 20.0f;
          sampled_trajectories[dst_base + 1] = shifted_y / 20.0f;
          sampled_trajectories[dst_base + 2] = shifted_cos;
          sampled_trajectories[dst_base + 3] = shifted_sin;
        }
      }

      input_data_map["sampled_trajectories"].insert(
        input_data_map["sampled_trajectories"].end(), sampled_trajectories.begin(),
        sampled_trajectories.end());
    }
  }

  // Ego history
  {
    const std::optional<rclcpp::Time> reference_time =
      params_.use_time_interpolation ? std::make_optional(frame_context.frame_time) : std::nullopt;
    const std::vector<float> single_ego_agent_past = preprocess::create_ego_agent_past(
      ego_history_, EGO_HISTORY_SHAPE[1], map_to_ego_transform, reference_time);
    input_data_map["ego_agent_past"] =
      utils::replicate_for_batch(single_ego_agent_past, params_.batch_size);
  }
  // Ego state
  {
    const auto ego_current_state = preprocess::create_ego_current_state(
      frame_context.ego_kinematic_state, frame_context.ego_acceleration,
      static_cast<float>(vehicle_spec_.wheel_base));
    input_data_map["ego_current_state"] =
      utils::replicate_for_batch(ego_current_state, params_.batch_size);
  }
  // Agent data on ego reference frame
  {
    const auto neighbor_agents_past = flatten_histories_to_vector(
      frame_context.ego_centric_neighbor_histories, MAX_NUM_NEIGHBORS, INPUT_T + 1);
    input_data_map["neighbor_agents_past"] =
      utils::replicate_for_batch(neighbor_agents_past, params_.batch_size);
  }
  // Static objects
  // TODO(Daniel): add static objects
  {
    std::vector<int64_t> single_batch_shape(
      STATIC_OBJECTS_SHAPE.begin() + 1, STATIC_OBJECTS_SHAPE.end());
    auto static_objects_data = utils::create_float_data(single_batch_shape, 0.0f);
    input_data_map["static_objects"] =
      utils::replicate_for_batch(static_objects_data, params_.batch_size);
  }

  // map data on ego reference frame
  {
    const std::vector<int64_t> segment_indices = lane_segment_context_->select_lane_segment_indices(
      map_to_ego_transform, center_x, center_y, NUM_SEGMENTS_IN_LANE);
    const auto [lanes, lanes_speed_limit] = lane_segment_context_->create_tensor_data_from_indices(
      map_to_ego_transform, traffic_light_id_map_, segment_indices, NUM_SEGMENTS_IN_LANE);
    input_data_map["lanes"] = utils::replicate_for_batch(lanes, params_.batch_size);
    input_data_map["lanes_speed_limit"] =
      utils::replicate_for_batch(lanes_speed_limit, params_.batch_size);
  }

  // route data on ego reference frame
  {
    const std::vector<int64_t> segment_indices =
      lane_segment_context_->select_route_segment_indices(
        *route_ptr_, center_x, center_y, center_z, NUM_SEGMENTS_IN_ROUTE);
    const auto [route_lanes, route_lanes_speed_limit] =
      lane_segment_context_->create_tensor_data_from_indices(
        map_to_ego_transform, traffic_light_id_map_, segment_indices, NUM_SEGMENTS_IN_ROUTE);
    input_data_map["route_lanes"] = utils::replicate_for_batch(route_lanes, params_.batch_size);
    input_data_map["route_lanes_speed_limit"] =
      utils::replicate_for_batch(route_lanes_speed_limit, params_.batch_size);
  }

  // polygons
  {
    const auto & polygons =
      lane_segment_context_->create_polygon_tensor(map_to_ego_transform, center_x, center_y);
    input_data_map["polygons"] = utils::replicate_for_batch(polygons, params_.batch_size);
  }

  // line strings
  {
    const auto & line_strings =
      lane_segment_context_->create_line_string_tensor(map_to_ego_transform, center_x, center_y);
    input_data_map["line_strings"] = utils::replicate_for_batch(line_strings, params_.batch_size);
  }

  // goal pose
  {
    const auto & goal_pose = route_ptr_->goal_pose;

    // Convert goal pose to 4x4 transformation matrix
    const Eigen::Matrix4d goal_pose_map_4x4 = utils::pose_to_matrix4d(goal_pose);

    // Transform to ego frame
    const Eigen::Matrix4d goal_pose_ego_4x4 = map_to_ego_transform * goal_pose_map_4x4;

    // Extract relative position
    const float x = goal_pose_ego_4x4(0, 3);
    const float y = goal_pose_ego_4x4(1, 3);

    // Extract heading as cos/sin from rotation matrix
    const auto [cos_yaw, sin_yaw] =
      utils::rotation_matrix_to_cos_sin(goal_pose_ego_4x4.block<3, 3>(0, 0));

    std::vector<float> single_goal_pose = {x, y, cos_yaw, sin_yaw};
    input_data_map["goal_pose"] = utils::replicate_for_batch(single_goal_pose, params_.batch_size);
  }

  // ego shape
  {
    const std::vector<float> single_ego_shape = {
      static_cast<float>(vehicle_spec_.wheel_base),
      static_cast<float>(vehicle_spec_.vehicle_length),
      static_cast<float>(vehicle_spec_.vehicle_width)};
    input_data_map["ego_shape"] = utils::replicate_for_batch(single_ego_shape, params_.batch_size);
  }

  // turn indicators
  {
    // copy from back to front, and use the front value for padding if not enough history
    std::vector<float> single_turn_indicators(INPUT_T + 1, 0.0f);
    for (int64_t t = 0; t < INPUT_T + 1; ++t) {
      const int64_t index = std::max(
        static_cast<int64_t>(turn_indicators_history_.size()) - 1 - t, static_cast<int64_t>(0));
      single_turn_indicators[INPUT_T - t] = turn_indicators_history_[index].report;
    }
    input_data_map["turn_indicators"] =
      utils::replicate_for_batch(single_turn_indicators, params_.batch_size);
  }

  // control delay
  {
    std::vector<float> single_delay = {static_cast<float>(delay_step)};
    input_data_map["delay"] = utils::replicate_for_batch(single_delay, params_.batch_size);
  }

  return input_data_map;
}

TensorrtInference::InferenceResult DiffusionPlannerCore::run_inference(
  const InputDataMap & input_data_map)
{
  if (!tensorrt_inference_) {
    TensorrtInference::InferenceResult result;
    result.error_msg = "Model not loaded";
    return result;
  }
  return tensorrt_inference_->infer(input_data_map);
}

PlannerOutput DiffusionPlannerCore::create_planner_output(
  const std::vector<float> & predictions, const std::vector<float> & turn_indicator_logit,
  const FrameContext & frame_context, const rclcpp::Time & timestamp, const UUID & generator_uuid)
{
  const auto agent_poses =
    postprocess::parse_predictions(predictions, frame_context.ego_to_map_transform);
  last_agent_poses_map_ = agent_poses;

  const bool enable_force_stop =
    frame_context.ego_kinematic_state.twist.twist.linear.x > std::numeric_limits<double>::epsilon();

  PlannerOutput output;

  // Trajectory and CandidateTrajectories
  for (int i = 0; i < params_.batch_size; i++) {
    auto trajectory = postprocess::create_ego_trajectory(
      agent_poses, timestamp, frame_context.ego_kinematic_state.pose.pose.position, i,
      params_.velocity_smoothing_window, enable_force_stop, params_.stopping_threshold);

    if (params_.shift_x) {
      for (auto & point : trajectory.points) {
        point.pose = utils::shift_x(point.pose, -vehicle_spec_.base_link_to_center);
      }
    }

    if (i == 0) {
      // Use the first trajectory as the main output trajectory
      output.trajectory = trajectory;
    }

    const auto candidate_trajectory = autoware_internal_planning_msgs::build<
                                        autoware_internal_planning_msgs::msg::CandidateTrajectory>()
                                        .header(trajectory.header)
                                        .generator_id(generator_uuid)
                                        .points(trajectory.points);

    std_msgs::msg::String generator_name_msg;
    generator_name_msg.data = std::string("DiffusionPlanner_batch_") + std::to_string(i);

    const auto generator_info =
      autoware_internal_planning_msgs::build<autoware_internal_planning_msgs::msg::GeneratorInfo>()
        .generator_id(generator_uuid)
        .generator_name(generator_name_msg);

    output.candidate_trajectories.candidate_trajectories.push_back(candidate_trajectory);
    output.candidate_trajectories.generator_info.push_back(generator_info);
  }

  // PredictedObjects
  // Use the first prediction as the main predicted objects
  constexpr int64_t batch_idx = 0;
  output.predicted_objects = postprocess::create_predicted_objects(
    agent_poses, frame_context.ego_centric_neighbor_histories, timestamp, batch_idx);

  // TurnIndicatorsCommand
  // Use the first batch's logit as the main turn indicator command.
  constexpr int64_t turn_indicator_batch_idx = 0;
  const std::vector<float> first_turn_indicator_logit(
    turn_indicator_logit.begin() + TURN_INDICATOR_OUTPUT_DIM * turn_indicator_batch_idx,
    turn_indicator_logit.begin() + TURN_INDICATOR_OUTPUT_DIM * (turn_indicator_batch_idx + 1));
  const int64_t prev_report = turn_indicators_history_.empty()
                                ? TurnIndicatorsReport::DISABLE
                                : turn_indicators_history_.back().report;
  output.turn_indicator_command =
    turn_indicator_manager_.evaluate(first_turn_indicator_logit, timestamp, prev_report);

  // Cache the published trajectory so the next create_frame_context() call can advance the
  // virtual ego state along it, exactly as autoware_perfect_tracker would do externally.
  last_output_trajectory_ = output.trajectory;

  return output;
}

autoware_perception_msgs::msg::TrafficLightGroup
DiffusionPlannerCore::get_first_traffic_light_on_route(const FrameContext & frame_context) const
{
  if (!lane_segment_context_ || !route_ptr_) {
    return autoware_perception_msgs::msg::TrafficLightGroup{};
  }

  const geometry_msgs::msg::Pose & pose_center =
    params_.shift_x
      ? utils::shift_x(
          frame_context.ego_kinematic_state.pose.pose, vehicle_spec_.base_link_to_center)
      : frame_context.ego_kinematic_state.pose.pose;

  const double center_x = pose_center.position.x;
  const double center_y = pose_center.position.y;
  const double center_z = pose_center.position.z;

  return lane_segment_context_->get_first_traffic_light_on_route(
    *route_ptr_, center_x, center_y, center_z, traffic_light_id_map_);
}

int64_t DiffusionPlannerCore::count_valid_elements(
  const InputDataMap & input_data_map, const std::string & data_key) const
{
  const int64_t batch_idx = 0;

  if (data_key == "lanes") {
    return postprocess::count_valid_elements(
      input_data_map.at("lanes"), LANES_SHAPE[1], LANES_SHAPE[2], LANES_SHAPE[3], batch_idx);
  } else if (data_key == "route_lanes") {
    return postprocess::count_valid_elements(
      input_data_map.at("route_lanes"), ROUTE_LANES_SHAPE[1], ROUTE_LANES_SHAPE[2],
      ROUTE_LANES_SHAPE[3], batch_idx);
  } else if (data_key == "polygons") {
    return postprocess::count_valid_elements(
      input_data_map.at("polygons"), POLYGONS_SHAPE[1], POLYGONS_SHAPE[2], POLYGONS_SHAPE[3],
      batch_idx);
  } else if (data_key == "line_strings") {
    return postprocess::count_valid_elements(
      input_data_map.at("line_strings"), LINE_STRINGS_SHAPE[1], LINE_STRINGS_SHAPE[2],
      LINE_STRINGS_SHAPE[3], batch_idx);
  } else if (data_key == "neighbor_agents_past") {
    return postprocess::count_valid_elements(
      input_data_map.at("neighbor_agents_past"), NEIGHBOR_SHAPE[1], NEIGHBOR_SHAPE[2],
      NEIGHBOR_SHAPE[3], batch_idx);
  }

  throw std::invalid_argument("Unknown data_key '" + data_key + "' in count_valid_elements()");
}

}  // namespace autoware::diffusion_planner
