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

#include "autoware/tensorrt_e2e/providers/context_input_provider.hpp"

#include <autoware/diffusion_planner/dimensions.hpp>
#include <autoware/diffusion_planner/preprocessing/preprocessing_utils.hpp>
#include <autoware/diffusion_planner/utils/utils.hpp>
#include <autoware/lanelet2_utils/conversion.hpp>

#include <algorithm>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace autoware::tensorrt_e2e
{

namespace dp = autoware::diffusion_planner;

namespace
{

/// Validate a claimed spec against an expectation, with a readable error on mismatch.
void validate_shape(
  const TensorSpec & spec, const std::vector<int64_t> & expected, const std::string & reason)
{
  if (spec.shape != expected) {
    throw std::runtime_error(
      "Model input '" + spec.name + "' has shape " + shape_to_string(spec.shape) + "; expected " +
      shape_to_string(expected) + " (" + reason + ")");
  }
}

}  // namespace

ContextInputProvider::ContextInputProvider(
  rclcpp::Node & node, const autoware::vehicle_info_utils::VehicleInfo & vehicle_info)
: node_(node),
  wheel_base_(vehicle_info.wheel_base_m),
  vehicle_length_(
    vehicle_info.front_overhang_m + vehicle_info.wheel_base_m + vehicle_info.rear_overhang_m),
  vehicle_width_(
    vehicle_info.left_overhang_m + vehicle_info.wheel_tread_m + vehicle_info.right_overhang_m)
{
  traffic_light_msg_timeout_s_ =
    node_.declare_parameter<double>("context.traffic_light_group_msg_timeout_seconds", 0.2);
  ignore_neighbors_ = node_.declare_parameter<bool>("context.ignore_neighbors", false);
  turn_indicators_enabled_ =
    node_.declare_parameter<bool>("context.turn_indicators.enabled", true);
  ignore_unknown_neighbors_ =
    node_.declare_parameter<bool>("context.ignore_unknown_neighbors", true);
  line_string_max_step_m_ =
    node_.declare_parameter<double>("context.line_string_max_step_m", 5.0);
  use_time_interpolation_ =
    node_.declare_parameter<bool>("context.use_time_interpolation", false);
}

std::vector<std::string> ContextInputProvider::claim_inputs(
  const std::vector<TensorSpec> & engine_inputs)
{
  std::vector<std::string> claimed;
  const auto claim = [&engine_inputs, &claimed](
                       const std::string & name, std::vector<int64_t> & shape_out) -> const TensorSpec * {
    const TensorSpec * spec = find_spec(engine_inputs, name);
    if (spec) {
      shape_out = spec->shape;
      claimed.push_back(name);
    }
    return spec;
  };

  if (const auto * spec = claim("ego_current_state", ego_current_state_shape_)) {
    validate_shape(*spec, {1, 10}, "ego current state feature");
  }
  if (const auto * spec = claim("ego_agent_past", ego_agent_past_shape_)) {
    if (spec->shape.size() != 3 || spec->shape[0] != 1 || spec->shape[2] != dp::POSE_DIM) {
      throw std::runtime_error(
        "Model input 'ego_agent_past' has shape " + shape_to_string(spec->shape) +
        "; expected [1, T, " + std::to_string(dp::POSE_DIM) + "]");
    }
  }
  if (const auto * spec = claim("neighbor_agents_past", neighbor_shape_)) {
    // The history length is fixed by the reused diffusion planner AgentData implementation.
    if (
      spec->shape.size() != 4 || spec->shape[0] != 1 ||
      spec->shape[2] != dp::INPUT_T_WITH_CURRENT ||
      spec->shape[3] != static_cast<int64_t>(dp::AGENT_STATE_DIM)) {
      throw std::runtime_error(
        "Model input 'neighbor_agents_past' has shape " + shape_to_string(spec->shape) +
        "; expected [1, N, " + std::to_string(dp::INPUT_T_WITH_CURRENT) + ", " +
        std::to_string(dp::AGENT_STATE_DIM) +
        "] (the history length is fixed by the diffusion planner feature pipeline)");
    }
  }
  claim("static_objects", static_objects_shape_);  // Always zero-filled; any shape is accepted.

  if (const auto * spec = claim("lanes", lanes_shape_)) {
    if (
      spec->shape.size() != 4 || spec->shape[0] != 1 ||
      spec->shape[2] != dp::POINTS_PER_SEGMENT || spec->shape[3] != dp::SEGMENT_POINT_DIM) {
      throw std::runtime_error(
        "Model input 'lanes' has shape " + shape_to_string(spec->shape) + "; expected [1, S, " +
        std::to_string(dp::POINTS_PER_SEGMENT) + ", " + std::to_string(dp::SEGMENT_POINT_DIM) +
        "]");
    }
  }
  if (const auto * spec = claim("lanes_speed_limit", lanes_speed_limit_shape_)) {
    if (lanes_shape_.empty()) {
      throw std::runtime_error("Model takes 'lanes_speed_limit' but not 'lanes'");
    }
    validate_shape(*spec, {1, lanes_shape_[1], 1}, "one speed limit per lane segment");
  }
  if (const auto * spec = claim("lanes_has_speed_limit", lanes_has_speed_limit_shape_)) {
    if (lanes_shape_.empty()) {
      throw std::runtime_error("Model takes 'lanes_has_speed_limit' but not 'lanes'");
    }
    validate_shape(*spec, {1, lanes_shape_[1], 1}, "one speed limit flag per lane segment");
  }

  if (const auto * spec = claim("route_lanes", route_lanes_shape_)) {
    if (
      spec->shape.size() != 4 || spec->shape[0] != 1 ||
      spec->shape[2] != dp::POINTS_PER_SEGMENT || spec->shape[3] != dp::SEGMENT_POINT_DIM) {
      throw std::runtime_error(
        "Model input 'route_lanes' has shape " + shape_to_string(spec->shape) +
        "; expected [1, S, " + std::to_string(dp::POINTS_PER_SEGMENT) + ", " +
        std::to_string(dp::SEGMENT_POINT_DIM) + "]");
    }
  }
  if (const auto * spec = claim("route_lanes_speed_limit", route_lanes_speed_limit_shape_)) {
    if (route_lanes_shape_.empty()) {
      throw std::runtime_error("Model takes 'route_lanes_speed_limit' but not 'route_lanes'");
    }
    validate_shape(*spec, {1, route_lanes_shape_[1], 1}, "one speed limit per route segment");
  }
  if (const auto * spec = claim("route_lanes_has_speed_limit", route_lanes_has_speed_limit_shape_)) {
    if (route_lanes_shape_.empty()) {
      throw std::runtime_error("Model takes 'route_lanes_has_speed_limit' but not 'route_lanes'");
    }
    validate_shape(
      *spec, {1, route_lanes_shape_[1], 1}, "one speed limit flag per route segment");
  }

  if (const auto * spec = claim("polygons", polygons_shape_)) {
    validate_shape(
      *spec,
      std::vector<int64_t>(dp::POLYGONS_SHAPE.begin(), dp::POLYGONS_SHAPE.end()),
      "fixed by the diffusion planner feature pipeline");
  }
  if (const auto * spec = claim("line_strings", line_strings_shape_)) {
    validate_shape(
      *spec,
      std::vector<int64_t>(dp::LINE_STRINGS_SHAPE.begin(), dp::LINE_STRINGS_SHAPE.end()),
      "fixed by the diffusion planner feature pipeline");
  }
  if (const auto * spec = claim("goal_pose", goal_pose_shape_)) {
    validate_shape(*spec, {1, dp::POSE_DIM}, "goal pose as x, y, cos(yaw), sin(yaw)");
  }
  if (const auto * spec = claim("ego_shape", ego_shape_shape_)) {
    validate_shape(*spec, {1, 3}, "wheel base, length, width");
  }
  if (const auto * spec = claim("turn_indicators", turn_indicators_shape_)) {
    if (spec->shape.size() != 2 || spec->shape[0] != 1) {
      throw std::runtime_error(
        "Model input 'turn_indicators' has shape " + shape_to_string(spec->shape) +
        "; expected [1, T]");
    }
    if (!turn_indicators_enabled_) {
      // The graph wants the tensor; this deployment says the network never reads it.
      // Bind a constant once instead of subscribing and filling it every tick.
      turn_indicators_constant_.assign(
        static_cast<size_t>(spec->shape[1]), static_cast<float>(TurnIndicatorsReport::DISABLE));
      RCLCPP_INFO(
        node_.get_logger(),
        "Model input 'turn_indicators' is disabled by context.turn_indicators.enabled: "
        "no subscription, constant DISABLE");
    }
  }

  create_subscriptions();
  return claimed;
}

void ContextInputProvider::create_subscriptions()
{
  const bool needs_map = !lanes_shape_.empty() || !route_lanes_shape_.empty() ||
                         !polygons_shape_.empty() || !line_strings_shape_.empty();
  const bool needs_route = !route_lanes_shape_.empty() || !goal_pose_shape_.empty();
  const bool needs_traffic = !lanes_shape_.empty() || !route_lanes_shape_.empty();

  if (!neighbor_shape_.empty() && !ignore_neighbors_) {
    sub_tracked_objects_ =
      std::make_unique<autoware_utils::InterProcessPollingSubscriber<TrackedObjects>>(
        &node_, "~/input/tracked_objects");
  }
  if (needs_traffic) {
    sub_traffic_signals_ = std::make_unique<autoware_utils::InterProcessPollingSubscriber<
      TrafficLightGroupArray, autoware_utils::polling_policy::All>>(
      &node_, "~/input/traffic_signals", rclcpp::QoS{10});
  }
  if (!turn_indicators_shape_.empty() && turn_indicators_enabled_) {
    sub_turn_indicators_ =
      std::make_unique<autoware_utils::InterProcessPollingSubscriber<TurnIndicatorsReport>>(
        &node_, "~/input/turn_indicators");
  }
  if (needs_route) {
    sub_route_ = std::make_unique<autoware_utils::InterProcessPollingSubscriber<
      LaneletRoute, autoware_utils::polling_policy::Newest>>(
      &node_, "~/input/route", rclcpp::QoS{1}.transient_local());
  }
  if (needs_map) {
    sub_map_ = node_.create_subscription<LaneletMapBin>(
      "~/input/vector_map", rclcpp::QoS{1}.transient_local(),
      std::bind(&ContextInputProvider::on_map, this, std::placeholders::_1));
  }
}

void ContextInputProvider::on_map(const LaneletMapBin::ConstSharedPtr map_msg)
{
  const auto lanelet_map_ptr =
    autoware::experimental::lanelet2_utils::from_autoware_map_msgs(*map_msg);
  lane_segment_context_ = std::make_unique<dp::preprocess::LaneSegmentContext>(
    lanelet_map_ptr, line_string_max_step_m_);
}

bool ContextInputProvider::collect(
  const EgoFrame & ego, const rclcpp::Time & now, TensorMap & inputs, std::string & error)
{
  // Traffic light states are accumulated regardless of individual tensor claims so that lane
  // tensors always carry the freshest signal encoding (mirrors DiffusionPlannerCore).
  if (sub_traffic_signals_) {
    const auto traffic_signals = sub_traffic_signals_->take_data();
    dp::preprocess::process_traffic_signals(
      traffic_signals, traffic_light_id_map_, now, traffic_light_msg_timeout_s_);
  }
  if (sub_route_) {
    const auto route = sub_route_->take_data();
    route_ptr_ = (!route_ptr_ || route) ? route : route_ptr_;
  }

  return collect_ego_tensors(ego, inputs, error) &&
         collect_neighbor_tensors(ego, inputs, error) &&
         collect_map_tensors(ego, inputs, error) && collect_route_tensors(ego, inputs, error) &&
         collect_turn_indicator_tensor(inputs, error);
}

bool ContextInputProvider::collect_ego_tensors(
  const EgoFrame & ego, TensorMap & inputs, std::string & error)
{
  if (!ego_agent_past_shape_.empty()) {
    const auto history_length = static_cast<size_t>(ego_agent_past_shape_[1]);
    ego_history_.push_back(ego.reference_odometry);
    while (ego_history_.size() > history_length) {
      ego_history_.pop_front();
    }
    const std::optional<rclcpp::Time> reference_time =
      use_time_interpolation_ ? std::make_optional(ego.stamp) : std::nullopt;
    inputs["ego_agent_past"] = Tensor::from_host(
      ego_agent_past_shape_,
      dp::preprocess::create_ego_agent_past(
        ego_history_, history_length, ego.map_to_ego, reference_time));
  }

  if (!ego_current_state_shape_.empty()) {
    if (!ego.acceleration) {
      error = "No acceleration received yet (required by 'ego_current_state')";
      return false;
    }
    inputs["ego_current_state"] = Tensor::from_host(
      ego_current_state_shape_,
      dp::preprocess::create_ego_current_state(
        ego.odometry, *ego.acceleration, static_cast<float>(wheel_base_)));
  }

  if (!ego_shape_shape_.empty()) {
    inputs["ego_shape"] = Tensor::from_host(
      ego_shape_shape_, {static_cast<float>(wheel_base_), static_cast<float>(vehicle_length_),
                         static_cast<float>(vehicle_width_)});
  }

  if (!static_objects_shape_.empty()) {
    inputs["static_objects"] = Tensor::from_host(
      static_objects_shape_,
      std::vector<float>(static_cast<size_t>(shape_num_elements(static_objects_shape_)), 0.0f));
  }

  return true;
}

bool ContextInputProvider::collect_neighbor_tensors(
  const EgoFrame & ego, TensorMap & inputs, std::string & error)
{
  if (neighbor_shape_.empty()) {
    return true;
  }

  TrackedObjects::ConstSharedPtr objects;
  if (!ignore_neighbors_) {
    objects = sub_tracked_objects_->take_data();
    if (!objects) {
      error = "No tracked objects received yet (required by 'neighbor_agents_past')";
      return false;
    }
  } else {
    objects = std::make_shared<TrackedObjects>();
  }

  const auto max_num_neighbors = static_cast<size_t>(neighbor_shape_[1]);
  agent_data_.update_histories(*objects, ignore_unknown_neighbors_);
  last_neighbor_histories_ =
    agent_data_.transformed_and_trimmed_histories(ego.map_to_ego, max_num_neighbors);
  inputs["neighbor_agents_past"] = Tensor::from_host(
    neighbor_shape_, dp::flatten_histories_to_vector(
                       last_neighbor_histories_, max_num_neighbors, dp::INPUT_T_WITH_CURRENT));
  return true;
}

bool ContextInputProvider::collect_map_tensors(
  const EgoFrame & ego, TensorMap & inputs, std::string & error)
{
  const bool needs_map = !lanes_shape_.empty() || !polygons_shape_.empty() ||
                         !line_strings_shape_.empty() || !route_lanes_shape_.empty();
  if (!needs_map) {
    return true;
  }
  if (!lane_segment_context_) {
    error = "Vector map not received yet";
    return false;
  }

  const auto center_x = static_cast<float>(ego.ego_to_map(0, 3));
  const auto center_y = static_cast<float>(ego.ego_to_map(1, 3));

  if (!lanes_shape_.empty()) {
    const int64_t num_segments = lanes_shape_[1];
    const std::vector<int64_t> segment_indices = lane_segment_context_->select_lane_segment_indices(
      ego.map_to_ego, center_x, center_y, num_segments);
    auto [lanes, lanes_speed_limit] = lane_segment_context_->create_tensor_data_from_indices(
      ego.map_to_ego, traffic_light_id_map_, segment_indices, num_segments);
    inputs["lanes"] = Tensor::from_host(lanes_shape_, std::move(lanes));
    if (!lanes_has_speed_limit_shape_.empty()) {
      // Same values as the speed limit tensor; the engine converts to bool by dtype.
      inputs["lanes_has_speed_limit"] =
        Tensor::from_host(lanes_has_speed_limit_shape_, lanes_speed_limit);
    }
    if (!lanes_speed_limit_shape_.empty()) {
      inputs["lanes_speed_limit"] =
        Tensor::from_host(lanes_speed_limit_shape_, std::move(lanes_speed_limit));
    }
  }

  if (!polygons_shape_.empty()) {
    inputs["polygons"] = Tensor::from_host(
      polygons_shape_,
      lane_segment_context_->create_polygon_tensor(ego.map_to_ego, center_x, center_y));
  }
  if (!line_strings_shape_.empty()) {
    inputs["line_strings"] = Tensor::from_host(
      line_strings_shape_,
      lane_segment_context_->create_line_string_tensor(ego.map_to_ego, center_x, center_y));
  }

  return true;
}

bool ContextInputProvider::collect_route_tensors(
  const EgoFrame & ego, TensorMap & inputs, std::string & error)
{
  if (route_lanes_shape_.empty() && goal_pose_shape_.empty()) {
    return true;
  }
  if (!route_ptr_) {
    error = "Route not received yet";
    return false;
  }

  if (!route_lanes_shape_.empty()) {
    if (!lane_segment_context_) {
      error = "Vector map not received yet";
      return false;
    }
    const double center_x = ego.ego_to_map(0, 3);
    const double center_y = ego.ego_to_map(1, 3);
    const double center_z = ego.ego_to_map(2, 3);
    const int64_t num_segments = route_lanes_shape_[1];
    const std::vector<int64_t> segment_indices =
      lane_segment_context_->select_route_segment_indices(
        *route_ptr_, center_x, center_y, center_z, num_segments);
    auto [route_lanes, route_speed_limit] = lane_segment_context_->create_tensor_data_from_indices(
      ego.map_to_ego, traffic_light_id_map_, segment_indices, num_segments);
    inputs["route_lanes"] = Tensor::from_host(route_lanes_shape_, std::move(route_lanes));
    if (!route_lanes_has_speed_limit_shape_.empty()) {
      inputs["route_lanes_has_speed_limit"] =
        Tensor::from_host(route_lanes_has_speed_limit_shape_, route_speed_limit);
    }
    if (!route_lanes_speed_limit_shape_.empty()) {
      inputs["route_lanes_speed_limit"] =
        Tensor::from_host(route_lanes_speed_limit_shape_, std::move(route_speed_limit));
    }
  }

  if (!goal_pose_shape_.empty()) {
    const Eigen::Matrix4d goal_pose_map =
      dp::utils::pose_to_matrix4d(route_ptr_->goal_pose);
    const Eigen::Matrix4d goal_pose_ego = ego.map_to_ego * goal_pose_map;
    const auto [cos_yaw, sin_yaw] =
      dp::utils::rotation_matrix_to_cos_sin(goal_pose_ego.block<3, 3>(0, 0));
    inputs["goal_pose"] = Tensor::from_host(
      goal_pose_shape_, {static_cast<float>(goal_pose_ego(0, 3)),
                         static_cast<float>(goal_pose_ego(1, 3)), cos_yaw, sin_yaw});
  }

  return true;
}

bool ContextInputProvider::collect_turn_indicator_tensor(TensorMap & inputs, std::string & error)
{
  if (turn_indicators_shape_.empty()) {
    return true;
  }

  if (!turn_indicators_enabled_) {
    inputs["turn_indicators"] = Tensor::from_host(turn_indicators_shape_, turn_indicators_constant_);
    return true;
  }

  const auto report = sub_turn_indicators_->take_data();
  if (!report) {
    error = "No turn indicator report received yet";
    return false;
  }

  const auto history_length = static_cast<size_t>(turn_indicators_shape_[1]);
  turn_indicators_history_.push_back(*report);
  while (turn_indicators_history_.size() > history_length) {
    turn_indicators_history_.pop_front();
  }

  // Copy from back to front; pad with the oldest value (mirrors DiffusionPlannerCore).
  std::vector<float> data(history_length, 0.0f);
  const auto last_index = static_cast<int64_t>(history_length) - 1;
  for (int64_t t = 0; t <= last_index; ++t) {
    const int64_t index =
      std::max(static_cast<int64_t>(turn_indicators_history_.size()) - 1 - t, int64_t{0});
    data[last_index - t] = static_cast<float>(turn_indicators_history_[index].report);
  }
  inputs["turn_indicators"] = Tensor::from_host(turn_indicators_shape_, std::move(data));
  return true;
}

}  // namespace autoware::tensorrt_e2e
