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
#include "autoware/diffusion_planner/inference/guidance/centerline_guidance.hpp"
#include "autoware/diffusion_planner/inference/guidance/start_guidance.hpp"
#include "autoware/diffusion_planner/inference/guidance/stop_guidance.hpp"
#include "autoware/diffusion_planner/inference/multi_step_inference.hpp"
#include "autoware/diffusion_planner/inference/single_step_inference.hpp"
#include "autoware/diffusion_planner/postprocessing/postprocessing_utils.hpp"
#include "autoware/diffusion_planner/preprocessing/preprocessing_utils.hpp"
#include "autoware/diffusion_planner/utils/utils.hpp"

#ifdef AUTOWARE_DIFFUSION_PLANNER_USE_ONNXRUNTIME
#include "autoware/diffusion_planner/inference/onnxruntime_inference.hpp"
#endif

#include <autoware_internal_planning_msgs/msg/candidate_trajectory.hpp>
#include <autoware_internal_planning_msgs/msg/generator_info.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::diffusion_planner
{
#ifdef AUTOWARE_DIFFUSION_PLANNER_USE_ONNXRUNTIME
namespace
{
bool is_onnxruntime_backend(const std::string & backend)
{
  return backend == "ort_cpu" || backend == "ort_cuda" || backend == "ort_tensorrt";
}

std::string onnxruntime_execution_provider_from_backend(const std::string & backend)
{
  if (backend == "ort_cpu") {
    return "cpu";
  }
  if (backend == "ort_cuda") {
    return "cuda";
  }
  if (backend == "ort_tensorrt") {
    return "tensorrt";
  }
  throw std::invalid_argument(
    "Unsupported model.backend '" + backend +
    "'. Expected 'tensorrt', 'ort_cpu', 'ort_cuda', or 'ort_tensorrt'.");
}
}  // namespace
#endif

DiffusionPlannerCore::DiffusionPlannerCore(
  const DiffusionPlannerParams & params, const VehicleInfo & vehicle_info)
: params_(params), vehicle_spec_(vehicle_info)
{
  // The no-signal HDP graph has 15 ONNX inputs (sampled trajectory plus 14
  // scene/context inputs). The compatibility `delay` entry is also retained
  // for the legacy waypoint path.
  sync_turn_indicator_managers();
}

void DiffusionPlannerCore::sync_turn_indicator_managers()
{
  const auto hold_duration = rclcpp::Duration::from_seconds(params_.turn_indicator_hold_duration);
  const auto on_confirmation_duration =
    rclcpp::Duration::from_seconds(params_.turn_indicator_on_confirmation_duration);
  const size_t desired = static_cast<size_t>(std::max<int>(params_.batch_size, 1));

  if (turn_indicator_managers_.size() > desired) {
    turn_indicator_managers_.erase(
      turn_indicator_managers_.begin() + static_cast<std::ptrdiff_t>(desired),
      turn_indicator_managers_.end());
  }
  while (turn_indicator_managers_.size() < desired) {
    turn_indicator_managers_.emplace_back(hold_duration, on_confirmation_duration);
  }
  for (auto & manager : turn_indicator_managers_) {
    manager.set_durations(hold_duration, on_confirmation_duration);
  }
}

void DiffusionPlannerCore::load_model()
{
  last_agent_poses_map_.clear();
  diffusion_planner_inference_.reset();
  is_velocity_representation_ = false;
  utils::check_weight_version(params_.args_path);
  // Validate the Python/export contract before allocating a TensorRT engine.  In
  // particular, ego_history_frames is an internal Python crop (21 in the current
  // checkpoint); the external ego_agent_past tensor remains the 31-frame contract.
  utils::validate_velocity_model_contract(params_.args_path);

  // Configure runtime-adaptive prediction dimensions from the model's args.json.
  // Full models predict ego + neighbors (num_prediction_agents == MAX_NUM_AGENTS); ego-only
  // models (predicted_neighbor_num == 0) predict only the ego trajectory.
  const auto prediction_dims =
    utils::load_prediction_dims(params_.args_path, MAX_NUM_AGENTS, OUTPUT_T);
  g_num_prediction_agents = prediction_dims.num_prediction_agents;
  g_sampled_trajectory_len = prediction_dims.sampled_trajectory_len;
  RCLCPP_INFO(
    rclcpp::get_logger("diffusion_planner"),
    "Prediction dimensions: num_prediction_agents=%ld, sampled_trajectory_len=%ld",
    static_cast<long>(g_num_prediction_agents), static_cast<long>(g_sampled_trajectory_len));

  // Velocity-latent (temporal ego) models integrate displacements into absolute waypoints inside
  // the ONNX graph and expect a pure-noise latent with no current-state prefix. The multi_step
  // path (external DPM solver, waypoint prefix constraint, waypoint-space start/stop/centerline
  // guidance, waypoint-statistics denormalization, no cumsum) assumes the legacy waypoint latent
  // and would silently emit invalid trajectories. Fail fast on that mismatch instead.
  const bool is_velocity_latent = g_sampled_trajectory_len <= OUTPUT_T;
  if (is_velocity_latent && params_.model_type != "single_step") {
    throw std::runtime_error(
      "Velocity-representation (temporal ego) weights require model.type='single_step'; got '" +
      params_.model_type +
      "'. The multi_step path assumes the legacy waypoint latent and is "
      "incompatible with this model.");
  }
  is_velocity_representation_ = is_velocity_latent;

  if (is_velocity_latent) {
    // The Python converter builds all temporal inputs on a 10 Hz grid and the
    // HDP output is a fixed 0.1 s displacement sequence.  Running the planner
    // at another rate would change the meaning of the six-step neighbor window,
    // the 21-frame ego window, and the published trajectory timestamps.
    constexpr double expected_frequency_hz = 1.0 / constants::PREDICTION_TIME_STEP_S;
    if (
      !std::isfinite(params_.planning_frequency_hz) ||
      std::abs(params_.planning_frequency_hz - expected_frequency_hz) > 1e-6) {
      throw std::runtime_error(
        "Velocity-representation HDP requires planning_frequency_hz=" +
        std::to_string(expected_frequency_hz) + " to match the Python 10 Hz data grid; got " +
        std::to_string(params_.planning_frequency_hz) + ".");
    }
    if (!params_.use_time_interpolation) {
      throw std::runtime_error(
        "Velocity-representation HDP requires use_time_interpolation=true so ego_agent_past "
        "is sampled on the Python 0.1 s grid.");
    }
  }

  observation_normalization_ = utils::load_observation_normalization(params_.args_path);
  state_normalization_ = utils::load_state_normalization(params_.args_path);

  // Initialize guidance modules
  StartGuidanceConfig start_guidance_config;
  start_guidance_config.reference_distance_m =
    static_cast<float>(params_.start_guidance_reference_distance_m);
  start_guidance_config.max_scale = static_cast<float>(params_.start_guidance_max_scale);
  start_guidance_config.x_mean = static_cast<float>(state_normalization_.first.at(0));
  start_guidance_config.x_std = static_cast<float>(state_normalization_.second.at(0));
  start_guidance_config.y_mean = static_cast<float>(state_normalization_.first.at(1));
  start_guidance_config.y_std = static_cast<float>(state_normalization_.second.at(1));
  start_guidance_ = std::make_shared<StartGuidance>(start_guidance_config);
  start_guidance_->set_enabled(start_guidance_enabled_);

  StopGuidanceConfig stop_guidance_config;
  stop_guidance_config.stop_acceleration_mps2 =
    static_cast<float>(params_.stop_guidance_stop_acceleration_mps2);
  stop_guidance_config.x_mean = static_cast<float>(state_normalization_.first.at(0));
  stop_guidance_config.x_std = static_cast<float>(state_normalization_.second.at(0));
  stop_guidance_config.y_mean = static_cast<float>(state_normalization_.first.at(1));
  stop_guidance_config.y_std = static_cast<float>(state_normalization_.second.at(1));
  stop_guidance_ = std::make_shared<StopGuidance>(stop_guidance_config);
  stop_guidance_->set_enabled(stop_guidance_enabled_);

  CenterlineGuidanceConfig centerline_guidance_config;
  centerline_guidance_config.start_time_s =
    static_cast<float>(params_.centerline_guidance_start_time_s);
  centerline_guidance_config.x_mean = static_cast<float>(state_normalization_.first.at(0));
  centerline_guidance_config.x_std = static_cast<float>(state_normalization_.second.at(0));
  centerline_guidance_config.y_mean = static_cast<float>(state_normalization_.first.at(1));
  centerline_guidance_config.y_std = static_cast<float>(state_normalization_.second.at(1));
  centerline_guidance_ = std::make_shared<CenterlineGuidance>(centerline_guidance_config);
  centerline_guidance_->set_enabled(centerline_guidance_enabled_);

  std::unordered_map<std::string, std::shared_ptr<Guidance>> guidances{
    {"start", start_guidance_}, {"stop", stop_guidance_}, {"centerline", centerline_guidance_}};
  if (params_.backend == "tensorrt" && params_.model_type == "single_step") {
    diffusion_planner_inference_ = std::make_unique<SingleStepInference>(
      params_.single_step_model_path, params_.plugins_path, params_.batch_size,
      params_.trt_precision, params_.use_cuda_graph);
  } else if (params_.backend == "tensorrt" && params_.model_type == "multi_step") {
    diffusion_planner_inference_ = std::make_unique<MultiStepInference>(
      params_.encoder_model_path, params_.decoder_model_path, params_.turn_indicator_model_path,
      params_.plugins_path, params_.batch_size, params_.trt_precision, params_.use_cuda_graph,
      params_.dpm_solver_steps, std::move(guidances));
#ifdef AUTOWARE_DIFFUSION_PLANNER_USE_ONNXRUNTIME
  } else if (is_onnxruntime_backend(params_.backend) && params_.model_type == "single_step") {
    diffusion_planner_inference_ = std::make_unique<OnnxruntimeSingleStepInference>(
      params_.single_step_model_path, onnxruntime_execution_provider_from_backend(params_.backend),
      params_.plugins_path, params_.batch_size);
  } else if (is_onnxruntime_backend(params_.backend) && params_.model_type == "multi_step") {
    diffusion_planner_inference_ = std::make_unique<OnnxruntimeMultiStepInference>(
      params_.encoder_model_path, params_.decoder_model_path, params_.turn_indicator_model_path,
      onnxruntime_execution_provider_from_backend(params_.backend), params_.plugins_path,
      params_.batch_size, params_.dpm_solver_steps, std::move(guidances));
#endif
  } else {
    if (params_.backend != "tensorrt") {
      throw std::invalid_argument(
        "Unsupported model.backend '" + params_.backend +
        "'. ONNX Runtime support is not available in this build.");
    }
    throw std::invalid_argument(
      "Unsupported model.type '" + params_.model_type +
      "'. Expected 'single_step' or 'multi_step'.");
  }
}

void DiffusionPlannerCore::update_params(const DiffusionPlannerParams & params)
{
  params_ = params;
  sync_turn_indicator_managers();
  if (start_guidance_) {
    StartGuidanceConfig start_guidance_config;
    start_guidance_config.reference_distance_m =
      static_cast<float>(params_.start_guidance_reference_distance_m);
    start_guidance_config.max_scale = static_cast<float>(params_.start_guidance_max_scale);
    start_guidance_config.x_mean = static_cast<float>(state_normalization_.first.at(0));
    start_guidance_config.x_std = static_cast<float>(state_normalization_.second.at(0));
    start_guidance_config.y_mean = static_cast<float>(state_normalization_.first.at(1));
    start_guidance_config.y_std = static_cast<float>(state_normalization_.second.at(1));
    start_guidance_->set_config(start_guidance_config);
    start_guidance_->set_enabled(start_guidance_enabled_);
  }
  if (stop_guidance_) {
    StopGuidanceConfig stop_guidance_config;
    stop_guidance_config.stop_acceleration_mps2 =
      static_cast<float>(params_.stop_guidance_stop_acceleration_mps2);
    stop_guidance_config.x_mean = static_cast<float>(state_normalization_.first.at(0));
    stop_guidance_config.x_std = static_cast<float>(state_normalization_.second.at(0));
    stop_guidance_config.y_mean = static_cast<float>(state_normalization_.first.at(1));
    stop_guidance_config.y_std = static_cast<float>(state_normalization_.second.at(1));
    stop_guidance_->set_config(stop_guidance_config);
    stop_guidance_->set_enabled(stop_guidance_enabled_);
  }
  if (centerline_guidance_) {
    CenterlineGuidanceConfig centerline_guidance_config;
    centerline_guidance_config.start_time_s =
      static_cast<float>(params_.centerline_guidance_start_time_s);
    centerline_guidance_config.x_mean = static_cast<float>(state_normalization_.first.at(0));
    centerline_guidance_config.x_std = static_cast<float>(state_normalization_.second.at(0));
    centerline_guidance_config.y_mean = static_cast<float>(state_normalization_.first.at(1));
    centerline_guidance_config.y_std = static_cast<float>(state_normalization_.second.at(1));
    centerline_guidance_->set_config(centerline_guidance_config);
    centerline_guidance_->set_enabled(centerline_guidance_enabled_);
  }
}

void DiffusionPlannerCore::set_start_guidance_enabled(const bool enabled)
{
  start_guidance_enabled_ = enabled;
  if (start_guidance_) {
    start_guidance_->set_enabled(enabled);
  }
}

void DiffusionPlannerCore::set_stop_guidance_enabled(const bool enabled)
{
  stop_guidance_enabled_ = enabled;
  if (stop_guidance_) {
    stop_guidance_->set_enabled(enabled);
  }
}

void DiffusionPlannerCore::set_centerline_guidance_enabled(const bool enabled)
{
  centerline_guidance_enabled_ = enabled;
  if (centerline_guidance_) {
    centerline_guidance_->set_enabled(enabled);
  }
}

void DiffusionPlannerCore::set_map(
  const std::shared_ptr<const lanelet::LaneletMap> & lanelet_map_ptr)
{
  lane_segment_context_ = std::make_unique<preprocess::LaneSegmentContext>(
    lanelet_map_ptr, params_.line_string_max_step_m);
}

std::optional<FrameContext> DiffusionPlannerCore::create_frame_context(
  const std::shared_ptr<const Odometry> & ego_kinematic_state,
  const std::shared_ptr<const AccelWithCovarianceStamped> & ego_acceleration,
  const std::shared_ptr<const TrackedObjects> & objects,
  const std::vector<std::shared_ptr<const autoware_perception_msgs::msg::TrafficLightGroupArray>> &
    traffic_signals,
  [[maybe_unused]] const std::shared_ptr<const TurnIndicatorsReport> & turn_indicators,
  const LaneletRoute::ConstSharedPtr & route_ptr, const rclcpp::Time & current_time)
{
  route_ptr_ = (!route_ptr_ || route_ptr) ? route_ptr : route_ptr_;

  TrackedObjects empty_object_list;
  auto effective_objects = objects;

  if (params_.ignore_neighbors) {
    effective_objects = std::make_shared<TrackedObjects>(empty_object_list);
  }

  if (!effective_objects || !ego_kinematic_state || !ego_acceleration) {
    return std::nullopt;
  }

  if (!route_ptr_) {
    return std::nullopt;
  }

  // The converter drops frames when any required temporal input is older than 500 ms.  The
  // runtime subscribers use the Latest polling policy, so a non-null pointer can otherwise be an
  // arbitrarily old message after a producer stalls.  Do not run the model on that stale scene;
  // clear all temporal state so the next fresh frame starts a clean sequence.  The route is
  // intentionally excluded because it is a persistent planning state rather than a frame-rate
  // sensor input.
  const auto input_age_s = [&current_time](const rclcpp::Time & message_time) {
    return (current_time - message_time).seconds();
  };
  const double max_input_age_s = params_.ego_history_reset_gap_s;
  const bool stale_objects =
    !params_.ignore_neighbors && input_age_s(rclcpp::Time(objects->header.stamp)) > max_input_age_s;
  const bool stale_temporal_input =
    input_age_s(rclcpp::Time(ego_kinematic_state->header.stamp)) > max_input_age_s ||
    input_age_s(rclcpp::Time(ego_acceleration->header.stamp)) > max_input_age_s || stale_objects;
  if (stale_temporal_input) {
    ego_history_.clear();
    agent_data_.clear_histories();
    last_agent_poses_map_.clear();
    return std::nullopt;
  }

  Odometry kinematic_state = *ego_kinematic_state;
  if (params_.shift_x) {
    kinematic_state.pose.pose =
      utils::shift_x(kinematic_state.pose.pose, vehicle_spec_.base_link_to_center);
  }

  // Get transforms
  const geometry_msgs::msg::Pose & pose_base_link = kinematic_state.pose.pose;
  const Eigen::Matrix4d ego_to_map_transform = utils::pose_to_matrix4d(pose_base_link);
  const Eigen::Matrix4d map_to_ego_transform = utils::inverse(ego_to_map_transform);

  // All temporal inputs are trained on a fixed 0.1 s grid.  A short planner
  // overrun is recoverable by ego-pose interpolation, but a long timestamp
  // discontinuity cannot be reconstructed and must not carry stale state
  // across the gap.
  const rclcpp::Time current_ego_time(ego_kinematic_state->header.stamp);
  if (!ego_history_.empty()) {
    const rclcpp::Time previous_ego_time(ego_history_.back().header.stamp);
    const double ego_history_gap_s = (current_ego_time - previous_ego_time).seconds();
    if (ego_history_gap_s < 0.0 || ego_history_gap_s > params_.ego_history_reset_gap_s) {
      RCLCPP_WARN(
        rclcpp::get_logger("diffusion_planner"),
        "Resetting temporal histories after ego timestamp gap of %.3f s", ego_history_gap_s);
      ego_history_.clear();
      agent_data_.clear_histories();
      last_agent_poses_map_.clear();
    }
  }

  // Update ego history
  ego_history_.push_back(kinematic_state);
  if (ego_history_.size() > EGO_HISTORY_BUFFER_SIZE) {
    ego_history_.pop_front();
  }

  // Update neighbor agent data
  // The HDP training converter keeps non-UNKNOWN Polygon objects in the
  // neighbor context.  Match that contract only for HDP; legacy waypoint
  // deployments retain the historical Polygon filtering behavior.
  agent_data_.update_histories(*effective_objects, is_velocity_representation_);
  const auto processed_neighbor_histories =
    agent_data_.transformed_and_trimmed_histories(map_to_ego_transform, NEIGHBOR_SHAPE[1]);

  // Update traffic light map
  const auto & traffic_light_msg_timeout_s = params_.traffic_light_group_msg_timeout_seconds;
  preprocess::process_traffic_signals(
    traffic_signals, traffic_light_id_map_, current_time, traffic_light_msg_timeout_s);

  // Create frame context
  const rclcpp::Time frame_time(ego_kinematic_state->header.stamp);
  const FrameContext frame_context{
    *ego_kinematic_state, *ego_acceleration, ego_to_map_transform, processed_neighbor_histories,
    frame_time};

  return frame_context;
}

InputDataMap DiffusionPlannerCore::create_input_data(const FrameContext & frame_context)
{
  InputDataMap input_data_map;

  if (stop_guidance_) {
    const auto & linear = frame_context.ego_kinematic_state.twist.twist.linear;
    stop_guidance_->set_current_speed_mps(static_cast<float>(std::hypot(linear.x, linear.y)));
  }

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
    const int64_t copy_steps = std::clamp<int64_t>(params_.delay_step, 0, OUTPUT_T / 2);
    // The warm-start below seeds the sampled-trajectory prefix with the previous frame's poses
    // encoded in waypoint-normalization statistics ((x-10)/20, ...). That is only valid for the
    // legacy waypoint latent. Velocity-latent (temporal ego) models expect a pure-noise latent
    // here (std=0.5 displacement space), so seeding it injects out-of-distribution values into
    // step 0 every cycle. Restrict the warm-start to legacy models.
    const bool is_legacy_waypoint_latent = g_sampled_trajectory_len > OUTPUT_T;
    const bool has_previous_output = !last_agent_poses_map_.empty() && is_legacy_waypoint_latent;

    for (int64_t b = 0; b < params_.batch_size; b++) {
      std::vector<float> sampled_trajectories =
        preprocess::create_sampled_trajectories(params_.temperature_list[b]);

      if (has_previous_output) {
        constexpr int64_t agent_idx = 0;
        delay_step = copy_steps;
        for (int64_t t = 0; t <= copy_steps; ++t) {
          const size_t dst_base = agent_idx * g_sampled_trajectory_len * POSE_DIM + (t)*POSE_DIM;
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
    if (centerline_guidance_) {
      centerline_guidance_->set_route_lanes(input_data_map["route_lanes"]);
    }
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

  // control delay
  {
    const std::vector<float> single_delay = {static_cast<float>(delay_step)};
    input_data_map["delay"] = utils::replicate_for_batch(single_delay, params_.batch_size);
  }

  return input_data_map;
}

InferenceResult DiffusionPlannerCore::run_inference(const InputDataMap & input_data_map)
{
  if (!diffusion_planner_inference_) {
    return tl::unexpected(std::string{"Model not loaded"});
  }
  return diffusion_planner_inference_->infer(input_data_map);
}

PlannerOutput DiffusionPlannerCore::create_planner_output(
  const InferenceOutput & inference_output, const FrameContext & frame_context,
  const rclcpp::Time & timestamp, const UUID & generator_uuid)
{
  const auto & [raw_predictions, turn_indicator_logit] = inference_output.outputs;
  const std::vector<float> denormalized_predictions =
    inference_output.is_denormalized
      ? raw_predictions
      : postprocess::denormalize_prediction(raw_predictions, state_normalization_);
  std::vector<float> denormalized_denoising_predictions;
  if (!inference_output.denoising_predictions.empty()) {
    denormalized_denoising_predictions =
      inference_output.is_denormalized
        ? inference_output.denoising_predictions
        : postprocess::denormalize_prediction(
            inference_output.denoising_predictions, state_normalization_, true);
  }

  const auto agent_poses =
    postprocess::parse_predictions(denormalized_predictions, frame_context.ego_to_map_transform);
  last_agent_poses_map_ = agent_poses;

  // Force-stop latches the trajectory once the smoothed velocity first drops below
  // stopping_threshold: the remaining horizon is frozen at that pose with zero speed.  It only
  // applies while the ego is actually moving, so a standing start is never latched.
  const bool ego_is_moving =
    frame_context.ego_kinematic_state.twist.twist.linear.x > std::numeric_limits<double>::epsilon();
  const bool enable_force_stop = params_.enable_force_stop && ego_is_moving;

  PlannerOutput output;
  output.denoising_steps = postprocess::create_denoising_steps_message(
    denormalized_denoising_predictions, inference_output.denoising_timesteps);

  // Trajectory and CandidateTrajectories
  // Legacy waypoint models need the configured moving average because their
  // velocity is reconstructed from independently predicted positions.  HDP's
  // cumulative waypoint output is generated from per-step displacement actions;
  // differencing it recovers those actions, so applying the legacy moving
  // average would alter the learned velocity profile and add preview delay.
  const int64_t trajectory_velocity_smoothing_window =
    is_velocity_representation_ ? 1 : params_.velocity_smoothing_window;
  // parse_predictions() is expressed around the shifted reference pose when shift_x is enabled.
  // Use that same reference as the base for the first displacement; passing the unshifted
  // base_link position would add the base_link-to-center offset to the first speed calculation.
  const geometry_msgs::msg::Pose trajectory_base_pose =
    params_.shift_x
      ? utils::shift_x(
          frame_context.ego_kinematic_state.pose.pose, vehicle_spec_.base_link_to_center)
      : frame_context.ego_kinematic_state.pose.pose;
  const auto expected_turn_indicator_logit_size =
    static_cast<size_t>(std::max(params_.batch_size, 1)) *
    static_cast<size_t>(TURN_INDICATOR_OUTPUT_DIM);
  const bool turn_indicator_output_valid =
    turn_indicator_logit.size() == expected_turn_indicator_logit_size;
  if (!turn_indicator_output_valid) {
    throw std::runtime_error(
      "Turn-indicator output has " + std::to_string(turn_indicator_logit.size()) +
      " values; expected " + std::to_string(expected_turn_indicator_logit_size) +
      " for the final three-class model.");
  }
  // The manager degrades a non-finite logit to DISABLE rather than aborting the planning
  // cycle for an auxiliary output, so surface it here: an untraceable blinker dropout is
  // the failure mode this guards against.
  if (!std::all_of(turn_indicator_logit.begin(), turn_indicator_logit.end(), [](const float v) {
        return std::isfinite(v);
      })) {
    RCLCPP_WARN_THROTTLE(
      rclcpp::get_logger("diffusion_planner"), throttle_clock_, 5000,
      "Turn-indicator logits contain non-finite values; forcing the command to DISABLE.");
  }
  for (int i = 0; i < params_.batch_size; i++) {
    auto trajectory = postprocess::create_ego_trajectory(
      agent_poses, timestamp, trajectory_base_pose.position, i,
      trajectory_velocity_smoothing_window, enable_force_stop, params_.stopping_threshold);

    if (params_.shift_x) {
      for (auto & point : trajectory.points) {
        point.pose = utils::shift_x(point.pose, -vehicle_spec_.base_link_to_center);
      }
    }

    if (is_velocity_representation_ && params_.prepend_current_ego_state) {
      const auto & twist = frame_context.ego_kinematic_state.twist.twist;
      const auto & acceleration = frame_context.ego_acceleration.accel.accel;
      postprocess::prepend_current_ego_state(
        trajectory, frame_context.ego_kinematic_state.pose.pose, twist.linear.x, twist.linear.y,
        acceleration.linear.x, twist.angular.z);
    }

    if (i == 0) {
      // Use the first trajectory as the main output trajectory
      output.trajectory = trajectory;
    }

    // TurnIndicatorsCommand
    const std::vector<float> single_turn_indicator_logit(
      turn_indicator_logit.begin() + TURN_INDICATOR_OUTPUT_DIM * i,
      turn_indicator_logit.begin() + TURN_INDICATOR_OUTPUT_DIM * (i + 1));
    const TurnIndicatorsCommand turn_indicators_command =
      turn_indicator_managers_.at(i).evaluate(single_turn_indicator_logit, timestamp);

    if (i == 0) {
      // Publish the first trajectory's command on the standalone turn indicator topic.
      output.turn_indicators_command = turn_indicators_command;
      // Expose the raw pre-debounce logits so the published command can be audited
      // against the model's per-cycle observation.
      output.turn_indicator_logit.data = single_turn_indicator_logit;
    }

    const auto candidate_trajectory = autoware_internal_planning_msgs::build<
                                        autoware_internal_planning_msgs::msg::CandidateTrajectory>()
                                        .header(trajectory.header)
                                        .generator_id(generator_uuid)
                                        .points(trajectory.points)
                                        .turn_indicators_command(turn_indicators_command);

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

  output.guidance_triggered = inference_output.guidance_triggered;

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
