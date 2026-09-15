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

#include "autoware/ml_planner/ml_planner_core.hpp"

#include "autoware/ml_planner/constants.hpp"
#include "autoware/ml_planner/dimensions.hpp"
#include "autoware/ml_planner/inference/single_step_inference.hpp"
#include "autoware/ml_planner/postprocessing/postprocessing_utils.hpp"
#include "autoware/ml_planner/preprocessing/items/initial_noise.hpp"
#include "autoware/ml_planner/utils/utils.hpp"

#ifdef AUTOWARE_ML_PLANNER_USE_ONNXRUNTIME
#include "autoware/ml_planner/inference/onnxruntime_inference.hpp"
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

namespace autoware::ml_planner
{
#ifdef AUTOWARE_ML_PLANNER_USE_ONNXRUNTIME
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

MLPlannerCore::MLPlannerCore(const MLPlannerParams & params, const VehicleInfo & vehicle_info)
: params_(params), vehicle_spec_(vehicle_info)
{
  if (
    params_.batch_size < 1 || params_.batch_size > 2 ||
    params_.noise_scale_list.size() != static_cast<size_t>(params_.batch_size)) {
    throw std::invalid_argument(
      "batch_size must be 1 or 2 and noise_scale must contain exactly batch_size values");
  }
#ifdef AUTOWARE_ML_PLANNER_USE_ACADOS
  if (params_.trajectory_optimization.enable) {
    trajectory_optimizer_ = std::make_unique<optimization::TrajectoryOptimizer>(
      params_.trajectory_optimization, vehicle_info, static_cast<size_t>(params_.batch_size));
  }
#endif
  if (params_.road_border_avoidance.enable) {
    road_border_avoidance_ = std::make_unique<postprocess::RoadBorderAvoidance>(
      params_.road_border_avoidance, vehicle_info);
  }
}

void MLPlannerCore::load_model()
{
  ml_planner_inference_.reset();
  if (params_.backend == "tensorrt") {
    ml_planner_inference_ = std::make_unique<SingleStepInference>(
      params_.model_path, params_.plugins_path, params_.batch_size, params_.trt_precision,
      params_.use_cuda_graph);
#ifdef AUTOWARE_ML_PLANNER_USE_ONNXRUNTIME
  } else if (is_onnxruntime_backend(params_.backend)) {
    ml_planner_inference_ = std::make_unique<OnnxruntimeSingleStepInference>(
      params_.model_path, onnxruntime_execution_provider_from_backend(params_.backend),
      params_.plugins_path);
#endif
  } else {
    if (params_.backend != "tensorrt") {
      throw std::invalid_argument(
        "Unsupported model.backend '" + params_.backend +
        "'. ONNX Runtime support is not available in this build.");
    }
    throw std::invalid_argument("Unsupported model.backend '" + params_.backend + "'.");
  }
}

void MLPlannerCore::update_params(const MLPlannerParams & params)
{
  if (
    params.batch_size < 1 || params.batch_size > 2 ||
    params.noise_scale_list.size() != static_cast<size_t>(params.batch_size)) {
    throw std::invalid_argument(
      "batch_size must be 1 or 2 and noise_scale must contain exactly batch_size values");
  }
  params_ = params;
}

void MLPlannerCore::set_map(const std::shared_ptr<const lanelet::LaneletMap> & lanelet_map_ptr)
{
  lane_segment_context_ = std::make_unique<preprocess::LaneSegmentContext>(
    lanelet_map_ptr, params_.line_string_max_step_m);
  if (road_border_avoidance_ && lanelet_map_ptr) {
    road_border_avoidance_->set_map(*lanelet_map_ptr);
  }
}

MLPlannerCore::BufferUpdateResult MLPlannerCore::update_buffer(
  const std::vector<std::shared_ptr<const Odometry>> & ego_kinematic_states,
  const std::vector<std::shared_ptr<const TrackedObjects>> & objects,
  const std::vector<std::shared_ptr<const autoware_perception_msgs::msg::TrafficLightGroupArray>> &
    traffic_signals,
  const std::vector<std::shared_ptr<const TurnIndicatorsReport>> & turn_indicators,
  const LaneletRoute::ConstSharedPtr & route_ptr)
{
  if (route_ptr) {
    route_ptr_ = route_ptr;
  }
  for (const auto & msg : ego_kinematic_states) {
    if (msg) {
      ego_history_.push_back(*msg);
    }
  }
  for (const auto & msg : turn_indicators) {
    if (msg) {
      turn_indicators_history_.push_back(*msg);
    }
  }
  for (const auto & msg : objects) {
    if (msg) {
      objects_history_.push_back(*msg);
    }
  }
  for (const auto & msg : traffic_signals) {
    if (msg) {
      traffic_signals_history_.push_back(*msg);
    }
  }

  std::vector<std::string> missing_inputs;
  if (ego_history_.empty()) {
    missing_inputs.emplace_back("ego kinematic state");
  }
  if (objects_history_.empty()) {
    missing_inputs.emplace_back("tracked objects");
  }
  if (!route_ptr_) {
    missing_inputs.emplace_back("route");
  }
  if (turn_indicators_history_.empty()) {
    missing_inputs.emplace_back("turn indicators");
  }
  if (!missing_inputs.empty()) {
    std::string error{"Missing required input: "};
    for (size_t index = 0; index < missing_inputs.size(); ++index) {
      if (index > 0) {
        error += ", ";
      }
      error += missing_inputs[index];
    }
    return tl::unexpected(std::move(error));
  }

  return preprocess::FrameInputs{
    frame_time(),
    preprocess::MessageView<nav_msgs::msg::Odometry>{ego_history_.msgs()},
    preprocess::MessageView<autoware_vehicle_msgs::msg::TurnIndicatorsReport>{
      turn_indicators_history_.msgs()},
    preprocess::MessageView<autoware_perception_msgs::msg::TrackedObjects>{objects_history_.msgs()},
    preprocess::MessageView<autoware_perception_msgs::msg::TrafficLightGroupArray>{
      traffic_signals_history_.msgs()},
    *route_ptr_};
}

preprocess::TensorMapResult MLPlannerCore::create_input_data(
  const preprocess::FrameInputs & frame_inputs)
{
  selected_agents_.clear();
  if (!lane_segment_context_) {
    return tl::unexpected(std::string{"Map context is unavailable"});
  }

  const preprocess::InputBuilderParams builder_params{
    params_.traffic_light_group_msg_timeout_seconds};
  auto single_input_result = preprocess::create_input_data_map(
    frame_inputs, *lane_segment_context_, vehicle_spec_, builder_params);
  if (!single_input_result) {
    return tl::unexpected(single_input_result.error());
  }
  auto builder_output = std::move(single_input_result.value());
  selected_agents_ = std::move(builder_output.selected_agents);
  TensorMap single_input_data_map = std::move(builder_output.tensors);

  // Replicate for batch
  TensorMap input_data_map;
  for (auto & [key, value] : single_input_data_map) {
    input_data_map[key] = utils::replicate_for_batch(value, params_.batch_size);
  }

  // Initial Gaussian noise for the sampler embedded in the ONNX graph.
  std::vector<size_t> sampled_shape;
  std::transform(
    INITIAL_NOISE_SHAPE.begin(), INITIAL_NOISE_SHAPE.end(), std::back_inserter(sampled_shape),
    [](const int64_t dim) { return static_cast<size_t>(dim); });
  sampled_shape.front() = static_cast<size_t>(params_.batch_size);
  xt::xarray<float> sampled_tensor = xt::xarray<float>::from_shape(sampled_shape);
  const size_t single_sample_size = sampled_tensor.size() / sampled_shape.front();
  for (int64_t b = 0; b < params_.batch_size; b++) {
    const xt::xarray<float> initial_noise =
      preprocess::create_initial_noise(params_.noise_scale_list[b]);
    std::copy(
      initial_noise.begin(), initial_noise.end(),
      sampled_tensor.begin() + static_cast<std::ptrdiff_t>(b * single_sample_size));
  }
  input_data_map["initial_noise"] = std::move(sampled_tensor);

  return input_data_map;
}

rclcpp::Time MLPlannerCore::frame_time() const
{
  return rclcpp::Time(ego_history_.back().header.stamp);
}

InferenceResult MLPlannerCore::run_inference(const TensorMap & input_data_map)
{
  if (!ml_planner_inference_) {
    return tl::unexpected(std::string{"Model not loaded"});
  }
  return ml_planner_inference_->infer(input_data_map);
}

PlannerOutput MLPlannerCore::create_planner_output(
  const InferenceOutput & inference_output, const rclcpp::Time & timestamp,
  const UUID & generator_uuid, const double current_steering_angle_rad)
{
  // Derive the frame state from the raw message buffers
  const Odometry & kinematic_state = ego_history_.back();
  const Eigen::Matrix4d ego_to_map_transform = utils::pose_to_matrix4d(kinematic_state.pose.pose);

  const auto & raw_predictions = inference_output.trajectory;
  const auto & turn_indicator_logits = inference_output.turn_indicator_logits;
  const auto expected_logits_size =
    static_cast<size_t>(params_.batch_size * TURN_INDICATOR_OUTPUT_DIM);
  if (turn_indicator_logits.size() != expected_logits_size) {
    throw std::runtime_error(
      "turn_indicator_logits size mismatch: expected " + std::to_string(expected_logits_size) +
      ", got " + std::to_string(turn_indicator_logits.size()));
  }
  const std::vector<float> denormalized_predictions =
    inference_output.is_denormalized ? raw_predictions
                                     : postprocess::denormalize_prediction(raw_predictions);

  const auto agent_poses =
    postprocess::parse_predictions(denormalized_predictions, ego_to_map_transform);

  PlannerOutput output;
  // Trajectory and CandidateTrajectories
  for (int i = 0; i < params_.batch_size; i++) {
    auto trajectory = postprocess::create_ego_trajectory(agent_poses, timestamp, i);

    if (i == 0) {
      // Keep the untouched model output for the debug topics.
      output.raw_trajectory = trajectory;
    }

    if (road_border_avoidance_) {
      auto avoidance_result = road_border_avoidance_->adjust(trajectory, kinematic_state.pose.pose);
      if (i == 0) {
        output.avoidance_debug.active = true;
        output.avoidance_debug.shifted_points =
          static_cast<int>(avoidance_result.num_shifted_points);
        output.avoidance_debug.unresolved_points =
          static_cast<int>(avoidance_result.num_unresolved_points);
        output.avoidance_adjusted_trajectory = avoidance_result.trajectory;
      }
      trajectory = std::move(avoidance_result.trajectory);
    }

#ifdef AUTOWARE_ML_PLANNER_USE_ACADOS
    if (trajectory_optimizer_) {
      auto optimization_result = trajectory_optimizer_->optimize(
        trajectory, kinematic_state, current_steering_angle_rad, static_cast<size_t>(i));
      if (i == 0) {
        output.optimization_debug.attempted = true;
        output.optimization_debug.optimized = optimization_result.optimized;
        output.optimization_debug.solver_status = optimization_result.solver_status;
        output.optimization_debug.solve_time_ms = optimization_result.solve_time_ms;
      }
      trajectory = std::move(optimization_result.trajectory);
    }
#else
    (void)current_steering_angle_rad;
#endif

    if (params_.stop_point_fixing.enable) {
      if (i == 0) {
        output.pre_stop_fixing_trajectory = trajectory;
      }
      postprocess::fix_stop_points(trajectory, params_.stop_point_fixing);
    }

    if (i == 0) {
      // Use the first trajectory as the main output trajectory
      output.trajectory = trajectory;
    }

    // TurnIndicatorsCommand
    const auto logits_begin =
      turn_indicator_logits.begin() + static_cast<std::ptrdiff_t>(i * TURN_INDICATOR_OUTPUT_DIM);
    const std::vector<float> batch_logits(logits_begin, logits_begin + TURN_INDICATOR_OUTPUT_DIM);
    if (!std::all_of(batch_logits.begin(), batch_logits.end(), [](const float value) {
          return std::isfinite(value);
        })) {
      throw std::runtime_error("turn_indicator_logits contains a non-finite value");
    }
    const auto selected_class = static_cast<uint8_t>(std::distance(
      batch_logits.begin(), std::max_element(batch_logits.begin(), batch_logits.end())));
    TurnIndicatorsCommand turn_indicators_command;
    turn_indicators_command.stamp = timestamp;
    if (selected_class == TURN_INDICATOR_OUTPUT_DISABLE) {
      turn_indicators_command.command = TurnIndicatorsCommand::DISABLE;
    } else if (selected_class == TURN_INDICATOR_OUTPUT_ENABLE_LEFT) {
      turn_indicators_command.command = TurnIndicatorsCommand::ENABLE_LEFT;
    } else {
      turn_indicators_command.command = TurnIndicatorsCommand::ENABLE_RIGHT;
    }

    if (i == 0) {
      // Publish the first trajectory's command on the standalone turn indicator topic.
      output.turn_indicators_command = turn_indicators_command;
    }

    autoware_internal_planning_msgs::msg::CandidateTrajectory candidate_trajectory;
    candidate_trajectory.header = trajectory.header;
    candidate_trajectory.generator_id = generator_uuid;
    candidate_trajectory.points = trajectory.points;
    candidate_trajectory.turn_indicators_command = turn_indicators_command;

    std_msgs::msg::String generator_name_msg;
    generator_name_msg.data = std::string("MLPlanner_batch_") + std::to_string(i);

    autoware_internal_planning_msgs::msg::GeneratorInfo generator_info;
    generator_info.generator_id = generator_uuid;
    generator_info.generator_name = generator_name_msg;

    output.candidate_trajectories.candidate_trajectories.push_back(candidate_trajectory);
    output.candidate_trajectories.generator_info.push_back(generator_info);
  }

  // PredictedObjects
  // Use the first prediction as the main predicted objects
  constexpr int64_t batch_idx = 0;
  output.predicted_objects =
    postprocess::create_predicted_objects(agent_poses, selected_agents_, timestamp, batch_idx);

  return output;
}

autoware_perception_msgs::msg::TrafficLightGroup MLPlannerCore::get_first_traffic_light_on_route()
  const
{
  if (!lane_segment_context_ || !route_ptr_ || ego_history_.empty()) {
    return autoware_perception_msgs::msg::TrafficLightGroup{};
  }

  const geometry_msgs::msg::Pose & pose_center = ego_history_.back().pose.pose;

  const double center_x = pose_center.position.x;
  const double center_y = pose_center.position.y;
  const double center_z = pose_center.position.z;

  const auto traffic_light_id_map = preprocess::create_traffic_signal_map(
    traffic_signals_history_.msgs(), frame_time(), params_.traffic_light_group_msg_timeout_seconds);

  return lane_segment_context_->get_first_traffic_light_on_route(
    *route_ptr_, center_x, center_y, center_z, traffic_light_id_map);
}

int64_t MLPlannerCore::count_valid_elements(
  const TensorMap & input_data_map, const std::string & data_key) const
{
  const int64_t batch_idx = 0;

  if (data_key == "lanes") {
    return postprocess::count_valid_elements(
      input_data_map.at("lanes"), LANES_SHAPE[1], LANES_SHAPE[2], LANES_SHAPE[3], batch_idx);
  } else if (data_key == "route_lanes") {
    return postprocess::count_valid_elements(
      input_data_map.at("route_lanes"), ROUTE_LANES_SHAPE[1], ROUTE_LANES_SHAPE[2],
      ROUTE_LANES_SHAPE[3], batch_idx);
  } else if (data_key == "intersection_area") {
    return postprocess::count_valid_elements(
      input_data_map.at("intersection_area"), INTERSECTION_AREA_SHAPE[1],
      INTERSECTION_AREA_SHAPE[2], INTERSECTION_AREA_SHAPE[3], batch_idx);
  } else if (data_key == "stop_lines") {
    return postprocess::count_valid_elements(
      input_data_map.at("stop_lines"), STOP_LINES_SHAPE[1], STOP_LINES_SHAPE[2],
      STOP_LINES_SHAPE[3], batch_idx);
  } else if (data_key == "road_borders") {
    return postprocess::count_valid_elements(
      input_data_map.at("road_borders"), ROAD_BORDERS_SHAPE[1], ROAD_BORDERS_SHAPE[2],
      ROAD_BORDERS_SHAPE[3], batch_idx);
  } else if (data_key == "neighbor_agents_past") {
    return postprocess::count_valid_elements(
      input_data_map.at("neighbor_agents_past"), NEIGHBOR_SHAPE[1], NEIGHBOR_SHAPE[2],
      NEIGHBOR_SHAPE[3], batch_idx);
  }

  throw std::invalid_argument("Unknown data_key '" + data_key + "' in count_valid_elements()");
}

}  // namespace autoware::ml_planner
