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
namespace
{
#ifdef AUTOWARE_ML_PLANNER_USE_ACADOS
bool optimization_params_changed(
  const optimization::TrajectoryOptimizationParams & lhs,
  const optimization::TrajectoryOptimizationParams & rhs)
{
  return lhs.enable != rhs.enable || lhs.weight_longitudinal != rhs.weight_longitudinal ||
         lhs.weight_lateral != rhs.weight_lateral || lhs.weight_yaw != rhs.weight_yaw ||
         lhs.weight_velocity != rhs.weight_velocity ||
         lhs.weight_steering_angle != rhs.weight_steering_angle ||
         lhs.weight_acceleration != rhs.weight_acceleration ||
         lhs.weight_steering_rate != rhs.weight_steering_rate ||
         lhs.terminal_weight_scale != rhs.terminal_weight_scale ||
         lhs.goal.weight_longitudinal != rhs.goal.weight_longitudinal ||
         lhs.goal.weight_lateral != rhs.goal.weight_lateral ||
         lhs.goal.weight_yaw != rhs.goal.weight_yaw ||
         lhs.goal.weight_velocity != rhs.goal.weight_velocity ||
         lhs.goal.snap_distance_m != rhs.goal.snap_distance_m ||
         lhs.goal.unlatch_horizon_s != rhs.goal.unlatch_horizon_s ||
         lhs.goal.unlatch_min_speed_mps != rhs.goal.unlatch_min_speed_mps ||
         lhs.min_velocity_mps != rhs.min_velocity_mps ||
         lhs.max_velocity_mps != rhs.max_velocity_mps ||
         lhs.min_acceleration_mps2 != rhs.min_acceleration_mps2 ||
         lhs.max_acceleration_mps2 != rhs.max_acceleration_mps2 ||
         lhs.max_steering_rate_rps != rhs.max_steering_rate_rps ||
         lhs.max_lateral_acceleration_mps2 != rhs.max_lateral_acceleration_mps2 ||
         lhs.max_sqp_iterations != rhs.max_sqp_iterations ||
         lhs.temporal_consistency.enable != rhs.temporal_consistency.enable ||
         lhs.temporal_consistency.weight_longitudinal !=
           rhs.temporal_consistency.weight_longitudinal ||
         lhs.temporal_consistency.weight_lateral != rhs.temporal_consistency.weight_lateral ||
         lhs.temporal_consistency.weight_yaw != rhs.temporal_consistency.weight_yaw ||
         lhs.temporal_consistency.weight_velocity != rhs.temporal_consistency.weight_velocity ||
         lhs.temporal_consistency.decay_time_constant_s !=
           rhs.temporal_consistency.decay_time_constant_s ||
         lhs.temporal_consistency.far_weight_ratio != rhs.temporal_consistency.far_weight_ratio;
}
#endif

bool road_border_avoidance_params_changed(
  const postprocess::RoadBorderAvoidanceParams & lhs,
  const postprocess::RoadBorderAvoidanceParams & rhs)
{
  return lhs.enable != rhs.enable || lhs.start_time_s != rhs.start_time_s ||
         lhs.footprint_margin_m != rhs.footprint_margin_m ||
         lhs.search_radius_m != rhs.search_radius_m || lhs.shift_step_m != rhs.shift_step_m ||
         lhs.max_lateral_shift_m != rhs.max_lateral_shift_m ||
         lhs.propagate_shift != rhs.propagate_shift ||
         lhs.max_reoptimizations != rhs.max_reoptimizations;
}

/**
 * @brief Move the reference by the shift the road border avoidance applied to `target`.
 *
 * Reference point k is moved by (adjusted[k] - target[k]), with its total offset from
 * raw[k] capped at max_shift_m. Point k is the same stage (t = (k + 1) * 0.1 s) in all four
 * trajectories. For the first check target is the raw output itself; afterwards it is the
 * optimized trajectory, so the reference is pushed out by the clearance the solution lacked.
 *
 * @return false if no reference point moved (nothing to gain from solving again).
 */
bool shift_reference(
  Trajectory & reference, const Trajectory & raw, const Trajectory & target,
  const Trajectory & adjusted, const double max_shift_m)
{
  constexpr double min_move_m = 1e-3;
  const size_t size = std::min(
    {reference.points.size(), raw.points.size(), target.points.size(), adjusted.points.size()});
  bool moved = false;
  for (size_t k = 0; k < size; ++k) {
    auto & position = reference.points[k].pose.position;
    const auto & raw_position = raw.points[k].pose.position;
    const auto & target_position = target.points[k].pose.position;
    const auto & adjusted_position = adjusted.points[k].pose.position;
    Eigen::Vector2d offset(
      position.x - raw_position.x + adjusted_position.x - target_position.x,
      position.y - raw_position.y + adjusted_position.y - target_position.y);
    const double norm = offset.norm();
    if (norm > max_shift_m) {
      offset *= max_shift_m / norm;
    }
    const double new_x = raw_position.x + offset.x();
    const double new_y = raw_position.y + offset.y();
    moved = moved || std::hypot(new_x - position.x, new_y - position.y) > min_move_m;
    position.x = new_x;
    position.y = new_y;
  }
  return moved;
}
}  // namespace

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
: params_(params), vehicle_info_(vehicle_info), vehicle_spec_(vehicle_info)
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
#ifdef AUTOWARE_ML_PLANNER_USE_ACADOS
  const bool rebuild_optimizer =
    params.batch_size != params_.batch_size ||
    optimization_params_changed(params.trajectory_optimization, params_.trajectory_optimization);
#endif
  const bool rebuild_avoidance = road_border_avoidance_params_changed(
    params.road_border_avoidance, params_.road_border_avoidance);

  params_ = params;

#ifdef AUTOWARE_ML_PLANNER_USE_ACADOS
  if (rebuild_optimizer) {
    trajectory_optimizer_.reset();
    if (params_.trajectory_optimization.enable) {
      trajectory_optimizer_ = std::make_unique<optimization::TrajectoryOptimizer>(
        params_.trajectory_optimization, vehicle_info_, static_cast<size_t>(params_.batch_size));
    }
  }
#endif

  if (rebuild_avoidance) {
    road_border_avoidance_.reset();
    if (params_.road_border_avoidance.enable) {
      road_border_avoidance_ = std::make_unique<postprocess::RoadBorderAvoidance>(
        params_.road_border_avoidance, vehicle_info_);
      if (lanelet_map_ptr_) {
        road_border_avoidance_->set_map(*lanelet_map_ptr_);
      }
    }
  }
}

void MLPlannerCore::set_map(const std::shared_ptr<const lanelet::LaneletMap> & lanelet_map_ptr)
{
  lanelet_map_ptr_ = lanelet_map_ptr;
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

  if (params_.virtual_pose.enable) {
    frame_ego_ = build_frame_ego();
  } else {
    frame_ego_ = ego_history_.back();
    virtual_pose_result_.reset();
    virtual_history_.clear();
  }
  const auto & frame_ego_history =
    params_.virtual_pose.enable ? virtual_history_.msgs() : ego_history_.msgs();

  return preprocess::FrameInputs{
    frame_time(),
    preprocess::MessageView<nav_msgs::msg::Odometry>{frame_ego_history},
    preprocess::MessageView<autoware_vehicle_msgs::msg::TurnIndicatorsReport>{
      turn_indicators_history_.msgs()},
    preprocess::MessageView<autoware_perception_msgs::msg::TrackedObjects>{objects_history_.msgs()},
    preprocess::MessageView<autoware_perception_msgs::msg::TrafficLightGroupArray>{
      traffic_signals_history_.msgs()},
    *route_ptr_};
}

Odometry MLPlannerCore::build_frame_ego()
{
  const Odometry & measured = ego_history_.back();
  if (virtual_history_.empty()) {
    for (const auto & msg : ego_history_.msgs()) {
      virtual_history_.push_back(msg);
    }
  }

  virtual_pose_result_ = utils::VirtualPoseResult{measured.pose.pose, false, false, 0.0, 0.0};
  if (previous_frame_pose_ && !previous_ego_prediction_.empty()) {
    // Earlier frame poses, oldest first, at least 5 cm apart, excluding the previous frame pose
    // itself (the newest history entry), then the previous frame pose and its prediction.
    constexpr double MIN_PREFIX_SPACING_M = 0.05;
    std::vector<Eigen::Matrix4d> newest_first;
    Eigen::Vector2d successor = previous_frame_pose_->block<2, 1>(0, 3);
    const auto & past = virtual_history_.msgs();
    for (auto it = past.rbegin(); it != past.rend() && static_cast<int64_t>(newest_first.size()) <
                                                         params_.virtual_pose.history_prefix_count;
         ++it) {
      const Eigen::Matrix4d pose = utils::pose_to_matrix4d(it->pose.pose);
      if ((pose.block<2, 1>(0, 3) - successor).norm() < MIN_PREFIX_SPACING_M) {
        continue;
      }
      newest_first.push_back(pose);
      successor = pose.block<2, 1>(0, 3);
    }
    std::vector<Eigen::Matrix4d> polyline(newest_first.rbegin(), newest_first.rend());
    const auto prefix_count = static_cast<int64_t>(polyline.size());
    polyline.push_back(*previous_frame_pose_);
    polyline.insert(
      polyline.end(), previous_ego_prediction_.begin(), previous_ego_prediction_.end());

    virtual_pose_result_ = utils::compute_virtual_pose(
      measured.pose.pose, polyline, prefix_count, params_.virtual_pose);
  }

  if (virtual_pose_result_->reset) {
    // The frame pose jumps onto the vehicle: a past of earlier virtual poses would end in that
    // jump, which the model reads as a sudden lateral motion. Restart the past from the measured
    // poses, which end at the new frame pose.
    virtual_history_.clear();
    for (const auto & msg : ego_history_.msgs()) {
      virtual_history_.push_back(msg);
    }
    return measured;
  }

  Odometry frame = measured;
  frame.pose.pose = virtual_pose_result_->pose;
  virtual_history_.push_back(frame);
  return frame;
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

RefinedCandidate MLPlannerCore::refine_candidate(
  const Trajectory & raw_trajectory, const size_t batch_index, const Odometry & kinematic_state,
  const double current_steering_angle_rad)
{
  RefinedCandidate candidate;
  candidate.reference = raw_trajectory;
  candidate.trajectory = raw_trajectory;
  const auto & ego_pose = kinematic_state.pose.pose;
  const double max_shift_m = params_.road_border_avoidance.max_lateral_shift_m;

  // Road border check of the raw output: the shifted output becomes the reference.
  if (road_border_avoidance_) {
    const auto check = road_border_avoidance_->adjust(raw_trajectory, ego_pose);
    candidate.avoidance_debug.active = true;
    candidate.avoidance_debug.shifted_points = static_cast<int>(check.num_shifted_points);
    candidate.avoidance_debug.unresolved_points = static_cast<int>(check.num_unresolved_points);
    shift_reference(
      candidate.reference, raw_trajectory, raw_trajectory, check.trajectory, max_shift_m);
  }

  // Post-processing for the path without the optimizer, from the planning start pose so the
  // published trajectory and a virtual pose taken from it never return onto the vehicle.
  if (params_.path_smoothing.enable) {
    postprocess::smooth_initial_path(
      candidate.reference, ego_pose, kinematic_state.twist.twist.linear.x, params_.path_smoothing);
    postprocess::smooth_path_tail(candidate.reference, params_.path_smoothing);
  }
  if (params_.velocity_smoothing.enable) {
    postprocess::smooth_initial_velocity(candidate.reference, params_.velocity_smoothing);
  }
  if (params_.curve_speed_limit.enable) {
    postprocess::limit_curve_speed(candidate.reference, params_.curve_speed_limit);
  }
  candidate.trajectory = candidate.reference;

#ifdef AUTOWARE_ML_PLANNER_USE_ACADOS
  if (!trajectory_optimizer_) {
    return candidate;
  }

  trajectory_optimizer_->latch_goal_if_reached(candidate.reference);
  const auto solve = [&](const Trajectory & reference) {
    return trajectory_optimizer_->optimize(
      reference, kinematic_state, current_steering_angle_rad, batch_index);
  };
  auto result = solve(candidate.reference);
  double solve_time_ms = result.solve_time_ms;

  // The solver trades the reference off against smoothness, so the optimized path may cut
  // back into a border the reference was shifted away from. Check it the same way and, while
  // it still overlaps, push the reference out by the missing clearance and solve again. All
  // solves share the previous cycle's warm start; only the solution used is accepted.
  int reoptimizations = 0;
  while (road_border_avoidance_ && result.optimized) {
    const auto check = road_border_avoidance_->adjust(result.trajectory, ego_pose);
    candidate.avoidance_debug.remaining_overlapping_points =
      static_cast<int>(check.num_shifted_points + check.num_unresolved_points);
    if (!check.modified() || reoptimizations >= params_.road_border_avoidance.max_reoptimizations) {
      break;
    }
    auto next_reference = candidate.reference;
    if (!shift_reference(
          next_reference, raw_trajectory, result.trajectory, check.trajectory, max_shift_m)) {
      break;
    }
    auto retry = solve(next_reference);
    ++reoptimizations;
    solve_time_ms += retry.solve_time_ms;
    if (!retry.optimized) {
      // Keep the last solution that converged.
      break;
    }
    candidate.reference = std::move(next_reference);
    result = std::move(retry);
  }
  candidate.avoidance_debug.reoptimizations = reoptimizations;

  trajectory_optimizer_->accept(batch_index, result);
  auto & optimization_debug = candidate.optimization_debug;
  optimization_debug.attempted = true;
  optimization_debug.optimized = result.optimized;
  optimization_debug.solver_status = result.solver_status;
  optimization_debug.solve_time_ms = solve_time_ms;
  candidate.trajectory =
    result.optimized ? std::make_optional(std::move(result.trajectory)) : std::nullopt;
#else
  (void)batch_index;
  (void)current_steering_angle_rad;
#endif

  return candidate;
}

PlannerOutput MLPlannerCore::create_planner_output(
  const InferenceOutput & inference_output, const rclcpp::Time & timestamp,
  const UUID & generator_uuid, const double current_steering_angle_rad)
{
  // The model output is in the frame it was given (frame_ego_, the virtual pose when enabled).
  // Everything downstream (border avoidance, smoothing, optimization) starts from that same
  // pose: started at the measured pose, the optimized trajectory (and a virtual pose taken from
  // it) would return onto the vehicle every cycle. Speed and steering stay measured so the start
  // is physically consistent.
  const Odometry & kinematic_state = ego_history_.back();
  Odometry planning_start = kinematic_state;
  if (params_.virtual_pose.enable) {
    planning_start.pose.pose = frame_ego_.pose.pose;
  }
  const Eigen::Matrix4d ego_to_map_transform = utils::pose_to_matrix4d(frame_ego_.pose.pose);

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
  if (params_.virtual_pose.reference == "raw") {
    previous_frame_pose_ = ego_to_map_transform;
    previous_ego_prediction_ = agent_poses.front().front();
  }

#ifdef AUTOWARE_ML_PLANNER_USE_ACADOS
  if (trajectory_optimizer_) {
    trajectory_optimizer_->set_goal(
      route_ptr_ ? std::make_optional(route_ptr_->goal_pose) : std::nullopt, kinematic_state);
    // A reset moved the planning start onto the vehicle: the previous plan started at the old
    // virtual pose, so neither its warm start nor its temporal pull may shape this one.
    if (params_.virtual_pose.enable && virtual_pose_result_ && virtual_pose_result_->reset) {
      trajectory_optimizer_->drop_previous_solutions();
    }
  }
#endif

  PlannerOutput output;
  // Trajectory and CandidateTrajectories
  for (int i = 0; i < params_.batch_size; i++) {
    auto trajectory =
      postprocess::create_ego_trajectory(agent_poses, timestamp, frame_ego_.pose.pose.position, i);

    auto refined = refine_candidate(
      trajectory, static_cast<size_t>(i), planning_start, current_steering_angle_rad);
    if (i == 0) {
      // Keep the untouched model output for the debug topics.
      output.raw_trajectory = trajectory;
      output.avoidance_debug = refined.avoidance_debug;
      output.optimization_debug = refined.optimization_debug;
      if (refined.avoidance_debug.active) {
        output.avoidance_adjusted_trajectory = refined.reference;
      }
    }
    // A candidate whose optimization failed is dropped entirely rather than falling back to
    // the raw model output (see PlannerOutput::trajectory).
    if (!refined.trajectory) {
      continue;
    }
    trajectory = std::move(*refined.trajectory);

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

  // With the optimized reference, the next virtual pose is taken from the trajectory that is
  // actually published. A cycle whose optimization failed publishes none and keeps the previous
  // one.
  if (
    params_.virtual_pose.reference == "optimized" && output.trajectory &&
    output.trajectory->points.size() >= 2) {
    const auto & points = output.trajectory->points;
    previous_frame_pose_ = utils::pose_to_matrix4d(points.front().pose);
    previous_ego_prediction_.clear();
    for (size_t k = 1; k < points.size(); ++k) {
      previous_ego_prediction_.push_back(utils::pose_to_matrix4d(points[k].pose));
    }
  }
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
