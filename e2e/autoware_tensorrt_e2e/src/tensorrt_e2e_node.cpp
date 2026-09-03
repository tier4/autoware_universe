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

#include "autoware/tensorrt_e2e/tensorrt_e2e_node.hpp"

#include "autoware/tensorrt_e2e/input_provider_registry.hpp"

#include <autoware/diffusion_planner/postprocessing/postprocessing_utils.hpp>
#include <autoware/diffusion_planner/preprocessing/preprocessing_utils.hpp>
#include <autoware/diffusion_planner/utils/utils.hpp>
#include <autoware_utils_uuid/uuid_helper.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace autoware::tensorrt_e2e
{
namespace dp = autoware::diffusion_planner;
using autoware_internal_planning_msgs::msg::PlanningFactor;
using diagnostic_msgs::msg::DiagnosticStatus;

namespace
{
constexpr int64_t LOG_THROTTLE_INTERVAL_MS = 5000;
}  // namespace

TensorrtE2eNode::TensorrtE2eNode(const rclcpp::NodeOptions & options)
: Node("tensorrt_e2e", options),
  tf_buffer_(this->get_clock()),
  generator_uuid_(autoware_utils_uuid::generate_uuid())
{
  set_up_params();
  vehicle_info_ = autoware::vehicle_info_utils::VehicleInfoUtils(*this).getVehicleInfo();
  base_link_to_center_ =
    (vehicle_info_.front_overhang_m + vehicle_info_.wheel_base_m - vehicle_info_.rear_overhang_m) /
    2.0;
  postprocess_params_.base_link_offset = params_.shift_x ? base_link_to_center_ : 0.0;

  pub_trajectory_ = create_publisher<Trajectory>("~/output/trajectory", 1);
  pub_trajectories_ = create_publisher<CandidateTrajectories>("~/output/trajectories", 1);
  pub_objects_ = create_publisher<PredictedObjects>("~/output/predicted_objects", rclcpp::QoS(1));
  pub_processing_time_ = create_publisher<autoware_internal_debug_msgs::msg::Float64Stamped>(
    "~/debug/processing_time_ms", 1);
  diagnostics_ = std::make_unique<DiagnosticsInterface>(this, "inference_status");
  debug_publisher_ = std::make_unique<autoware_utils_debug::DebugPublisher>(this, get_name());
  planning_factor_interface_ =
    std::make_unique<autoware::planning_factor_interface::PlanningFactorInterface>(
      this, "tensorrt_e2e");
  stop_watch_.tic("cyclic");

  try {
    initialize_pipeline();
    pipeline_ready_ = true;
    if (params_.build_only) {
      RCLCPP_INFO(get_logger(), "Build only mode enabled. Exiting after building the engine.");
      std::exit(EXIT_SUCCESS);
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR_STREAM(get_logger(), e.what() << ". Inference will be disabled.");
    diagnostics_->update_level_and_message(DiagnosticStatus::ERROR, e.what());
    diagnostics_->publish(get_clock()->now());
    if (params_.build_only) {
      RCLCPP_ERROR(get_logger(), "Build only mode: exiting due to initialization failure.");
      std::exit(EXIT_FAILURE);
    }
  }

  timer_ = rclcpp::create_timer(
    this, get_clock(), rclcpp::Rate(params_.planning_frequency_hz).period(),
    std::bind(&TensorrtE2eNode::on_timer, this));
}

void TensorrtE2eNode::set_up_params()
{
  params_.model_path = declare_parameter<std::string>("model_path", "");
  params_.plugins_path = declare_parameter<std::string>("plugins_path", "");
  params_.precision = declare_parameter<std::string>("precision", "fp16");
  params_.trt_workspace_mib = declare_parameter<int64_t>("trt_workspace_mib", 4096);
  params_.args_path = declare_parameter<std::string>("args_path", "");
  params_.build_only = declare_parameter<bool>("build_only", false);
  params_.planning_frequency_hz = declare_parameter<double>("planning_frequency_hz", 10.0);
  params_.shift_x = declare_parameter<bool>("shift_x", false);
  params_.sensor_inputs = declare_parameter<std::vector<std::string>>(
    "sensor_inputs", std::vector<std::string>{});
  params_.enable_context_inputs = declare_parameter<bool>("enable_context_inputs", true);

  postprocess_params_.prediction_tensor =
    declare_parameter<std::string>("postprocess.prediction_tensor", "prediction");
  postprocess_params_.extra_trajectory_tensors = declare_parameter<std::vector<std::string>>(
    "postprocess.extra_trajectory_tensors", std::vector<std::string>{});
  postprocess_params_.horizon_seconds =
    declare_parameter<double>("postprocess.horizon_seconds", 4.0);
  postprocess_params_.time_step = declare_parameter<double>("postprocess.time_step", 0.1);
  postprocess_params_.velocity_smoothing_window =
    declare_parameter<int64_t>("postprocess.velocity_smoothing_window", 8);
  postprocess_params_.stopping_threshold =
    declare_parameter<double>("postprocess.stopping_threshold", 0.3);
  postprocess_params_.generator_name =
    declare_parameter<std::string>("postprocess.generator_name", "TensorrtE2e");

  planning_factor_params_.enable_stop = declare_parameter<bool>("planning_factor.enable_stop", false);
  planning_factor_params_.enable_slowdown =
    declare_parameter<bool>("planning_factor.enable_slowdown", false);
  planning_factor_params_.detection_config.stop_velocity_threshold =
    declare_parameter<double>("planning_factor.stop_velocity_threshold", 0.1);
  planning_factor_params_.detection_config.stop_keep_duration_threshold =
    declare_parameter<double>("planning_factor.stop_keep_duration_threshold", 1.0);
  planning_factor_params_.detection_config.slowdown_accel_threshold =
    declare_parameter<double>("planning_factor.slowdown_accel_threshold", -0.3);
}

void TensorrtE2eNode::create_providers()
{
  // Providers come from the registry, each registered by its own source file, so this
  // node is the same on every model line regardless of which providers a line ships.
  for (const auto & sensor : params_.sensor_inputs) {
    providers_.push_back(make_input_provider(sensor, *this, tf_buffer_));
  }
  if (params_.enable_context_inputs) {
    auto context_provider = std::make_unique<ContextInputProvider>(*this, vehicle_info_);
    context_provider_ = context_provider.get();
    providers_.push_back(std::move(context_provider));
  }
  if (providers_.empty()) {
    throw std::runtime_error(
      "No input providers configured: set sensor_inputs and/or enable_context_inputs");
  }
}

void TensorrtE2eNode::initialize_pipeline()
{
  diagnostics_->update_level_and_message(DiagnosticStatus::WARN, "Loading model");
  diagnostics_->publish(get_clock()->now());

  InferenceEngine::Config engine_config;
  engine_config.model_path = params_.model_path;
  engine_config.plugins_path = params_.plugins_path;
  engine_config.precision = params_.precision;
  engine_config.max_workspace_size =
    static_cast<size_t>(params_.trt_workspace_mib) * 1024ULL * 1024ULL;
  engine_ = std::make_unique<InferenceEngine>(engine_config);

  {
    std::ostringstream manifest;
    for (const auto & spec : engine_->input_specs()) {
      manifest << " " << spec.name << shape_to_string(spec.shape);
    }
    RCLCPP_INFO_STREAM(get_logger(), "Engine inputs:" << manifest.str());
  }

  create_providers();

  // Match provider claims against the engine input manifest.
  std::map<std::string, std::string> claimed_by;  // tensor name -> provider name
  for (const auto & provider : providers_) {
    for (const auto & tensor_name : provider->claim_inputs(engine_->input_specs())) {
      const auto [it, inserted] = claimed_by.emplace(tensor_name, provider->name());
      if (!inserted) {
        throw std::runtime_error(
          "Input tensor '" + tensor_name + "' is claimed by both '" + it->second + "' and '" +
          provider->name() + "'");
      }
      RCLCPP_INFO(
        get_logger(), "Input '%s' is provided by '%s'", tensor_name.c_str(),
        provider->name().c_str());
    }
  }
  std::vector<std::string> unclaimed;
  for (const auto & spec : engine_->input_specs()) {
    if (claimed_by.find(spec.name) == claimed_by.end()) {
      unclaimed.push_back(spec.name + shape_to_string(spec.shape));
    }
  }
  if (!unclaimed.empty()) {
    std::ostringstream oss;
    oss << "No provider produces the following model inputs:";
    for (const auto & name : unclaimed) {
      oss << " " << name;
    }
    oss << ". Enable the matching provider (sensor_inputs / enable_context_inputs) or adjust "
           "the tensor name parameters.";
    throw std::runtime_error(oss.str());
  }

  postprocessor_ = std::make_unique<TrajectoryPostprocessor>(postprocess_params_);
  postprocessor_->validate_output_specs(engine_->output_specs());

  if (!params_.args_path.empty()) {
    normalization_map_ = dp::utils::load_normalization_stats(params_.args_path);
    RCLCPP_INFO_STREAM(
      get_logger(), "Loaded normalization stats for " << normalization_map_.size()
                                                      << " tensors from " << params_.args_path);
  }

  diagnostics_->update_level_and_message(DiagnosticStatus::OK, "Model loaded");
  diagnostics_->publish(get_clock()->now());
}

std::optional<EgoFrame> TensorrtE2eNode::create_ego_frame()
{
  const auto odometry = sub_odometry_.take_data();
  if (!odometry) {
    return std::nullopt;
  }
  const auto acceleration = sub_acceleration_.take_data();

  EgoFrame ego;
  ego.odometry = *odometry;
  ego.reference_odometry = *odometry;
  if (params_.shift_x) {
    ego.reference_odometry.pose.pose =
      dp::utils::shift_x(odometry->pose.pose, base_link_to_center_);
  }
  if (acceleration) {
    ego.acceleration = *acceleration;
  }
  ego.ego_to_map = dp::utils::pose_to_matrix4d(ego.reference_odometry.pose.pose);
  ego.map_to_ego = dp::utils::inverse(ego.ego_to_map);
  ego.stamp = rclcpp::Time(odometry->header.stamp);
  return ego;
}

void TensorrtE2eNode::apply_normalization(TensorMap & inputs) const
{
  if (normalization_map_.empty()) {
    return;
  }
  // Delegate to the diffusion planner implementation for tensors that have stats; it throws on
  // missing keys, so tensors without stats (e.g. GPU-normalized images) are filtered out first.
  dp::preprocess::InputDataMap to_normalize;
  for (auto & [name, tensor] : inputs) {
    if (!tensor.is_device() && normalization_map_.count(name) > 0) {
      to_normalize.emplace(name, std::move(tensor.host_data));
    }
  }
  dp::preprocess::normalize_input_data(to_normalize, normalization_map_);
  for (auto & [name, data] : to_normalize) {
    inputs[name].host_data = std::move(data);
  }
}

std::optional<std::string> TensorrtE2eNode::find_invalid_tensor(const TensorMap & inputs)
{
  for (const auto & [name, tensor] : inputs) {
    if (tensor.is_device()) {
      continue;
    }
    const bool valid = std::all_of(
      tensor.host_data.begin(), tensor.host_data.end(),
      [](const float value) { return std::isfinite(value); });
    if (!valid) {
      return name;
    }
  }
  return std::nullopt;
}

void TensorrtE2eNode::on_timer()
{
  stop_watch_.tic("processing_time");
  diagnostics_->clear();
  const rclcpp::Time now = get_clock()->now();

  const auto finish = [this, &now](const int8_t level, const std::string & message) {
    if (level != DiagnosticStatus::OK) {
      diagnostics_->update_level_and_message(level, message);
    }
    diagnostics_->add_key_value("processing_time_ms", stop_watch_.toc("processing_time"));
    diagnostics_->publish(now);
  };

  if (!pipeline_ready_) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), LOG_THROTTLE_INTERVAL_MS,
      "Pipeline is not initialized. Inference is disabled (check the startup errors).");
    finish(DiagnosticStatus::ERROR, "Pipeline not initialized");
    return;
  }

  const auto ego = create_ego_frame();
  if (!ego) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), LOG_THROTTLE_INTERVAL_MS, "Waiting for odometry...");
    finish(DiagnosticStatus::WARN, "No odometry received");
    return;
  }

  // Collect all model inputs.
  TickTiming timing;
  stop_watch_.tic("collect");
  TensorMap inputs;
  for (const auto & provider : providers_) {
    std::string error;
    if (!provider->collect(*ego, now, inputs, error)) {
      RCLCPP_WARN_STREAM_THROTTLE(
        get_logger(), *get_clock(), LOG_THROTTLE_INTERVAL_MS,
        "Input collection failed [" << provider->name() << "]: " << error);
      finish(DiagnosticStatus::WARN, "[" + provider->name() + "] " + error);
      return;
    }
  }

  apply_normalization(inputs);
  add_input_diagnostics(inputs);
  if (const auto invalid_tensor = find_invalid_tensor(inputs)) {
    RCLCPP_WARN_STREAM_THROTTLE(
      get_logger(), *get_clock(), LOG_THROTTLE_INTERVAL_MS,
      "Input tensor '" << *invalid_tensor << "' contains invalid values");
    finish(DiagnosticStatus::WARN, "Input tensor '" + *invalid_tensor + "' has invalid values");
    return;
  }

  // Inference.
  timing.collect_ms = stop_watch_.toc("collect");
  stop_watch_.tic("inference");
  const auto result = engine_->infer(inputs);
  timing.inference_ms = stop_watch_.toc("inference");
  if (!result.outputs) {
    RCLCPP_WARN_STREAM_THROTTLE(
      get_logger(), *get_clock(), LOG_THROTTLE_INTERVAL_MS,
      "Inference failed: " << result.error_msg);
    finish(DiagnosticStatus::ERROR, result.error_msg);
    return;
  }

  // Postprocess and publish.
  stop_watch_.tic("postprocess");
  TrajectoryPostprocessor::Output output;
  try {
    const auto * neighbor_histories =
      context_provider_ ? &context_provider_->last_neighbor_histories() : nullptr;
    output = postprocessor_->process(
      *result.outputs, *ego, neighbor_histories, ego->stamp, generator_uuid_);
  } catch (const std::exception & e) {
    RCLCPP_ERROR_STREAM(get_logger(), "Postprocessing failed: " << e.what());
    finish(DiagnosticStatus::ERROR, e.what());
    return;
  }

  timing.postprocess_ms = stop_watch_.toc("postprocess");

  pub_trajectory_->publish(output.trajectory);
  pub_trajectories_->publish(output.candidate_trajectories);
  if (output.predicted_objects) {
    pub_objects_->publish(*output.predicted_objects);
  }
  publish_planning_factor(output.trajectory);

  // Timing: the whole tick must fit in the planning period to sustain the output rate.
  const double processing_time_ms = stop_watch_.toc("processing_time");
  timing.total_ms = processing_time_ms;
  publish_debug_timing(now, *ego, timing);
  const double period_ms = 1e3 / params_.planning_frequency_hz;
  autoware_internal_debug_msgs::msg::Float64Stamped processing_time_msg;
  processing_time_msg.stamp = now;
  processing_time_msg.data = processing_time_ms;
  pub_processing_time_->publish(processing_time_msg);

  diagnostics_->add_key_value("processing_time_ms", processing_time_ms);
  if (processing_time_ms > period_ms) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), LOG_THROTTLE_INTERVAL_MS,
      "Processing took %.1f ms, exceeding the %.1f ms planning period", processing_time_ms,
      period_ms);
    diagnostics_->update_level_and_message(
      DiagnosticStatus::WARN, "Processing time exceeded the planning period");
  }
  diagnostics_->publish(now);
}

void TensorrtE2eNode::add_input_diagnostics(const TensorMap & inputs)
{
  for (const auto & provider : providers_) {
    provider->add_diagnostics(*diagnostics_);
  }
  // Same keys and the same counting as autoware_diffusion_planner: a [1, N, P, D] tensor's
  // valid elements are its non-zero rows in batch 0.
  static const std::vector<std::pair<const char *, const char *>> kCounted = {
    {"lanes", "valid_lane_count"},
    {"route_lanes", "valid_route_count"},
    {"polygons", "valid_polygon_count"},
    {"line_strings", "valid_line_string_count"},
    {"neighbor_agents_past", "valid_neighbor_count"},
  };
  for (const auto & [tensor_name, key] : kCounted) {
    const auto it = inputs.find(tensor_name);
    if (it == inputs.end() || it->second.is_device() || it->second.shape.size() != 4) {
      continue;
    }
    const auto & shape = it->second.shape;
    diagnostics_->add_key_value(
      key, dp::postprocess::count_valid_elements(
             it->second.host_data, shape[1], shape[2], shape[3], /*batch_idx=*/0));
  }
}

void TensorrtE2eNode::publish_planning_factor(const Trajectory & trajectory)
{
  const auto & points = trajectory.points;
  const auto detected =
    dp::detect_planning_factors(points, planning_factor_params_.detection_config);

  if (planning_factor_params_.enable_stop && detected.stop) {
    const auto & stop = *detected.stop;
    planning_factor_interface_->add(
      points, stop.ego_pose, stop.stop_pose, PlanningFactor::STOP,
      autoware_internal_planning_msgs::msg::SafetyFactorArray{});
  }
  if (planning_factor_params_.enable_slowdown && detected.slowdown) {
    const auto & slowdown = *detected.slowdown;
    planning_factor_interface_->add(
      points, slowdown.ego_pose, slowdown.start_pose, slowdown.end_pose, PlanningFactor::SLOW_DOWN,
      autoware_internal_planning_msgs::msg::SafetyFactorArray{}, true, slowdown.start_velocity,
      slowdown.end_velocity);
  }
  planning_factor_interface_->publish();
}

void TensorrtE2eNode::publish_debug_timing(
  const rclcpp::Time & now, const EgoFrame & ego, const TickTiming & timing)
{
  using autoware_internal_debug_msgs::msg::Float64Stamped;
  // Latency is measured from the freshest sensor frame behind this output, as bevfusion
  // measures it from its cloud stamp; without a sensor provider, from the ego frame.
  rclcpp::Time input_stamp = ego.stamp;
  for (const auto & provider : providers_) {
    if (const auto stamp = provider->latest_input_stamp()) {
      input_stamp = *stamp;
      break;
    }
  }
  debug_publisher_->publish<Float64Stamped>("debug/cyclic_time_ms", stop_watch_.toc("cyclic", true));
  debug_publisher_->publish<Float64Stamped>(
    "debug/pipeline_latency_ms", (now - input_stamp).seconds() * 1e3);
  debug_publisher_->publish<Float64Stamped>("debug/processing_time/total_ms", timing.total_ms);
  debug_publisher_->publish<Float64Stamped>("debug/processing_time/collect_ms", timing.collect_ms);
  debug_publisher_->publish<Float64Stamped>(
    "debug/processing_time/inference_ms", timing.inference_ms);
  debug_publisher_->publish<Float64Stamped>(
    "debug/processing_time/postprocess_ms", timing.postprocess_ms);
}

}  // namespace autoware::tensorrt_e2e

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::tensorrt_e2e::TensorrtE2eNode)
