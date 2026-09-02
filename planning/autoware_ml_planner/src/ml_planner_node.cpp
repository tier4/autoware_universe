// Copyright 2025 TIER IV, Inc.
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

#include "autoware/ml_planner/ml_planner_node.hpp"

#include "autoware/ml_planner/constants.hpp"
#include "autoware/ml_planner/dimensions.hpp"
#include "autoware/ml_planner/preprocessing/preprocessing_utils.hpp"
#include "autoware/ml_planner/utils/marker_utils.hpp"
#include "autoware/ml_planner/utils/utils.hpp"

#include <autoware_utils_uuid/uuid_helper.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/logging.hpp>
#include <xtensor/xview.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <functional>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::ml_planner
{
using diagnostic_msgs::msg::DiagnosticStatus;

namespace
{
// Ego velocity [m/s] written into the model input while the start service is enabled.
constexpr float start_service_ego_velocity_mps = 1.0F;

std::string compute_file_hash_hex(const std::string & path)
{
  constexpr std::size_t HASH_READ_BUFFER_BYTES = 64 * 1024;
  std::ifstream ifs(path, std::ios::binary);
  if (!ifs) {
    return "<failed to open>";
  }
  std::array<char, HASH_READ_BUFFER_BYTES> buffer{};
  std::size_t combined = 0;
  std::hash<std::string_view> hasher;
  while (ifs) {
    ifs.read(buffer.data(), buffer.size());
    const std::streamsize n = ifs.gcount();
    if (n <= 0) {
      break;
    }
    const std::size_t chunk = hasher(std::string_view(buffer.data(), static_cast<std::size_t>(n)));
    // boost::hash_combine: 0x9e3779b97f4a7c15 is 2^64 / golden ratio.
    combined ^= chunk + 0x9e3779b97f4a7c15ULL + (combined << 6) + (combined >> 2);
  }
  std::ostringstream oss;
  oss << std::hex << std::setw(sizeof(std::size_t) * 2) << std::setfill('0') << combined;
  return oss.str();
}
}  // namespace

MLPlanner::MLPlanner(const rclcpp::NodeOptions & options)
: Node("ml_planner", options), generator_uuid_(autoware_utils_uuid::generate_uuid())
{
  // Initialize the node
  pub_trajectory_ = this->create_publisher<Trajectory>("~/output/trajectory", 1);
  pub_trajectories_ = this->create_publisher<CandidateTrajectories>("~/output/trajectories", 1);
  pub_objects_ =
    this->create_publisher<PredictedObjects>("~/output/predicted_objects", rclcpp::QoS(1));
  pub_route_marker_ = this->create_publisher<MarkerArray>("~/debug/route_marker", 10);
  pub_lane_marker_ = this->create_publisher<MarkerArray>("~/debug/lane_marker", 10);
  pub_linestring_marker_ = this->create_publisher<MarkerArray>("~/debug/linestring_marker", 10);
  pub_turn_indicators_ =
    this->create_publisher<TurnIndicatorsCommand>("~/output/turn_indicators", 1);
  pub_traffic_signal_ = this->create_publisher<autoware_perception_msgs::msg::TrafficLightGroup>(
    "~/output/debug/traffic_signal", 1);
  debug_processing_time_detail_pub_ = this->create_publisher<autoware_utils::ProcessingTimeDetail>(
    "~/debug/processing_time_detail_ms", 1);
  debug_processing_time_pub_ =
    this->create_publisher<autoware_internal_debug_msgs::msg::Float64Stamped>(
      "~/debug/processing_time_ms", 1);
  time_keeper_ = std::make_shared<autoware_utils::TimeKeeper>(debug_processing_time_detail_pub_);
  pub_inference_time_ =
    this->create_publisher<std_msgs::msg::Float64>("~/debug/inference_time_ms", 1);
  pub_raw_trajectory_ =
    this->create_publisher<Trajectory>("~/debug/optimization/raw_trajectory", 1);
  pub_optimization_status_ =
    this->create_publisher<std_msgs::msg::Int32>("~/debug/optimization/solver_status", 1);
  pub_optimization_time_ =
    this->create_publisher<std_msgs::msg::Float64>("~/debug/optimization/solve_time_ms", 1);
  pub_avoidance_trajectory_ =
    this->create_publisher<Trajectory>("~/debug/road_border_avoidance/adjusted_trajectory", 1);
  pub_avoidance_shifted_count_ = this->create_publisher<std_msgs::msg::Int32>(
    "~/debug/road_border_avoidance/shifted_point_count", 1);
  pub_pre_stop_fixing_trajectory_ =
    this->create_publisher<Trajectory>("~/debug/stop_point_fixing/unfixed_trajectory", 1);

  set_up_params();
  vehicle_info_ = autoware::vehicle_info_utils::VehicleInfoUtils(*this).getVehicleInfo();
  RCLCPP_INFO_STREAM(
    get_logger(),
    "vehicle_info: wheel_base_m=" << vehicle_info_.wheel_base_m
                                  << ", front_overhang_m=" << vehicle_info_.front_overhang_m
                                  << ", rear_overhang_m=" << vehicle_info_.rear_overhang_m
                                  << ", left_overhang_m=" << vehicle_info_.left_overhang_m
                                  << ", right_overhang_m=" << vehicle_info_.right_overhang_m
                                  << ", wheel_tread_m=" << vehicle_info_.wheel_tread_m);

  // Create core instance
  core_ = std::make_unique<MLPlannerCore>(params_, vehicle_info_);

  planning_factor_interface_ =
    std::make_unique<autoware::planning_factor_interface::PlanningFactorInterface>(
      this, "ml_planner");

  diagnostics_inference_ = std::make_unique<DiagnosticsInterface>(this, "inference_status");
  try {
    load_model();
    if (params_.build_only) {
      RCLCPP_INFO(get_logger(), "Build only mode enabled. Exiting after loading model.");
      std::exit(EXIT_SUCCESS);
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR_STREAM(get_logger(), e.what() << ". Inference will be disabled.");
    diagnostics_inference_->update_level_and_message(DiagnosticStatus::ERROR, e.what());
    diagnostics_inference_->publish(get_clock()->now());
    if (params_.build_only) {
      RCLCPP_ERROR(get_logger(), "Build only mode: exiting due to model load failure.");
      std::exit(EXIT_FAILURE);
    }
  }

  timer_ = rclcpp::create_timer(
    this, get_clock(), rclcpp::Rate(params_.planning_frequency_hz).period(),
    std::bind(&MLPlanner::on_timer, this));

  sub_map_ = create_subscription<HADMapBin>(
    "~/input/vector_map", rclcpp::QoS{1}.transient_local(),
    std::bind(&MLPlanner::on_map, this, std::placeholders::_1));

  // Parameter Callback
  set_param_res_ = add_on_set_parameters_callback(
    std::bind(&MLPlanner::on_parameter, this, std::placeholders::_1));

  srv_start_ = this->create_service<std_srvs::srv::SetBool>(
    "~/service/start",
    std::bind(&MLPlanner::on_start_service, this, std::placeholders::_1, std::placeholders::_2));
}

void MLPlanner::on_start_service(
  const std_srvs::srv::SetBool::Request::SharedPtr request,
  const std_srvs::srv::SetBool::Response::SharedPtr response)
{
  start_velocity_override_enabled_ = request->data;
  RCLCPP_INFO(
    get_logger(), "Start service: ego velocity override %s",
    start_velocity_override_enabled_ ? "enabled (1 m/s)" : "disabled");
  response->success = true;
  response->message = start_velocity_override_enabled_ ? "ego velocity override enabled (1 m/s)"
                                                       : "ego velocity override disabled";
}

void MLPlanner::set_up_params()
{
  // node params
  params_.model_path = this->declare_parameter<std::string>("model.onnx_model_path", "");
  params_.backend = this->declare_parameter<std::string>("model.backend", "tensorrt");
  params_.trt_precision = this->declare_parameter<std::string>("model.precision", "fp32");
  params_.use_cuda_graph = this->declare_parameter<bool>("model.use_cuda_graph", true);
  params_.plugins_path = this->declare_parameter<std::string>("plugins_path", "");
  params_.build_only = this->declare_parameter<bool>("build_only", false);
  params_.planning_frequency_hz = this->declare_parameter<double>("planning_frequency_hz", 10.0);
  params_.traffic_light_group_msg_timeout_seconds =
    this->declare_parameter<double>("traffic_light_group_msg_timeout_seconds", 0.2);
  params_.batch_size = this->declare_parameter<int>("batch_size", 1);
  params_.noise_scale_list = this->declare_parameter<std::vector<double>>("noise_scale", {1.0});
  params_.line_string_max_step_m = this->declare_parameter<double>("line_string_max_step_m", 5.0);

  // trajectory optimization params (static; changing them requires a restart)
  auto & opt = params_.trajectory_optimization;
  opt.enable = this->declare_parameter<bool>("trajectory_optimization.enable", false);
  opt.weight_longitudinal =
    this->declare_parameter<double>("trajectory_optimization.weight_longitudinal", 0.5);
  opt.weight_lateral =
    this->declare_parameter<double>("trajectory_optimization.weight_lateral", 0.5);
  opt.weight_yaw = this->declare_parameter<double>("trajectory_optimization.weight_yaw", 0.05);
  opt.weight_acceleration =
    this->declare_parameter<double>("trajectory_optimization.weight_acceleration", 0.1);
  opt.weight_steering_rate =
    this->declare_parameter<double>("trajectory_optimization.weight_steering_rate", 10.0);
  opt.terminal_weight_scale =
    this->declare_parameter<double>("trajectory_optimization.terminal_weight_scale", 2.5);
  opt.min_velocity_mps =
    this->declare_parameter<double>("trajectory_optimization.min_velocity_mps", 0.0);
  opt.max_velocity_mps =
    this->declare_parameter<double>("trajectory_optimization.max_velocity_mps", 30.0);
  opt.min_acceleration_mps2 =
    this->declare_parameter<double>("trajectory_optimization.min_acceleration_mps2", -4.0);
  opt.max_acceleration_mps2 =
    this->declare_parameter<double>("trajectory_optimization.max_acceleration_mps2", 3.0);
  opt.max_steering_rate_rps =
    this->declare_parameter<double>("trajectory_optimization.max_steering_rate_rps", 1.0);
  opt.max_lateral_acceleration_mps2 =
    this->declare_parameter<double>("trajectory_optimization.max_lateral_acceleration_mps2", 3.0);
  opt.max_sqp_iterations =
    this->declare_parameter<int>("trajectory_optimization.max_sqp_iterations", 50);
#ifndef AUTOWARE_ML_PLANNER_USE_ACADOS
  if (opt.enable) {
    RCLCPP_WARN(
      get_logger(),
      "trajectory_optimization.enable is true but this build has no acados support; "
      "trajectory optimization is disabled.");
    opt.enable = false;
  }
#endif

  // road border avoidance params (static; changing them requires a restart)
  auto & avoidance = params_.road_border_avoidance;
  avoidance.enable = this->declare_parameter<bool>("road_border_avoidance.enable", false);
  avoidance.footprint_margin_m =
    this->declare_parameter<double>("road_border_avoidance.footprint_margin_m", 0.2);
  avoidance.search_radius_m =
    this->declare_parameter<double>("road_border_avoidance.search_radius_m", 120.0);
  avoidance.shift_step_m =
    this->declare_parameter<double>("road_border_avoidance.shift_step_m", 0.1);
  avoidance.max_lateral_shift_m =
    this->declare_parameter<double>("road_border_avoidance.max_lateral_shift_m", 1.5);
  avoidance.propagate_shift =
    this->declare_parameter<bool>("road_border_avoidance.propagate_shift", true);

  // stop point fixing params (static; changing them requires a restart)
  auto & stop_fixing = params_.stop_point_fixing;
  stop_fixing.enable = this->declare_parameter<bool>("stop_point_fixing.enable", false);
  stop_fixing.velocity_threshold_mps =
    this->declare_parameter<double>("stop_point_fixing.velocity_threshold_mps", 0.3);
  stop_fixing.min_deceleration_duration_sec =
    this->declare_parameter<double>("stop_point_fixing.min_deceleration_duration_sec", 1.0);

  // planning factor params
  planning_factor_params_.enable_stop =
    this->declare_parameter<bool>("planning_factor.enable_stop", false);
  planning_factor_params_.enable_slowdown =
    this->declare_parameter<bool>("planning_factor.enable_slowdown", false);
  planning_factor_params_.detection_config.stop_velocity_threshold =
    this->declare_parameter<double>("planning_factor.stop_velocity_threshold", 0.1);
  planning_factor_params_.detection_config.stop_keep_duration_threshold =
    this->declare_parameter<double>("planning_factor.stop_keep_duration_threshold", 1.0);
  planning_factor_params_.detection_config.slowdown_accel_threshold =
    this->declare_parameter<double>("planning_factor.slowdown_accel_threshold", -0.3);

  // debug params
  debug_params_.publish_debug_map =
    this->declare_parameter<bool>("debug_params.publish_debug_map", false);
  debug_params_.publish_debug_route =
    this->declare_parameter<bool>("debug_params.publish_debug_route", true);
  debug_params_.publish_debug_linestrings =
    this->declare_parameter<bool>("debug_params.publish_debug_linestrings", true);
}

void MLPlanner::load_model()
{
  diagnostics_inference_->update_level_and_message(DiagnosticStatus::WARN, "Loading model");
  diagnostics_inference_->publish(get_clock()->now());
  core_->load_model();
  diagnostics_inference_->update_level_and_message(DiagnosticStatus::OK, "Model loaded");
  diagnostics_inference_->publish(get_clock()->now());

  RCLCPP_INFO_STREAM(
    get_logger(), "Loaded model.onnx_model_path=" << params_.model_path << " (hash="
                                                  << compute_file_hash_hex(params_.model_path)
                                                  << ")");
}

SetParametersResult MLPlanner::on_parameter(
  [[maybe_unused]] const std::vector<rclcpp::Parameter> & parameters)
{
  using autoware_utils::update_param;
  {
    MLPlannerParams temp_params = params_;
    const auto previous_model_path = params_.model_path;
    const auto previous_batch_size = params_.batch_size;
    const auto previous_backend = params_.backend;
    const auto previous_trt_precision = params_.trt_precision;
    const auto previous_use_cuda_graph = params_.use_cuda_graph;
    const auto previous_line_string_max_step_m = params_.line_string_max_step_m;
    update_param<std::string>(parameters, "model.onnx_model_path", temp_params.model_path);
    update_param<std::string>(parameters, "model.backend", temp_params.backend);
    update_param<std::string>(parameters, "model.precision", temp_params.trt_precision);
    update_param<bool>(parameters, "model.use_cuda_graph", temp_params.use_cuda_graph);
    update_param<double>(
      parameters, "traffic_light_group_msg_timeout_seconds",
      temp_params.traffic_light_group_msg_timeout_seconds);
    update_param<int>(parameters, "batch_size", temp_params.batch_size);
    update_param<std::vector<double>>(parameters, "noise_scale", temp_params.noise_scale_list);
    update_param<double>(parameters, "line_string_max_step_m", temp_params.line_string_max_step_m);
    if (temp_params.trt_precision != "fp32" && temp_params.trt_precision != "fp16") {
      SetParametersResult result;
      result.successful = false;
      result.reason = "model.precision must be either 'fp32' or 'fp16'";
      return result;
    }
    const bool valid_backend = temp_params.backend == "tensorrt"
#ifdef AUTOWARE_ML_PLANNER_USE_ONNXRUNTIME
                               || temp_params.backend == "ort_cpu" ||
                               temp_params.backend == "ort_cuda" ||
                               temp_params.backend == "ort_tensorrt"
#endif
      ;
    if (!valid_backend) {
      SetParametersResult result;
      result.successful = false;
      result.reason = "model.backend must be 'tensorrt'";
#ifdef AUTOWARE_ML_PLANNER_USE_ONNXRUNTIME
      result.reason += ", 'ort_cpu', 'ort_cuda', or 'ort_tensorrt'";
#else
      result.reason += "; ONNX Runtime support is not available in this build";
#endif
      return result;
    }
    if (
      temp_params.batch_size < 1 || temp_params.batch_size > 2 ||
      temp_params.noise_scale_list.size() != static_cast<size_t>(temp_params.batch_size)) {
      SetParametersResult result;
      result.successful = false;
      result.reason =
        "batch_size must be 1 or 2 and noise_scale must contain exactly batch_size values";
      return result;
    }
    const bool model_path_changed = temp_params.model_path != previous_model_path;
    const bool batch_size_changed = temp_params.batch_size != previous_batch_size;
    const bool backend_changed = temp_params.backend != previous_backend;
    const bool trt_config_changed = temp_params.trt_precision != previous_trt_precision ||
                                    temp_params.use_cuda_graph != previous_use_cuda_graph;
    const bool line_string_max_step_changed =
      temp_params.line_string_max_step_m != previous_line_string_max_step_m;
    params_ = temp_params;
    core_->update_params(params_);

    if (model_path_changed || batch_size_changed || backend_changed || trt_config_changed) {
      try {
        load_model();
      } catch (const std::exception & e) {
        RCLCPP_ERROR_STREAM(get_logger(), e.what() << ". Failed to reload model.");
        SetParametersResult result;
        result.successful = false;
        result.reason = e.what();
        return result;
      }
    }

    if (line_string_max_step_changed && lanelet_map_ptr_) {
      core_->set_map(lanelet_map_ptr_);
    }
  }

  {
    MLPlannerDebugParams temp_debug_params = debug_params_;
    update_param<bool>(
      parameters, "debug_params.publish_debug_map", temp_debug_params.publish_debug_map);
    update_param<bool>(
      parameters, "debug_params.publish_debug_route", temp_debug_params.publish_debug_route);
    update_param<bool>(
      parameters, "debug_params.publish_debug_linestrings",
      temp_debug_params.publish_debug_linestrings);
    debug_params_ = temp_debug_params;
  }

  SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  return result;
}

void MLPlanner::publish_first_traffic_light_on_route() const
{
  const auto msg = core_->get_first_traffic_light_on_route();
  pub_traffic_signal_->publish(msg);
}

void MLPlanner::publish_debug_markers(
  const TensorMap & input_data_map, const Eigen::Matrix4d & ego_to_map_transform,
  const rclcpp::Time & timestamp) const
{
  if (debug_params_.publish_debug_route) {
    auto lifetime = rclcpp::Duration::from_seconds(0.2);
    auto route_markers = utils::create_lane_marker(
      ego_to_map_transform, input_data_map.at("route_lanes"),
      std::vector<int64_t>(ROUTE_LANES_SHAPE.begin(), ROUTE_LANES_SHAPE.end()), timestamp, lifetime,
      {0.8, 0.8, 0.8, 0.8}, "map");
    pub_route_marker_->publish(route_markers);
  }

  if (debug_params_.publish_debug_map) {
    auto lifetime = rclcpp::Duration::from_seconds(0.2);
    auto lane_markers = utils::create_lane_marker(
      ego_to_map_transform, input_data_map.at("lanes"),
      std::vector<int64_t>(LANES_SHAPE.begin(), LANES_SHAPE.end()), timestamp, lifetime,
      {0.1, 0.1, 0.7, 0.8}, "map");
    pub_lane_marker_->publish(lane_markers);
  }

  if (debug_params_.publish_debug_linestrings) {
    auto lifetime = rclcpp::Duration::from_seconds(0.2);
    auto map_markers = utils::create_map_polyline_marker(
      ego_to_map_transform, input_data_map.at("stop_lines"),
      std::vector<int64_t>(STOP_LINES_SHAPE.begin(), STOP_LINES_SHAPE.end()), timestamp, lifetime,
      {1.0f, 0.65f, 0.0f, 0.8f}, "stop_line", "map");
    auto road_border_markers = utils::create_map_polyline_marker(
      ego_to_map_transform, input_data_map.at("road_borders"),
      std::vector<int64_t>(ROAD_BORDERS_SHAPE.begin(), ROAD_BORDERS_SHAPE.end()), timestamp,
      lifetime, {0.8f, 0.0f, 0.2f, 0.8f}, "road_border", "map");
    map_markers.markers.insert(
      map_markers.markers.end(), road_border_markers.markers.begin(),
      road_border_markers.markers.end());
    pub_linestring_marker_->publish(map_markers);
  }
}

void MLPlanner::on_timer()
{
  // Timer callback function
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);
  stop_watch_ptr_ = std::make_unique<autoware_utils_system::StopWatch<std::chrono::milliseconds>>();
  stop_watch_ptr_->tic("processing_time");

  diagnostics_inference_->clear();

  const rclcpp::Time current_time(get_clock()->now());
  if (!core_->is_model_loaded()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *this->get_clock(), constants::LOG_THROTTLE_INTERVAL_MS,
      "Model not loaded. Inference is disabled. Check model.* parameters.");
    diagnostics_inference_->update_level_and_message(DiagnosticStatus::ERROR, "Model not loaded");
    diagnostics_inference_->publish(current_time);
    return;
  }

  if (!core_->is_map_loaded()) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *this->get_clock(), constants::LOG_THROTTLE_INTERVAL_MS,
      "Waiting for map data...");
    diagnostics_inference_->update_level_and_message(DiagnosticStatus::WARN, "Map data not loaded");
    diagnostics_inference_->publish(current_time);
    return;
  }

  // Take data from subscribers
  auto objects = sub_tracked_objects_.take_data();
  auto ego_kinematic_state = sub_current_odometry_.take_data();
  auto traffic_signals = sub_traffic_signals_.take_data();
  auto temp_route_ptr = route_subscriber_.take_data();
  auto turn_indicators_ptr = sub_turn_indicators_.take_data();
  auto steering_ptr = sub_steering_.take_data();

  if (!steering_ptr) {
    constexpr auto message = "Steering status is not available";
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *this->get_clock(), constants::LOG_THROTTLE_INTERVAL_MS, "%s", message);
    diagnostics_inference_->update_level_and_message(DiagnosticStatus::ERROR, message);
    diagnostics_inference_->publish(current_time);
    return;
  }

  if (traffic_signals.empty()) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), constants::LOG_THROTTLE_INTERVAL_MS,
      "no traffic signal received. traffic light info will not be updated");
  }

  auto buffer_result = core_->update_buffer(
    ego_kinematic_state, objects, traffic_signals, turn_indicators_ptr, temp_route_ptr);
  if (!buffer_result) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *this->get_clock(), constants::LOG_THROTTLE_INTERVAL_MS,
      "Failed to update input buffers: %s", buffer_result.error().c_str());
    diagnostics_inference_->update_level_and_message(DiagnosticStatus::WARN, buffer_result.error());
    diagnostics_inference_->publish(current_time);
    return;
  }

  auto input_data_result = core_->create_input_data(buffer_result.value());
  if (!input_data_result) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *this->get_clock(), constants::LOG_THROTTLE_INTERVAL_MS,
      "Failed to create input data: %s", input_data_result.error().c_str());
    diagnostics_inference_->update_level_and_message(
      DiagnosticStatus::WARN, input_data_result.error());
    diagnostics_inference_->publish(current_time);
    return;
  }
  TensorMap input_data_map = std::move(input_data_result.value());
  const rclcpp::Time frame_time = core_->frame_time();

  if (start_velocity_override_enabled_) {
    // Make the model plan as if the vehicle were already moving: overwrite every ego
    // velocity entry of the input (all history timesteps, all batches) with 1 m/s.
    auto & ego_agent_past = input_data_map.at("ego_agent_past");
    xt::view(ego_agent_past, xt::all(), xt::all(), EGO_AGENT_PAST_IDX_VELOCITY) =
      start_service_ego_velocity_mps;
  }

  const Eigen::Matrix4d ego_to_map_transform = utils::pose_to_matrix4d(core_->ego_pose());
  publish_debug_markers(input_data_map, ego_to_map_transform, frame_time);

  publish_first_traffic_light_on_route();

  // Calculate and record metrics for diagnostics using core
  diagnostics_inference_->add_key_value(
    "valid_lane_count", core_->count_valid_elements(input_data_map, "lanes"));
  diagnostics_inference_->add_key_value(
    "valid_route_count", core_->count_valid_elements(input_data_map, "route_lanes"));
  diagnostics_inference_->add_key_value(
    "valid_intersection_area_count",
    core_->count_valid_elements(input_data_map, "intersection_area"));
  diagnostics_inference_->add_key_value(
    "valid_stop_line_count", core_->count_valid_elements(input_data_map, "stop_lines"));
  diagnostics_inference_->add_key_value(
    "valid_road_border_count", core_->count_valid_elements(input_data_map, "road_borders"));
  diagnostics_inference_->add_key_value(
    "valid_neighbor_count", core_->count_valid_elements(input_data_map, "neighbor_agents_past"));

  // normalization of data
  preprocess::normalize_input_data(input_data_map);
  if (!utils::check_input_map(input_data_map)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *this->get_clock(), constants::LOG_THROTTLE_INTERVAL_MS,
      "Input data contains invalid values");
    diagnostics_inference_->update_level_and_message(
      DiagnosticStatus::WARN, "Input data contains invalid values");
    diagnostics_inference_->publish(current_time);
    return;
  }

  // Run inference using core
  auto inference_result = core_->run_inference(input_data_map);

  if (!inference_result) {
    RCLCPP_WARN_STREAM_THROTTLE(
      get_logger(), *this->get_clock(), constants::LOG_THROTTLE_INTERVAL_MS,
      "Inference failed: " << inference_result.error());
    diagnostics_inference_->update_level_and_message(
      DiagnosticStatus::ERROR, inference_result.error());
    diagnostics_inference_->publish(frame_time);
    return;
  }

  std_msgs::msg::Float64 inference_time_msg;
  inference_time_msg.data = inference_result->inference_time_ms;
  pub_inference_time_->publish(inference_time_msg);

  const double current_steering_angle_rad = static_cast<double>(steering_ptr->steering_tire_angle);

  PlannerOutput planner_output;
  try {
    planner_output = core_->create_planner_output(
      *inference_result, frame_time, generator_uuid_, current_steering_angle_rad);
  } catch (const std::exception & e) {
    RCLCPP_ERROR_STREAM(get_logger(), "Postprocessing failed: " << e.what());
    diagnostics_inference_->update_level_and_message(DiagnosticStatus::ERROR, e.what());
    diagnostics_inference_->publish(frame_time);
    return;
  }

  pub_trajectory_->publish(planner_output.trajectory);
  pub_trajectories_->publish(planner_output.candidate_trajectories);
  pub_objects_->publish(planner_output.predicted_objects);
  pub_turn_indicators_->publish(planner_output.turn_indicators_command);

  const auto & avoidance_debug = planner_output.avoidance_debug;
  if (avoidance_debug.active) {
    if (planner_output.avoidance_adjusted_trajectory) {
      pub_avoidance_trajectory_->publish(*planner_output.avoidance_adjusted_trajectory);
    }
    std_msgs::msg::Int32 shifted_count_msg;
    shifted_count_msg.data = avoidance_debug.shifted_points + avoidance_debug.unresolved_points;
    pub_avoidance_shifted_count_->publish(shifted_count_msg);
    if (avoidance_debug.unresolved_points > 0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *this->get_clock(), constants::LOG_THROTTLE_INTERVAL_MS,
        "Road border avoidance could not clear %d trajectory points within the maximum "
        "lateral shift.",
        avoidance_debug.unresolved_points);
    }
  }

  const auto & optimization_debug = planner_output.optimization_debug;
  if (optimization_debug.attempted || avoidance_debug.active) {
    if (planner_output.raw_trajectory) {
      pub_raw_trajectory_->publish(*planner_output.raw_trajectory);
    }
  }
  if (planner_output.pre_stop_fixing_trajectory) {
    pub_pre_stop_fixing_trajectory_->publish(*planner_output.pre_stop_fixing_trajectory);
  }
  if (optimization_debug.attempted) {
    std_msgs::msg::Int32 status_msg;
    status_msg.data = optimization_debug.solver_status;
    pub_optimization_status_->publish(status_msg);
    std_msgs::msg::Float64 solve_time_msg;
    solve_time_msg.data = optimization_debug.solve_time_ms;
    pub_optimization_time_->publish(solve_time_msg);
    if (!optimization_debug.optimized) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *this->get_clock(), constants::LOG_THROTTLE_INTERVAL_MS,
        "Trajectory optimization failed (acados status %d); publishing the raw trajectory.",
        optimization_debug.solver_status);
      diagnostics_inference_->update_level_and_message(
        DiagnosticStatus::WARN, "Trajectory optimization failed");
    }
  }

  publish_planning_factor(planner_output.trajectory);

  // Publish diagnostics
  diagnostics_inference_->publish(frame_time);
  autoware_internal_debug_msgs::msg::Float64Stamped processing_time_msg;
  processing_time_msg.stamp = get_clock()->now();
  processing_time_msg.data = stop_watch_ptr_->toc("processing_time", true);
  debug_processing_time_pub_->publish(processing_time_msg);
}

void MLPlanner::publish_planning_factor(const Trajectory & trajectory)
{
  const auto & points = trajectory.points;
  const auto detection_result =
    detect_planning_factors(points, planning_factor_params_.detection_config);

  if (planning_factor_params_.enable_stop && detection_result.stop) {
    const auto & stop = *detection_result.stop;
    planning_factor_interface_->add(
      points, stop.ego_pose, stop.stop_pose, PlanningFactor::STOP,
      autoware_internal_planning_msgs::msg::SafetyFactorArray{});
  }

  if (planning_factor_params_.enable_slowdown && detection_result.slowdown) {
    const auto & slowdown = *detection_result.slowdown;
    planning_factor_interface_->add(
      points, slowdown.ego_pose, slowdown.start_pose, slowdown.end_pose, PlanningFactor::SLOW_DOWN,
      autoware_internal_planning_msgs::msg::SafetyFactorArray{}, true, slowdown.start_velocity,
      slowdown.end_velocity);
  }

  planning_factor_interface_->publish();
}

void MLPlanner::on_map(const HADMapBin::ConstSharedPtr map_msg)
{
  lanelet_map_ptr_ = autoware::experimental::lanelet2_utils::from_autoware_map_msgs(*map_msg);
  core_->set_map(lanelet_map_ptr_);
}

}  // namespace autoware::ml_planner
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::ml_planner::MLPlanner)
