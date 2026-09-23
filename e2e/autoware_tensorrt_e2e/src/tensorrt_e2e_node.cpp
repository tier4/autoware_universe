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
#include "autoware/tensorrt_e2e/deployment_manifest.hpp"
#include <autoware_utils_geometry/geometry.hpp>

#include "autoware/tensorrt_e2e/input_provider_registry.hpp"

#include <autoware/diffusion_planner/postprocessing/postprocessing_utils.hpp>
#include <autoware/diffusion_planner/preprocessing/preprocessing_utils.hpp>
#include <autoware/diffusion_planner/utils/utils.hpp>
#include <autoware_utils_uuid/uuid_helper.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
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
    latch_status(DiagnosticStatus::ERROR, std::string(e.what()) + " -- inference is disabled");
    if (params_.build_only) {
      RCLCPP_ERROR(get_logger(), "Build only mode: exiting due to initialization failure.");
      std::exit(EXIT_FAILURE);
    }
  }

  auto limit = [this](const char *name, double value) {
    const double configured = has_parameter(name)
                                  ? get_parameter(name).as_double()
                                  : declare_parameter<double>(name, value);
    if (!std::isfinite(configured) || configured <= 0.0) {
      throw std::runtime_error(std::string(name) +
                               " must be finite and positive");
    }
    return configured;
  };
  pose_limits_.translation_slack_m =
      limit("localization_reset.translation_slack_m", 2.0);
  pose_limits_.max_speed_mps = limit("localization_reset.max_speed_mps", 60.0);
  pose_limits_.yaw_slack_rad = limit("localization_reset.yaw_slack_rad", 0.35);
  pose_limits_.max_yaw_rate_rps =
      limit("localization_reset.max_yaw_rate_rps", 2.0);
  // These callbacks share the mutually-exclusive default callback group with
  // sensor collection. Retry only a sensor tick waiting for its ego bracket.
  sub_odometry_ = create_subscription<Odometry>(
      "~/input/odometry", rclcpp::QoS(200),
      [this](Odometry::ConstSharedPtr msg) {
        if (!odometry_history_.samples.empty()) {
          const auto &previous = odometry_history_.samples.back();
          const double dt = (rclcpp::Time(msg->header.stamp).nanoseconds() -
                             previous.stamp_ns) *
                            1e-9;
          auto pose = [](const Odometry &odom) {
            const auto matrix = dp::utils::pose_to_matrix4d(odom.pose.pose);
            return std::array<double, 4>{matrix(0, 3), matrix(1, 3),
                                         matrix(0, 0), matrix(1, 0)};
          };
          if (dt < 0 || pose_discontinuous(pose(previous.value), pose(*msg), dt,
                                           pose_limits_)) {
            ++localization_generation_;
            odometry_history_.samples.clear();
            acceleration_history_.samples.clear();
            steering_history_.samples.clear();
            pipeline_latency_.clear();
            processing_latency_.clear();
            RCLCPP_WARN(get_logger(),
                        "Localization discontinuity: discarded ego history");
          }
        }
        if (odometry_history_.insert(
                rclcpp::Time(msg->header.stamp).nanoseconds(), *msg,
                ego_history_keep_ns_)) {
          acceleration_history_.samples.clear();
          steering_history_.samples.clear();
        }
        if (waiting_for_ego_)
          run_once();
      });
  sub_acceleration_ = create_subscription<AccelWithCovarianceStamped>(
      "~/input/acceleration", rclcpp::QoS(100),
      [this](AccelWithCovarianceStamped::ConstSharedPtr msg) {
        acceleration_history_.insert(
            rclcpp::Time(msg->header.stamp).nanoseconds(), *msg,
            ego_history_keep_ns_);
        if (waiting_for_ego_)
          run_once();
      });

  if (recorded_ego_dynamics_) {
    sub_steering_ = create_subscription<SteeringReport>(
        "~/input/steering", rclcpp::QoS(100),
        [this](SteeringReport::ConstSharedPtr msg) {
          steering_history_.insert(rclcpp::Time(msg->stamp).nanoseconds(), *msg,
                                   ego_history_keep_ns_);
          if (waiting_for_ego_)
            run_once();
        });
  }
  if (engine_) {
    if (const auto *spec =
            find_spec(engine_->input_specs(), "ego_agent_past")) {
      // The context preprocessor samples at the deployed 0.1 s contract.
      ego_history_keep_ns_ = std::max(
          ego_history_keep_ns_, (spec->shape[1] + 10) * int64_t{100000000});
    }
  }

  wire_pacing();

  // No runtime condition throws out of this constructor any more. A component
  // constructor that throws is never loaded: launch_ros reports "Component constructor
  // threw an exception" and the node is simply absent, with the actual reason -- an
  // engine that failed to build, a missing artifact -- left somewhere further up a
  // shared container's log, which is how the pipeline failure below used to surface as
  // the pacing check firing on an empty provider list. Those reasons are reported on
  // `inference_status` instead, by a loaded node that keeps saying them. (A malformed
  // deployment still throws where rclcpp throws it -- an undeclared vehicle_info or
  // ml_package parameter -- exactly as autoware_bevfusion's read-only parameters do.)
  //
  // Waiting is not one of those reasons. At start-up this node is regularly composed
  // and running before the LiDAR pipeline that paces it exists, and the correct
  // behaviour is to wait: autoware_bevfusion, reading the same cloud, warns and skips
  // for as long as its inputs are missing and never takes itself or the container down.
  status_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  status_timer_ = create_wall_timer(
    std::chrono::milliseconds(1000), [this]() { report_status(); }, status_callback_group_);
}

void TensorrtE2eNode::wire_pacing()
{
  if (!pipeline_ready_) {
    // initialize_pipeline() failed and said why. There is no engine to run, so pacing a
    // tick off the sensor would only replace that message with a vaguer one every frame.
    return;
  }

  // Run when the sensor the model waits on delivers, rather than on a tick that
  // may land just before it: a timer makes each frame wait up to a full period
  // to be used, and the controller pays that as latency for an input that had
  // already arrived. autoware_bevfusion, reading the same cloud, is driven this
  // way and carries no timer at all.
  //
  // The callback is serialised with itself by its callback group, so a run
  // cannot re-enter; a frame arriving during one is simply the next run's, which
  // is what planning on the newest sample means.
  for (const auto & provider : providers_) {
    if (provider->pace([this]() { run_once(); })) {
      RCLCPP_INFO(
        get_logger(), "Paced by '%s': planning runs when its input arrives",
        provider->name().c_str());
      if (!pacing_provider_) {
        pacing_provider_ = provider.get();
      }
    }
  }
  if (!pacing_provider_) {
    // There is no second way to run. A timer here would plan on whatever the
    // last sensor sample happened to be, at a rate unrelated to it, and publish
    // a trajectory that looks exactly like a fresh one -- which is worse than
    // not running, because nothing downstream can tell the difference.
    const std::string message =
      "No input provider paces this model: nothing would ever trigger planning. "
      "A model this node can run has to consume a sensor (set sensor_inputs).";
    RCLCPP_ERROR(get_logger(), "%s", message.c_str());
    latch_status(DiagnosticStatus::ERROR, message);
  }
}

void TensorrtE2eNode::latch_status(const int8_t level, const std::string & message)
{
  latched_level_ = level;
  latched_message_ = message;
  diagnostics_->clear();
  diagnostics_->update_level_and_message(level, message);
  diagnostics_->publish(get_clock()->now());
}

void TensorrtE2eNode::report_status()
{
  std::unique_lock<std::mutex> lock(tick_mutex_, std::try_to_lock);
  if (!lock.owns_lock()) {
    // A tick holds it: the node is alive and is publishing its own status. Racing it to
    // the same diagnostic would only interleave two messages on one status object.
    return;
  }

  const rclcpp::Time now = get_clock()->now();

  // What each sensor provider's own subscription has delivered, on every status this
  // timer publishes. On the vehicle the node was found loaded, silent and not planning,
  // and nothing said whether the cloud was reaching it at all. Now the answer is on the
  // diagnostic: `bev_feature.received: 0` is a subscription problem, a count that keeps
  // climbing under an ERROR is this node's.
  const auto add_received_counts = [this]() {
    for (const auto & provider : providers_) {
      if (const auto received = provider->received_count()) {
        diagnostics_->add_key_value(
          provider->name() + ".received", static_cast<int64_t>(*received));
      }
    }
  };

  if (!latched_message_.empty()) {
    // Republished rather than said once at construction: a status that stops arriving is
    // reported as stale by the aggregator, which reads as "gone", not as "broken, here is
    // why". This is the only thing a node whose pipeline failed still does. It goes to
    // the log as well, throttled, because one ERROR line at load time is invisible in a
    // container log that keeps scrolling for the rest of the drive.
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), LOG_THROTTLE_INTERVAL_MS, "%s", latched_message_.c_str());
    diagnostics_->clear();
    diagnostics_->update_level_and_message(latched_level_, latched_message_);
    add_received_counts();
    diagnostics_->publish(now);
    return;
  }

  const double silent_seconds = last_tick_ ? (now - *last_tick_).seconds() : -1.0;
  if (silent_seconds >= 0.0 && silent_seconds < params_.input_timeout_seconds) {
    return;  // the sensor is delivering; each tick publishes its own status
  }

  const std::string source = pacing_provider_ ? pacing_provider_->name() : std::string("sensor");
  const std::string message =
    !last_tick_ ? "Waiting for the first input from '" + source + "'"
                : "No input from '" + source + "' for " +
                    std::to_string(static_cast<int64_t>(silent_seconds * 1e3)) + " ms";
  RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), LOG_THROTTLE_INTERVAL_MS, "%s", message.c_str());
  diagnostics_->clear();
  diagnostics_->update_level_and_message(DiagnosticStatus::WARN, message);
  add_received_counts();
  diagnostics_->publish(now);
}

void TensorrtE2eNode::set_up_params()
{
  recorded_ego_dynamics_ =
      declare_parameter<bool>("recorded_ego_dynamics", false);
  declare_parameter<bool>("require_deployment_manifest", false);
  params_.model_path = declare_parameter<std::string>("model_path", "");
  params_.plugins_path = declare_parameter<std::string>("plugins_path", "");
  params_.precision = declare_parameter<std::string>("precision", "fp16");
  params_.trt_workspace_mib = declare_parameter<int64_t>("trt_workspace_mib", 4096);
  params_.args_path = declare_parameter<std::string>("args_path", "");
  params_.build_only = declare_parameter<bool>("build_only", false);
  params_.dump_dir = declare_parameter<std::string>("debug.dump_dir", "");
  params_.dump_max_frames = declare_parameter<int64_t>("debug.dump_max_frames", 300);
  params_.shift_x = declare_parameter<bool>("shift_x", false);
  params_.sensor_inputs =
    declare_parameter<std::vector<std::string>>("sensor_inputs", std::vector<std::string>{});
  params_.enable_context_inputs = declare_parameter<bool>("enable_context_inputs", true);
  params_.input_timeout_seconds = declare_parameter<double>("input_timeout_seconds", 1.0);

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

  planning_factor_params_.enable_stop =
    declare_parameter<bool>("planning_factor.enable_stop", false);
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
  const bool any_tf = std::any_of(providers_.begin(), providers_.end(), [](const auto & provider) {
    return provider->uses_tf();
  });
  if (any_tf) {
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(tf_buffer_);
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

  create_providers();
  if (get_parameter("require_deployment_manifest").as_bool()) {
    const auto graph = std::filesystem::path(params_.model_path);
    const auto manifest = validate_deployment_manifest(
        graph.parent_path() / "deployment_manifest.json");
    if (graph.filename().string() !=
            manifest.at("planner_file").get<std::string>() ||
        std::filesystem::canonical(
            get_parameter("bev_feature.extractor.onnx_path").as_string()) !=
            std::filesystem::canonical(
                graph.parent_path() /
                manifest.at("extractor_file").get<std::string>()))
      throw std::runtime_error(
          "Configured ONNX paths do not match deployment manifest");
    for (const auto &[name, expected] : manifest.at("parameters").items()) {
      if (!has_parameter(name))
        throw std::runtime_error("Missing contract parameter: " + name);
      const auto parameter = get_parameter(name);
      nlohmann::json actual;
      switch (parameter.get_type()) {
      case rclcpp::ParameterType::PARAMETER_BOOL:
        actual = parameter.as_bool();
        break;
      case rclcpp::ParameterType::PARAMETER_INTEGER:
        actual = parameter.as_int();
        break;
      case rclcpp::ParameterType::PARAMETER_DOUBLE:
        actual = parameter.as_double();
        break;
      case rclcpp::ParameterType::PARAMETER_STRING:
        actual = parameter.as_string();
        break;
      case rclcpp::ParameterType::PARAMETER_STRING_ARRAY:
        actual = parameter.as_string_array();
        break;
      case rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY:
        actual = parameter.as_integer_array();
        break;
      case rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY:
        actual = parameter.as_double_array();
        break;
      default:
        throw std::runtime_error("Unsupported contract parameter: " + name);
      }
      if (actual != expected)
        throw std::runtime_error("Deployment contract mismatch: " + name);
    }
  }

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

  // One stream for the whole tick. A provider's GPU work, the network, and the output copy
  // are ordered on it, so nothing in the middle of a pass has to wait for the device: the
  // single host synchronization is the one that waits for the outputs.
  for (const auto & provider : providers_) {
    provider->bind_stream(engine_->stream());
  }

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
  waiting_for_ego_ = true;
  if (odometry_history_.samples.empty())
    return std::nullopt;
  const auto sensor_stamp =
      pacing_provider_ ? pacing_provider_->latest_input_stamp() : std::nullopt;
  const int64_t target = sensor_stamp
                             ? sensor_stamp->nanoseconds()
                             : odometry_history_.samples.back().stamp_ns;
  const auto bracket = odometry_history_.bracket(target);
  if (!bracket)
    return std::nullopt; // Never extrapolate a LiDAR pose from a stale twist.
  const auto &lo = odometry_history_.samples[bracket->first];
  const auto &hi = odometry_history_.samples[bracket->second];
  // A missing localization interval must not be bridged across a pose jump.
  if (hi.stamp_ns - lo.stamp_ns > 200000000LL)
    return std::nullopt;
  const double alpha = hi.stamp_ns == lo.stamp_ns
                           ? 0.0
                           : static_cast<double>(target - lo.stamp_ns) /
                                 (hi.stamp_ns - lo.stamp_ns);

  EgoFrame ego;
  ego.localization_generation = localization_generation_;
  ego.odometry = lo.value;
  ego.stamp = rclcpp::Time(target, get_clock()->get_clock_type());
  ego.odometry.header.stamp = ego.stamp;
  ego.odometry.pose.pose = autoware_utils_geometry::calc_interpolated_pose(
      lo.value.pose.pose, hi.value.pose.pose, alpha, false);
  auto blend = [alpha](double a, double b) { return a + alpha * (b - a); };
  ego.odometry.twist.twist.linear.x =
      blend(lo.value.twist.twist.linear.x, hi.value.twist.twist.linear.x);
  ego.odometry.twist.twist.linear.y =
      blend(lo.value.twist.twist.linear.y, hi.value.twist.twist.linear.y);
  ego.odometry.twist.twist.angular.z =
      blend(lo.value.twist.twist.angular.z, hi.value.twist.twist.angular.z);
  if (const auto *accel =
          acceleration_history_.at_or_before(target, 200000000LL)) {
    ego.acceleration = *accel;
  }
  if (const auto *steer = steering_history_.at_or_before(target, 200000000LL)) {
    ego.steering_angle = steer->steering_tire_angle;
  }
  if (find_spec(engine_->input_specs(), "ego_current_state") &&
      (!ego.acceleration || (recorded_ego_dynamics_ && !ego.steering_angle)))
    return std::nullopt;
  ego.reference_odometry = ego.odometry;
  if (params_.shift_x) {
    ego.reference_odometry.pose.pose =
      dp::utils::shift_x(ego.odometry.pose.pose, base_link_to_center_);
  }
  for (const auto &sample : odometry_history_.samples) {
    auto odom = sample.value;
  if (params_.shift_x)
      odom.pose.pose = dp::utils::shift_x(odom.pose.pose, base_link_to_center_);
    ego.reference_history.push_back(std::move(odom));
  }
  ego.ego_to_map = dp::utils::pose_to_matrix4d(ego.reference_odometry.pose.pose);
  ego.map_to_ego = dp::utils::inverse(ego.ego_to_map);
  waiting_for_ego_ = false;
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

void TensorrtE2eNode::run_once()
{
  if (runtime_failed_) {
    return;
  }
  // Excludes report_status(), which shares `diagnostics_`; two providers that both pace
  // would otherwise also be able to enter this from two executor threads at once.
  std::lock_guard<std::mutex> lock(tick_mutex_);
  last_tick_ = get_clock()->now();
  // Everything below throws: CHECK_CUDA_ERROR raises std::runtime_error, and so do the
  // extractor, the temporal cache and the inference engine. This runs in a subscription
  // callback, so an escaping exception does not fail this node -- it terminates the
  // process, and in the deployed configuration that process is the shared
  // /pointcloud_container, which takes the vehicle's whole CUDA sensing stack with it.
  // Catch here and go quiet instead: no trajectory, a latched ERROR diagnostic saying
  // why, and the sensing stack still running. The failure is sticky because a CUDA error
  // usually leaves the context unusable, so retrying every 100 ms would only fill the log.
  try {
    TickTiming timing;
    run_tick(timing);
    // Whatever a provider still owes runs now, whether the pass published or gave up:
    // a detection head's decode, say. It is off the trajectory's path on purpose, so the
    // consumer of the trajectory never waits for a message it does not read. It does
    // occupy this callback until it is done, which is why it is measured separately.
    stop_watch_.tic("finish");
    for (const auto & provider : providers_) {
      provider->finish_tick();
    }
    debug_publisher_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
      "debug/processing_time/finish_ms", stop_watch_.toc("finish"));
  } catch (const std::exception & e) {
    runtime_failed_ = true;
    RCLCPP_ERROR(get_logger(), "Inference failed and the planner is now disabled: %s", e.what());
    latch_status(
      DiagnosticStatus::ERROR, std::string("Inference failed, planner disabled: ") + e.what());
  }
}

void TensorrtE2eNode::run_tick(TickTiming & timing)
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
      get_logger(), *get_clock(), LOG_THROTTLE_INTERVAL_MS, "Waiting for timestamp-aligned ego state...");
    finish(DiagnosticStatus::WARN, "Waiting for timestamp-aligned ego state");
    return;
  }

  // Collect all model inputs. The pacing provider goes last: its sensor callback already
  // queued its GPU work before this pass began, so the other providers' CPU work (map
  // and route tensors, mostly) overlaps that instead of waiting behind it, and the pass
  // costs the longer of the two rather than their sum.
  stop_watch_.tic("collect");
  TensorMap inputs;
  std::vector<InputProviderInterface *> collection_order;
  collection_order.reserve(providers_.size());
  for (const auto & provider : providers_) {
    if (provider.get() != pacing_provider_) {
      collection_order.push_back(provider.get());
    }
  }
  if (pacing_provider_) {
    collection_order.push_back(pacing_provider_);
  }
  for (auto * provider : collection_order) {
    stop_watch_.tic("provider");
    std::string error;
    const bool collected = provider->collect(*ego, now, inputs, error);
    timing.provider_collect_ms.emplace_back(provider->name(), stop_watch_.toc("provider"));
    if (!collected) {
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

  // The engine already completed its output wait on the shared stream. Query
  // CUDA event durations here without adding another synchronization.
  for (auto &provider : providers_)
    provider->add_diagnostics(*diagnostics_);

  // Postprocess and publish.
  stop_watch_.tic("postprocess");
  TrajectoryPostprocessor::Output output;
  try {
    output = postprocessor_->process(*result.outputs, *ego, ego->stamp, generator_uuid_);
  } catch (const std::exception & e) {
    RCLCPP_ERROR_STREAM(get_logger(), "Postprocessing failed: " << e.what());
    finish(DiagnosticStatus::ERROR, e.what());
    return;
  }

  timing.postprocess_ms = stop_watch_.toc("postprocess");

  if (!params_.dump_dir.empty() && dumped_frames_ < params_.dump_max_frames) {
    dump_tensors(inputs, *result.outputs, *ego);
  }

  pub_trajectory_->publish(output.trajectory);
  pub_trajectories_->publish(output.candidate_trajectories);
  publish_planning_factor(output.trajectory);

  // Timing: the whole tick must fit in the planning period to sustain the output rate.
  const double processing_time_ms = stop_watch_.toc("processing_time");
  timing.total_ms = processing_time_ms;
  const auto published_at = get_clock()->now();
  debug_publisher_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
      "debug/input_age_ms", (now - ego->stamp).seconds() * 1e3);
  publish_debug_timing(published_at, *ego, timing);
  // Against the interval this run actually had, not a configured one: the pace
  // is the sensor's, and it is the pace the node has to keep up with.
  const double period_ms = previous_run_.has_value() ? (now - previous_run_.value()).seconds() * 1e3
                                                     : std::numeric_limits<double>::infinity();
  previous_run_ = now;
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
    // On the healthy status too, so a bag of a good run shows the count climbing and a
    // bag of a bad one shows where it stopped -- the same key report_status() carries
    // while nothing ticks.
    if (const auto received = provider->received_count()) {
      diagnostics_->add_key_value(provider->name() + ".received", static_cast<int64_t>(*received));
    }
  }
  // Same keys and the same counting as autoware_diffusion_planner: a [1, N, P, D] tensor's
  // valid elements are its non-zero rows in batch 0.
  static const std::vector<std::pair<const char *, const char *>> kCounted = {
    {"lanes", "valid_lane_count"},
    {"route_lanes", "valid_route_count"},
    {"polygons", "valid_polygon_count"},
    {"line_strings", "valid_line_string_count"},
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

void TensorrtE2eNode::dump_tensors(
  const TensorMap & inputs, const TensorMap & outputs, const EgoFrame & ego)
{
  namespace fs = std::filesystem;
  const fs::path dir(params_.dump_dir);
  std::error_code ec;
  fs::create_directories(dir, ec);
  if (ec) {
    RCLCPP_WARN_STREAM_THROTTLE(
      get_logger(), *get_clock(), LOG_THROTTLE_INTERVAL_MS,
      "debug.dump_dir: cannot create " << dir << ": " << ec.message());
    return;
  }
  const int64_t frame = dumped_frames_++;
  std::ostringstream manifest;
  const auto & pose = ego.odometry.pose.pose;
  const auto & twist = ego.odometry.twist.twist;
  manifest << std::setprecision(17) << "{\"frame\":" << frame << ",\"stamp\":" << ego.stamp.seconds()
           << ",\"ego\":{\"x\":" << pose.position.x << ",\"y\":" << pose.position.y
           << ",\"z\":" << pose.position.z << ",\"qx\":" << pose.orientation.x
           << ",\"qy\":" << pose.orientation.y << ",\"qz\":" << pose.orientation.z
           << ",\"qw\":" << pose.orientation.w << ",\"vx\":" << twist.linear.x
           << ",\"vy\":" << twist.linear.y << ",\"wz\":" << twist.angular.z << "}";
  const auto write_group = [&](const char * group, const TensorMap & tensors) {
    manifest << ",\"" << group << "\":{";
    bool first = true;
    for (const auto & [name, tensor] : tensors) {
      if (tensor.is_device()) {
        continue;  // a BEV feature map: hundreds of MB per frame, and not a context tensor
      }
      std::ostringstream file;
      file << std::setw(6) << std::setfill('0') << frame << "_" << group << "_" << name << ".f32";
      std::ofstream out(dir / file.str(), std::ios::binary);
      out.write(
        reinterpret_cast<const char *>(tensor.host_data.data()),
        static_cast<std::streamsize>(tensor.host_data.size() * sizeof(float)));
      manifest << (first ? "" : ",") << "\"" << name << "\":{\"file\":\"" << file.str()
               << "\",\"shape\":[";
      for (size_t i = 0; i < tensor.shape.size(); ++i) {
        manifest << (i ? "," : "") << tensor.shape[i];
      }
      manifest << "]}";
      first = false;
    }
    manifest << "}";
  };
  write_group("inputs", inputs);
  write_group("outputs", outputs);
  manifest << "}\n";
  std::ofstream(dir / "manifest.jsonl", std::ios::app) << manifest.str();
  if (frame == 0) {
    RCLCPP_INFO_STREAM(
      get_logger(), "debug.dump_dir: writing the first " << params_.dump_max_frames
                                                          << " inferences to " << dir);
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
  debug_publisher_->publish<Float64Stamped>(
    "debug/cyclic_time_ms", stop_watch_.toc("cyclic", true));
  debug_publisher_->publish<Float64Stamped>(
    "debug/pipeline_latency_ms", (now - input_stamp).seconds() * 1e3);
  pipeline_latency_.add((now - input_stamp).seconds() * 1e3);
  processing_latency_.add(timing.total_ms);
  for (const auto &entry : std::vector<std::pair<std::string, double>>{
           {"p95", 0.95}, {"p99", 0.99}}) {
    debug_publisher_->publish<Float64Stamped>(
        "debug/pipeline_latency_" + entry.first + "_ms",
        pipeline_latency_.percentile(entry.second));
    debug_publisher_->publish<Float64Stamped>(
        "debug/processing_time/" + entry.first + "_ms",
        processing_latency_.percentile(entry.second));
  }
  debug_publisher_->publish<Float64Stamped>("debug/processing_time/total_ms", timing.total_ms);
  debug_publisher_->publish<Float64Stamped>("debug/processing_time/collect_ms", timing.collect_ms);
  for (const auto & [provider, ms] : timing.provider_collect_ms) {
    debug_publisher_->publish<Float64Stamped>(
      "debug/processing_time/collect/" + provider + "_ms", ms);
  }
  debug_publisher_->publish<Float64Stamped>(
    "debug/processing_time/inference_ms", timing.inference_ms);
  debug_publisher_->publish<Float64Stamped>(
    "debug/processing_time/postprocess_ms", timing.postprocess_ms);
}

}  // namespace autoware::tensorrt_e2e

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::tensorrt_e2e::TensorrtE2eNode)
