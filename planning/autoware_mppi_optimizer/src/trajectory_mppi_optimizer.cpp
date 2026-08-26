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

#include "autoware/mppi_optimizer/trajectory_mppi_optimizer.hpp"

#include "autoware/mppi_optimizer/detail/trajectory_utils.hpp"
#include "autoware/mppi_optimizer/first_order_dubins_mppi_kinematic_limits_conversion.hpp"
#include "autoware/mppi_optimizer/first_order_dubins_mppi_vehicle_params_conversion.hpp"
#include "autoware/mppi_optimizer/first_order_dubins_mppi_vehicle_params_ros.hpp"
#include "autoware/mppi_optimizer/mppi_debug_markers.hpp"

#include <autoware_utils_debug/debug_publisher.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <autoware_internal_debug_msgs/msg/float64_stamped.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <algorithm>
#include <cmath>
#include <exception>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace autoware::mppi_optimizer::plugin
{
namespace
{

using autoware::trajectory_processor::plugin::ProcessingResult;
using autoware_utils_geometry::Segment2d;

/** @brief Converts processor parameters into MPPI cost parameters. */
FirstOrderDubinsMppiCostParams make_cost_params(const trajectory_mppi_optimizer::Params & params)
{
  if (params.obstacle_safe_margin < params.obstacle_collision_margin) {
    throw std::invalid_argument(
      "mppi_optimizer.obstacle_safe_margin must be greater than or equal to "
      "mppi_optimizer.obstacle_collision_margin");
  }
  if (params.road_border_safe_margin < params.road_border_collision_margin) {
    throw std::invalid_argument(
      "mppi_optimizer.road_border_safe_margin must be greater than or equal to "
      "mppi_optimizer.road_border_collision_margin");
  }

  FirstOrderDubinsMppiCostParams output;
  output.lambda = static_cast<float>(params.lambda);
  output.speed_coeff = static_cast<float>(params.speed_coeff);
  output.track_coeff = static_cast<float>(params.track_coeff);
  output.track_terminal_scale = static_cast<float>(params.track_terminal_scale);
  output.heading_coeff = static_cast<float>(params.heading_coeff);
  output.lateral_distance_coeff = static_cast<float>(params.lateral_distance_coeff);
  output.lateral_yaw_error_coeff = static_cast<float>(params.lateral_yaw_error_coeff);
  output.remaining_distance_coeff = static_cast<float>(params.remaining_distance_coeff);
  output.path_overshoot_coeff = static_cast<float>(params.path_overshoot_coeff);
  output.track_center_coeff = static_cast<float>(params.track_center_coeff);
  output.corner_buffer_coeff = static_cast<float>(params.corner_buffer_coeff);
  output.corner_safe_margin = static_cast<float>(params.corner_safe_margin);
  output.boundary_threshold = static_cast<float>(params.boundary_threshold);
  output.lateral_boundary_soft_margin = static_cast<float>(params.lateral_boundary_soft_margin);
  output.accel_cmd_coeff = static_cast<float>(params.accel_cmd_coeff);
  output.steer_cmd_coeff = static_cast<float>(params.steer_cmd_coeff);
  output.steer_rate_coeff = static_cast<float>(params.steer_rate_coeff);
  output.accel_cmd_std_dev = static_cast<float>(params.accel_cmd_std_dev);
  output.steer_cmd_std_dev = static_cast<float>(params.steer_cmd_std_dev);
  output.accel_cmd_noise_exponent = static_cast<float>(params.accel_cmd_noise_exponent);
  output.steer_cmd_noise_exponent = static_cast<float>(params.steer_cmd_noise_exponent);
  output.nominal_curvature_min_chord_length_m =
    static_cast<float>(params.nominal_curvature_min_chord_length_m);
  output.lateral_acceleration_coeff = static_cast<float>(params.lateral_acceleration_coeff);
  output.lateral_jerk_coeff = static_cast<float>(params.lateral_jerk_coeff);
  output.longitudinal_jerk_coeff = static_cast<float>(params.longitudinal_jerk_coeff);
  output.obstacle_collision_margin = static_cast<float>(params.obstacle_collision_margin);
  output.road_border_collision_margin = static_cast<float>(params.road_border_collision_margin);
  output.obstacle_safe_margin = static_cast<float>(params.obstacle_safe_margin);
  output.road_border_safe_margin = static_cast<float>(params.road_border_safe_margin);
  output.drivable_area_safe_margin = static_cast<float>(params.drivable_area_safe_margin);
  output.drivable_area_barrier_weight = static_cast<float>(params.drivable_area_barrier_weight);
  output.crash_contact_penalty = static_cast<float>(params.crash_contact_penalty);
  return output;
}

/** @brief Converts processor parameters into MPPI runtime options. */
FirstOrderDubinsMppiRuntimeOptions make_runtime_options(
  const trajectory_mppi_optimizer::Params & params)
{
  FirstOrderDubinsMppiRuntimeOptions output;
  output.enable_debug_trajectory_log = params.enable_debug_trajectory_log;
  output.debug_trajectory_log_directory = params.debug_trajectory_log_directory;
  output.ignore_obstacles = params.ignore_obstacles;
  output.ignore_road_borders = params.ignore_road_borders;
  output.ignore_drivable_area = params.ignore_drivable_area;
  output.force_cold_start_each_step = params.force_cold_start_each_step;
  output.skip_if_invalid = params.skip_if_invalid;
  output.min_optimization_length = static_cast<float>(params.min_optimization_length);
  output.use_last_control_as_nominal = params.use_last_control_as_nominal;
  output.use_temporal_mpt_as_nominal = params.use_temporal_mpt_as_nominal;
  output.prevent_reverse_velocity = params.prevent_reverse_velocity;
  output.enable_input_delay_compensation = params.enable_input_delay_compensation;
  return output;
}

/** @brief Reads standard actuator dynamics and combines them with vehicle geometry. */
FirstOrderDubinsMppiVehicleParams make_vehicle_params(
  rclcpp::Node & node, const autoware::vehicle_info_utils::VehicleInfo & vehicle_info)
{
  auto output = makeVehicleParams(vehicle_info);
  output.acc_time_constant =
    static_cast<float>(node.get_parameter("acc_time_constant").as_double());
  output.steer_time_constant =
    static_cast<float>(node.get_parameter("steer_time_constant").as_double());
  output.steer_rate_lim = static_cast<float>(node.get_parameter("steer_rate_lim").as_double());
  output.vel_rate_lim = static_cast<float>(node.get_parameter("vel_rate_lim").as_double());
  output.acc_time_delay = static_cast<float>(node.get_parameter("acc_time_delay").as_double());
  output.steer_time_delay = static_cast<float>(node.get_parameter("steer_time_delay").as_double());
  return output;
}

autoware::avoidance_target_detector::ExtendedRouteHandler::VelocityLimitOverrides
make_velocity_limit_overrides(const trajectory_mppi_optimizer::Params & params)
{
  autoware::avoidance_target_detector::ExtendedRouteHandler::VelocityLimitOverrides overrides;
  const auto & ids = params.limit_velocity_from_map_debug_lanelet_ids;
  const auto & velocities = params.limit_velocity_from_map_debug_max_velocities;
  if (ids.size() != velocities.size()) {
    throw std::invalid_argument(
      "mppi_optimizer.limit_velocity_from_map_debug_lanelet_ids and "
      "mppi_optimizer.limit_velocity_from_map_debug_max_velocities must have equal lengths");
  }
  for (std::size_t index = 0; index < ids.size(); ++index) {
    if (!std::isfinite(velocities[index]) || velocities[index] < 0.0) {
      throw std::invalid_argument(
        "mppi_optimizer.limit_velocity_from_map_debug_max_velocities must contain finite "
        "non-negative values");
    }
    if (!overrides.emplace(ids[index], velocities[index]).second) {
      throw std::invalid_argument(
        "mppi_optimizer.limit_velocity_from_map_debug_lanelet_ids must not contain duplicates");
    }
  }
  return overrides;
}

/** @brief Converts Autoware geometry segments into the MPPI host format. */
std::vector<Segment> to_mppi_segments(const std::vector<Segment2d> & segments)
{
  std::vector<Segment> output;
  output.reserve(segments.size());
  for (const auto & segment : segments) {
    output.push_back(
      Segment{
        static_cast<float>(boost::geometry::get<0, 0>(segment)),
        static_cast<float>(boost::geometry::get<0, 1>(segment)),
        static_cast<float>(boost::geometry::get<1, 0>(segment)),
        static_cast<float>(boost::geometry::get<1, 1>(segment))});
  }
  return output;
}

/** @brief Planar distance [m] from ego odometry to the first DP reference point. */
double ego_to_first_reference_point_distance_m(
  const nav_msgs::msg::Odometry & odometry, const Trajectory & reference)
{
  if (reference.points.empty()) {
    return 0.0;
  }
  const auto & ego = odometry.pose.pose.position;
  const auto & first = reference.points.front().pose.position;
  return std::hypot(ego.x - first.x, ego.y - first.y);
}

/** Match mppi::cost::detail::signedLateralOffsetPointToSegment (+ = left of tangent). */
double signed_lateral_offset_point_to_segment(
  const double px, const double py, const double x0, const double y0, const double x1,
  const double y1)
{
  const double dx = x1 - x0;
  const double dy = y1 - y0;
  const double len_sq = dx * dx + dy * dy;
  if (len_sq < 1.0E-8) {
    return px - x0;
  }
  const double t = std::clamp(((px - x0) * dx + (py - y0) * dy) / len_sq, 0.0, 1.0);
  const double cx = x0 + t * dx;
  const double cy = y0 + t * dy;
  const double len = std::hypot(dx, dy);
  return ((px - cx) * (-dy) + (py - cy) * dx) / len;
}

/**
 * @brief Signed cross-track error [m] from ego to the closest segment on the raw DP polyline.
 * Uses the same closest-segment search and sign convention as MPPI lateral_distance_coeff.
 */
double ego_signed_lateral_error_on_reference_m(
  const nav_msgs::msg::Odometry & odometry, const Trajectory & reference)
{
  if (reference.points.size() < 2) {
    return 0.0;
  }
  const double px = odometry.pose.pose.position.x;
  const double py = odometry.pose.pose.position.y;

  double best_signed = 0.0;
  double best_dist_sq = std::numeric_limits<double>::max();
  for (size_t i = 0; i + 1 < reference.points.size(); ++i) {
    const auto & p0 = reference.points[i].pose.position;
    const auto & p1 = reference.points[i + 1].pose.position;
    const double dx = p1.x - p0.x;
    const double dy = p1.y - p0.y;
    const double len_sq = dx * dx + dy * dy;
    const double t = (len_sq < 1.0E-8)
                       ? 0.0
                       : std::clamp(((px - p0.x) * dx + (py - p0.y) * dy) / len_sq, 0.0, 1.0);
    const double cx = p0.x + t * dx;
    const double cy = p0.y + t * dy;
    const double dist_sq = (px - cx) * (px - cx) + (py - cy) * (py - cy);
    if (dist_sq < best_dist_sq) {
      best_dist_sq = dist_sq;
      best_signed = signed_lateral_offset_point_to_segment(px, py, p0.x, p0.y, p1.x, p1.y);
    }
  }
  return best_signed;
}

}  // namespace

void TrajectoryMppiOptimizer::on_initialize(
  const autoware::trajectory_processor::TrajectoryProcessorParams &)
{
  auto * const node = get_node_ptr();
  param_listener_ =
    std::make_unique<trajectory_mppi_optimizer::ParamListener>(node, "mppi_optimizer");
  params_ = param_listener_->get_params();
  map_velocity_limit_overrides_ = make_velocity_limit_overrides(params_);
  declare_first_order_dubins_mppi_vehicle_dynamics_params(*node);

  velocity_limit_sub_ =
    std::make_shared<autoware_utils_rclcpp::InterProcessPollingSubscriber<VelocityLimit>>(
      node, "~/input/external_velocity_limit_mps", rclcpp::QoS{1});

  reference_trajectory_pub_ =
    node->create_publisher<Trajectory>("~/debug/mppi/reference_trajectory", 1);
  nominal_control_trajectory_pub_ =
    node->create_publisher<Trajectory>("~/debug/mppi/nominal_control_trajectory", 1);
  optimized_trajectory_pub_ =
    node->create_publisher<Trajectory>("~/debug/mppi/optimized_trajectory", 1);
  nominal_trajectory_pub_ =
    node->create_publisher<Trajectory>("~/debug/mppi/nominal_trajectory", 1);
  velocity_limit_trajectory_pub_ =
    node->create_publisher<Trajectory>("~/debug/mppi/velocity_limit_trajectory", 1);
  markers_pub_ = node->create_publisher<MarkerArray>("~/debug/mppi/markers", 1);
  enabled_pub_ = node->create_publisher<std_msgs::msg::Bool>(
    "~/debug/mppi/enabled", rclcpp::QoS{1}.transient_local());
  debug_publisher_ = std::make_unique<autoware_utils_debug::DebugPublisher>(node, "~/debug");
  cost_diagnostics_ = std::make_unique<DiagnosticsInterface>(node, "mppi_cost_breakdown");
  publish_enabled(false);
}

void TrajectoryMppiOptimizer::update_params(
  const autoware::trajectory_processor::TrajectoryProcessorParams &)
{
}

ProcessingResult TrajectoryMppiOptimizer::process(
  TrajectoryPoints & trajectory_points,
  autoware::trajectory_processor::TrajectoryProcessorData & data)
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *get_time_keeper());

  auto updated_params = params_;
  if (param_listener_->try_update_params(updated_params)) {
    try {
      map_velocity_limit_overrides_ = make_velocity_limit_overrides(updated_params);
    } catch (const std::exception & error) {
      constexpr auto level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      publish_enabled(false);
      publish_status_diagnostic(level, error.what(), rclcpp::Time{data.candidate_header.stamp});
      RCLCPP_ERROR(get_node_ptr()->get_logger(), "%s", error.what());
      return ProcessingResult::Unchanged;
    }
    params_ = std::move(updated_params);
    reset_optimizer();
  }
  if (data.candidate_index != 0U) {
    return ProcessingResult::Unchanged;
  }

  pending_debug_.reset();
  pending_markers_.markers.clear();
  debug_pending_ = false;

  if (!params_.enabled) {
    publish_enabled(false);
    clear_markers(data.candidate_header);
    return ProcessingResult::Unchanged;
  }

  if (!data.current_odometry || !data.tracked_objects || !data.route || !data.lanelet_map_bin) {
    constexpr auto level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    publish_enabled(false);
    clear_markers(data.candidate_header);
    publish_status_diagnostic(
      level, "MPPI input data is not ready", rclcpp::Time{data.candidate_header.stamp});
    RCLCPP_WARN_THROTTLE(
      get_node_ptr()->get_logger(), *get_node_ptr()->get_clock(), 5000,
      "MPPI input data is not ready: odometry, tracked objects, route, or map is missing");
    return ProcessingResult::Unchanged;
  }

  try {
    update_route_context(data);
    ensure_optimizer();

    Trajectory input;
    input.header = data.candidate_header;
    input.points = trajectory_points;

    const auto objects_in_range = autoware::avoidance_target_detector::filter_objects_in_range(
      *data.tracked_objects, input, object_filter_margin_m_, object_filter_prediction_extension_s_);
    object_selector_.update_objects(
      get_node_ptr()->now(), objects_in_range, input, *extended_route_handler_);
    auto avoidance_targets = object_selector_.get_avoidance_targets(
      objects_in_range, input, extended_route_handler_->get_extended_route_bounds());
    const auto driving_along_targets =
      object_selector_.get_driving_along_vehicles(objects_in_range);

    auto all_targets = avoidance_targets;
    all_targets.objects.insert(
      all_targets.objects.end(), driving_along_targets.objects.begin(),
      driving_along_targets.objects.end());

    const double boundary_query_margin = context_->vehicle_info.max_longitudinal_offset_m + 1.0;
    const auto road_borders =
      extended_route_handler_->get_road_borders_around_trajectory(input, boundary_query_margin);
    const auto drivable_area =
      extended_route_handler_->get_drivable_area_around_trajectory(input, boundary_query_margin);

    const std::optional<geometry_msgs::msg::AccelWithCovarianceStamped> acceleration =
      data.current_acceleration ? std::make_optional(*data.current_acceleration) : std::nullopt;
    const std::optional<autoware_vehicle_msgs::msg::SteeringReport> steering =
      data.current_steering ? std::make_optional(*data.current_steering) : std::nullopt;

    const auto velocity_limit = velocity_limit_sub_->take_data();
    auto kinematic_limits =
      velocity_limit ? makeKinematicLimits(*velocity_limit) : FirstOrderDubinsMppiKinematicLimits{};
    if (params_.limit_velocity_from_map) {
      kinematic_limits.max_velocity_by_reference_point.reserve(input.points.size());
      for (const auto & point : input.points) {
        const auto map_velocity_limit = extended_route_handler_->get_velocity_limit(
          point.pose.position, map_velocity_limit_overrides_);
        kinematic_limits.max_velocity_by_reference_point.push_back(
          map_velocity_limit && std::isfinite(*map_velocity_limit) && *map_velocity_limit >= 0.0
            ? std::make_optional(static_cast<float>(*map_velocity_limit))
            : std::nullopt);
      }
    }

    const auto result = optimizer_->optimizeTrajectory(
      input, *data.current_odometry, acceleration, steering, all_targets,
      to_mppi_segments(road_borders), to_mppi_segments(drivable_area), kinematic_limits);

    pending_debug_ = result.debug;
    pending_debug_header_ = input.header;
    pending_markers_ = createMppiDebugMarkers(
      result.debug, road_borders, drivable_area, avoidance_targets, driving_along_targets,
      data.current_odometry->pose.pose.position.z);
    debug_pending_ = true;

    const bool apply_limited_fallback =
      result.debug.was_rejected && result.debug.velocity_limit_profile_active;
    const bool apply_result =
      !params_.shadow_mode && (!result.debug.was_rejected || apply_limited_fallback);
    publish_enabled(apply_result);
    publish_cost_diagnostics(result.debug, apply_result, rclcpp::Time{input.header.stamp});
    publish_processing_time(result.debug.timing);
    publish_ego_to_dp_first_point_distance(*data.current_odometry, input);
    publish_ego_signed_lateral_error_on_dp(*data.current_odometry, input);
    if (result.debug.was_rejected) {
      pending_markers_.markers.clear();
      clear_markers(input.header);
    }
    if (!apply_result) {
      return ProcessingResult::Unchanged;
    }

    trajectory_points = result.trajectory.points;
    return ProcessingResult::Modified;
  } catch (const std::exception & error) {
    constexpr auto level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    publish_enabled(false);
    clear_markers(data.candidate_header);
    publish_status_diagnostic(level, error.what(), rclcpp::Time{data.candidate_header.stamp});
    RCLCPP_ERROR_THROTTLE(
      get_node_ptr()->get_logger(), *get_node_ptr()->get_clock(), 1000,
      "MPPI optimization failed: %s", error.what());
    return ProcessingResult::Unchanged;
  }
}

void TrajectoryMppiOptimizer::reset_optimizer()
{
  optimizer_.reset();
  object_selector_ = autoware::avoidance_target_detector::TrackedObjectSelector{};
}

void TrajectoryMppiOptimizer::update_route_context(
  const autoware::trajectory_processor::TrajectoryProcessorData & data)
{
  const bool route_changed =
    !current_route_uuid_ || current_route_uuid_.value() != data.route->uuid;
  const bool map_changed = current_map_ != data.lanelet_map_bin;
  if (!route_changed && !map_changed && extended_route_handler_) {
    return;
  }

  current_route_uuid_ = data.route->uuid;
  current_map_ = data.lanelet_map_bin;
  extended_route_handler_ =
    std::make_shared<autoware::avoidance_target_detector::ExtendedRouteHandler>(
      *current_map_, *data.route);
  extended_route_handler_->create_map();
  reset_optimizer();
}

void TrajectoryMppiOptimizer::ensure_optimizer()
{
  if (optimizer_) {
    return;
  }

  auto cost_params = make_cost_params(params_);
  auto vehicle_params = make_vehicle_params(*get_node_ptr(), context_->vehicle_info);
  optimizer_ = std::make_unique<FirstOrderDubinsMppiInterface>();
  optimizer_->setCostParams(cost_params);
  optimizer_->setVehicleParams(vehicle_params);
  optimizer_->setRuntimeOptions(make_runtime_options(params_));

  const double max_longitudinal_offset = std::max(
    std::abs(context_->vehicle_info.min_longitudinal_offset_m),
    std::abs(context_->vehicle_info.max_longitudinal_offset_m));
  const double max_lateral_offset = std::max(
    std::abs(context_->vehicle_info.min_lateral_offset_m),
    std::abs(context_->vehicle_info.max_lateral_offset_m));
  const double collision_envelope_radius = std::hypot(
    max_longitudinal_offset + cost_params.obstacle_collision_margin,
    max_lateral_offset + cost_params.obstacle_collision_margin);
  const double barrier_envelope_radius =
    std::hypot(max_longitudinal_offset, max_lateral_offset) + cost_params.obstacle_safe_margin;
  object_filter_margin_m_ = std::max(collision_envelope_radius, barrier_envelope_radius);

  const double max_vehicle_delay_s =
    std::max(vehicle_params.acc_time_delay, vehicle_params.steer_time_delay);
  const double delay_steps =
    std::max(0.0, std::round(max_vehicle_delay_s / autoware::mppi_optimizer::detail::kMppiDt));
  object_filter_prediction_extension_s_ = delay_steps * autoware::mppi_optimizer::detail::kMppiDt;
}

void TrajectoryMppiOptimizer::publish_enabled(const bool enabled) const
{
  std_msgs::msg::Bool message;
  message.data = enabled;
  enabled_pub_->publish(message);
}

void TrajectoryMppiOptimizer::publish_debug_data(const std::string &) const
{
  if (!debug_pending_ || !pending_debug_) {
    return;
  }

  auto reference = pending_debug_->reference_trajectory;
  auto nominal_control = pending_debug_->reference_trajectory;
  auto optimized = pending_debug_->optimized_trajectory;
  auto nominal = pending_debug_->nominal_trajectory;
  auto velocity_limits = pending_debug_->reference_trajectory;
  reference.header = pending_debug_header_;
  nominal_control.header = pending_debug_header_;
  optimized.header = pending_debug_header_;
  nominal.header = pending_debug_header_;
  velocity_limits.header = pending_debug_header_;

  const auto & effective_limits = pending_debug_->effective_max_velocity_by_reference_point;
  const std::size_t velocity_limit_size =
    std::min(velocity_limits.points.size(), effective_limits.size());
  velocity_limits.points.resize(velocity_limit_size);
  for (std::size_t index = 0; index < velocity_limit_size; ++index) {
    velocity_limits.points[index].longitudinal_velocity_mps =
      effective_limits[index] ? *effective_limits[index] : std::numeric_limits<float>::quiet_NaN();
    velocity_limits.points[index].acceleration_mps2 = 0.0F;
  }

  const auto & profile = pending_debug_->nominal_control_profile;
  const std::size_t size = std::min(
    {nominal_control.points.size(), profile.acceleration_commands_mps2.size(),
     profile.steering_commands_rad.size()});
  nominal_control.points.resize(size);
  for (std::size_t index = 0; index < size; ++index) {
    nominal_control.points[index].acceleration_mps2 = profile.acceleration_commands_mps2[index];
    nominal_control.points[index].front_wheel_angle_rad = profile.steering_commands_rad[index];
  }

  reference_trajectory_pub_->publish(reference);
  nominal_control_trajectory_pub_->publish(nominal_control);
  optimized_trajectory_pub_->publish(optimized);
  nominal_trajectory_pub_->publish(nominal);
  velocity_limit_trajectory_pub_->publish(velocity_limits);
  markers_pub_->publish(pending_markers_);
  debug_pending_ = false;
}

void TrajectoryMppiOptimizer::publish_cost_diagnostics(
  const FirstOrderDubinsMppiDebug & debug, const bool was_applied, const rclcpp::Time & stamp)
{
  using diagnostic_msgs::msg::DiagnosticStatus;
  cost_diagnostics_->clear();
  const auto & cost = debug.cost_breakdown;
  cost_diagnostics_->add_key_value("controller_baseline_cost", debug.baseline_cost);
  cost_diagnostics_->add_key_value("output_total_cost", cost.total);
  cost_diagnostics_->add_key_value("output_minus_baseline_cost", cost.total - debug.baseline_cost);
  cost_diagnostics_->add_key_value("running_total", cost.running_total);
  cost_diagnostics_->add_key_value("terminal_total", cost.terminal_total);
  cost_diagnostics_->add_key_value("evaluated_timesteps", cost.evaluated_timesteps);
  cost_diagnostics_->add_key_value("state/speed", cost.speed);
  cost_diagnostics_->add_key_value("state/track", cost.track);
  cost_diagnostics_->add_key_value("state/heading", cost.heading);
  cost_diagnostics_->add_key_value("state/lateral_distance", cost.lateral_distance);
  cost_diagnostics_->add_key_value("state/signed_lateral_error_m", cost.signed_lateral_error_m);
  cost_diagnostics_->add_key_value("state/lateral_yaw_error", cost.lateral_yaw_error);
  cost_diagnostics_->add_key_value("state/track_center", cost.track_center);
  cost_diagnostics_->add_key_value("state/corner_buffer", cost.corner_buffer);
  cost_diagnostics_->add_key_value("state/drivable_area", cost.drivable_area);
  cost_diagnostics_->add_key_value("state/obstacle", cost.obstacle);
  cost_diagnostics_->add_key_value("state/road_border", cost.road_border);
  cost_diagnostics_->add_key_value("state/remaining_distance", cost.remaining_distance);
  cost_diagnostics_->add_key_value("state/path_overshoot", cost.path_overshoot);
  cost_diagnostics_->add_key_value("control/acceleration_command", cost.acceleration_command);
  cost_diagnostics_->add_key_value("control/steering_command", cost.steering_command);
  cost_diagnostics_->add_key_value("comfort/lateral_acceleration", cost.lateral_acceleration);
  cost_diagnostics_->add_key_value("comfort/lateral_jerk", cost.lateral_jerk);
  cost_diagnostics_->add_key_value("comfort/longitudinal_jerk", cost.longitudinal_jerk);
  cost_diagnostics_->add_key_value("comfort/steering_rate", cost.steering_rate);
  cost_diagnostics_->add_key_value("validation_reason", to_string(debug.validation.reasons));
  cost_diagnostics_->add_key_value(
    "first_invalid_index", debug.validation.first_invalid_index
                             ? std::to_string(debug.validation.first_invalid_index.value())
                             : std::string{"none"});
  cost_diagnostics_->add_key_value("was_rejected", debug.was_rejected);
  cost_diagnostics_->add_key_value(
    "external_velocity_limit_active", debug.external_velocity_limit_active);
  cost_diagnostics_->add_key_value("map_velocity_limit_active", debug.map_velocity_limit_active);
  cost_diagnostics_->add_key_value(
    "velocity_limit_profile_active", debug.velocity_limit_profile_active);
  cost_diagnostics_->add_key_value("was_applied", was_applied);

  if (cost.evaluated_timesteps == 0U) {
    cost_diagnostics_->update_level_and_message(
      DiagnosticStatus::STALE, "MPPI optimization skipped");
  } else if (!std::isfinite(cost.total) || !std::isfinite(debug.baseline_cost)) {
    cost_diagnostics_->update_level_and_message(DiagnosticStatus::ERROR, "Non-finite MPPI cost");
  } else if (debug.was_rejected) {
    cost_diagnostics_->update_level_and_message(DiagnosticStatus::WARN, "MPPI trajectory rejected");
  } else {
    cost_diagnostics_->update_level_and_message(
      DiagnosticStatus::OK, was_applied ? "MPPI trajectory applied" : "MPPI shadow output");
  }
  cost_diagnostics_->publish(stamp);
}

void TrajectoryMppiOptimizer::publish_status_diagnostic(
  const std::uint8_t level, const std::string & message, const rclcpp::Time & stamp)
{
  cost_diagnostics_->clear();
  cost_diagnostics_->update_level_and_message(level, message);
  cost_diagnostics_->publish(stamp);
}

void TrajectoryMppiOptimizer::publish_processing_time(const FirstOrderDubinsMppiTiming & timing)
{
  if (!debug_publisher_) {
    return;
  }
  // Sibling Float64Stamped topics under ~/debug/processing_time_ms/ so PlotJuggler shows
  // subdivisions next to the processor total (~/debug/processing_time_ms.data).
  debug_publisher_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
    "processing_time_ms/nominal", timing.seed_nominal_ms);
  debug_publisher_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
    "processing_time_ms/mppi", timing.total_ms);
}

void TrajectoryMppiOptimizer::publish_ego_to_dp_first_point_distance(
  const nav_msgs::msg::Odometry & odometry, const Trajectory & reference) const
{
  if (!debug_publisher_ || reference.points.empty()) {
    return;
  }
  debug_publisher_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
    "mppi/ego_to_dp_first_point_distance_m",
    ego_to_first_reference_point_distance_m(odometry, reference));
}

void TrajectoryMppiOptimizer::publish_ego_signed_lateral_error_on_dp(
  const nav_msgs::msg::Odometry & odometry, const Trajectory & reference) const
{
  if (!debug_publisher_ || reference.points.size() < 2) {
    return;
  }
  debug_publisher_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
    "mppi/ego_signed_lateral_error_on_dp_m",
    ego_signed_lateral_error_on_reference_m(odometry, reference));
}

void TrajectoryMppiOptimizer::clear_markers(const std_msgs::msg::Header & header) const
{
  visualization_msgs::msg::Marker marker;
  marker.header = header;
  marker.action = visualization_msgs::msg::Marker::DELETEALL;
  MarkerArray markers;
  markers.markers.push_back(marker);
  markers_pub_->publish(markers);
}

}  // namespace autoware::mppi_optimizer::plugin

PLUGINLIB_EXPORT_CLASS(
  autoware::mppi_optimizer::plugin::TrajectoryMppiOptimizer,
  autoware::trajectory_processor::plugin::TrajectoryProcessorPluginBase)
