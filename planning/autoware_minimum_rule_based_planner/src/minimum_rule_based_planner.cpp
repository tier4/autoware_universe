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

#include "minimum_rule_based_planner.hpp"

#include "autoware/trajectory_processor/trajectory_processor_parameters.hpp"

#include <autoware/avoidance_target_detector/object_filtering.hpp>
#include <autoware/motion_utils/resample/resample.hpp>
#include <autoware/motion_utils/trajectory/conversion.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/mppi_optimizer/detail/trajectory_utils.hpp>
#include <autoware/mppi_optimizer/first_order_dubins_mppi_cost_params_ros.hpp>
#include <autoware/mppi_optimizer/first_order_dubins_mppi_runtime_options_ros.hpp>
#include <autoware/mppi_optimizer/first_order_dubins_mppi_vehicle_params_ros.hpp>
#include <autoware/trajectory/utils/pretty_build.hpp>
#include <autoware/velocity_smoother/resample.hpp>
#include <autoware_utils/geometry/geometry.hpp>

#include <autoware_perception_msgs/msg/tracked_object.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace autoware::minimum_rule_based_planner
{

namespace
{
trajectory_processor::TrajectoryProcessorData make_optimizer_data(
  const MinimumRuleBasedPlannerNode::InputData & input_data)
{
  trajectory_processor::TrajectoryProcessorData data;
  data.current_odometry = input_data.odometry_ptr;
  data.current_acceleration = input_data.acceleration_ptr;
  return data;
}

minimum_rule_based_planner::plugin::ModifierData make_modifier_data(
  const MinimumRuleBasedPlannerNode::InputData & input_data)
{
  minimum_rule_based_planner::plugin::ModifierData data;
  data.odometry_ptr = input_data.odometry_ptr;
  data.acceleration_ptr = input_data.acceleration_ptr;
  data.predicted_objects_ptr = input_data.predicted_objects_ptr;
  data.obstacle_pointcloud_ptr = input_data.obstacle_pointcloud_ptr;
  return data;
}

void assign_time_from_start(
  TrajectoryPoints & traj_points, const geometry_msgs::msg::Point & ego_point)
{
  constexpr double k_min_velocity = 1.0;  // [m/s] velocity floor for dt = ds / v
  constexpr double k_min_dt = 1.0e-3;     // [s] keeps the time base strictly increasing

  if (traj_points.size() < 2) {
    return;
  }

  std::vector<double> times{0.0};
  times.reserve(traj_points.size());
  for (size_t i = 1; i < traj_points.size(); ++i) {
    const double ds = autoware_utils::calc_distance2d(traj_points.at(i - 1), traj_points.at(i));
    const double v =
      std::max<double>(std::abs(traj_points.at(i - 1).longitudinal_velocity_mps), k_min_velocity);
    times.push_back(times.back() + std::max(ds / v, k_min_dt));
  }

  const size_t ego_segment =
    autoware::motion_utils::findNearestSegmentIndex(traj_points, ego_point);
  const auto & segment_start = traj_points.at(ego_segment).pose.position;
  const auto & segment_end = traj_points.at(ego_segment + 1U).pose.position;
  const double segment_x = segment_end.x - segment_start.x;
  const double segment_y = segment_end.y - segment_start.y;
  const double squared_segment_length = segment_x * segment_x + segment_y * segment_y;
  const double projection = squared_segment_length > std::numeric_limits<double>::epsilon()
                              ? std::clamp(
                                  ((ego_point.x - segment_start.x) * segment_x +
                                   (ego_point.y - segment_start.y) * segment_y) /
                                    squared_segment_length,
                                  0.0, 1.0)
                              : 0.0;
  const double ego_time =
    times.at(ego_segment) + projection * (times.at(ego_segment + 1U) - times.at(ego_segment));
  for (size_t i = 0; i < traj_points.size(); ++i) {
    traj_points.at(i).time_from_start = rclcpp::Duration::from_seconds(times.at(i) - ego_time);
  }
}

std::optional<Trajectory> resample_for_mppi(const Trajectory & trajectory)
{
  const auto temporal_trajectory =
    autoware::experimental::trajectory::pretty_build_temporal(trajectory.points);
  if (!temporal_trajectory) {
    return std::nullopt;
  }

  Trajectory result;
  result.header = trajectory.header;
  result.points.reserve(
    trajectory.points.size() + static_cast<size_t>(autoware::mppi_optimizer::detail::kMppiHorizon));
  for (int i = 1; i <= autoware::mppi_optimizer::detail::kMppiHorizon; ++i) {
    const double time = i * autoware::mppi_optimizer::detail::kMppiDt;
    if (time > temporal_trajectory->end_time()) {
      break;
    }
    auto point = temporal_trajectory->compute_from_time(time);
    point.time_from_start = rclcpp::Duration::from_seconds(time);
    result.points.push_back(std::move(point));
  }
  if (result.points.size() < 2U) {
    return std::nullopt;
  }

  const double sampled_end_time = rclcpp::Duration(result.points.back().time_from_start).seconds();
  for (const auto & point : trajectory.points) {
    if (rclcpp::Duration(point.time_from_start).seconds() > sampled_end_time) {
      result.points.push_back(point);
    }
  }
  return result;
}

autoware::mppi_optimizer::TrackedObjects to_tracked_objects(const PredictedObjects & objects)
{
  autoware::mppi_optimizer::TrackedObjects result;
  result.header = objects.header;
  result.objects.reserve(objects.objects.size());
  for (const auto & predicted : objects.objects) {
    autoware_perception_msgs::msg::TrackedObject tracked;
    tracked.object_id = predicted.object_id;
    tracked.existence_probability = predicted.existence_probability;
    tracked.classification = predicted.classification;
    tracked.shape = predicted.shape;
    tracked.kinematics.pose_with_covariance = predicted.kinematics.initial_pose_with_covariance;
    tracked.kinematics.twist_with_covariance = predicted.kinematics.initial_twist_with_covariance;
    tracked.kinematics.acceleration_with_covariance =
      predicted.kinematics.initial_acceleration_with_covariance;
    tracked.kinematics.orientation_availability =
      autoware_perception_msgs::msg::TrackedObjectKinematics::AVAILABLE;
    tracked.kinematics.is_stationary =
      std::abs(tracked.kinematics.twist_with_covariance.twist.linear.x) < 0.1;
    result.objects.push_back(std::move(tracked));
  }
  return result;
}

template <class Segment>
std::vector<autoware::mppi_optimizer::Segment> to_mppi_segments(
  const std::vector<Segment> & segments)
{
  std::vector<autoware::mppi_optimizer::Segment> result;
  result.reserve(segments.size());
  for (const auto & segment : segments) {
    result.push_back(
      {static_cast<float>(boost::geometry::get<0, 0>(segment)),
       static_cast<float>(boost::geometry::get<0, 1>(segment)),
       static_cast<float>(boost::geometry::get<1, 0>(segment)),
       static_cast<float>(boost::geometry::get<1, 1>(segment))});
  }
  return result;
}
}  // namespace

MinimumRuleBasedPlannerNode::MinimumRuleBasedPlannerNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("minimum_rule_based_planner_node", options),
  go_generator_uuid_(autoware_utils_uuid::generate_uuid()),
  stop_generator_uuid_(autoware_utils_uuid::generate_uuid()),
  vehicle_info_(vehicle_info_utils::VehicleInfoUtils(*this).getVehicleInfo()),
  optimizer_context_(
    std::make_shared<autoware::trajectory_processor::TrajectoryProcessorContext>(this)),
  modifier_plugin_loader_(
    "autoware_minimum_rule_based_planner",
    "autoware::minimum_rule_based_planner::plugin::PluginInterface"),
  modifier_context_(std::make_shared<plugin::ModifierContext>(this))
{
  autoware::mppi_optimizer::declare_first_order_dubins_mppi_cost_params(*this);
  autoware::mppi_optimizer::declare_first_order_dubins_mppi_vehicle_dynamics_params(*this);
  autoware::mppi_optimizer::declare_first_order_dubins_mppi_runtime_options(*this);

  param_listener_ =
    std::make_shared<::minimum_rule_based_planner::ParamListener>(get_node_parameters_interface());

  pub_trajectories_ =
    this->create_publisher<CandidateTrajectories>("~/output/candidate_trajectories", 1);
  pub_stop_lines_marker_ =
    this->create_publisher<visualization_msgs::msg::MarkerArray>("~/debug/stop_lines", 1);
  pub_debug_path_ = this->create_publisher<PathWithLaneId>("~/debug/path_with_lane_id", 1);
  pub_debug_trajectory_ = this->create_publisher<Trajectory>("~/debug/trajectory", 1);
  pub_debug_stop_trajectory_ = this->create_publisher<Trajectory>("~/debug/stop_trajectory", 1);
  pub_debug_shifted_trajectory_ =
    this->create_publisher<Trajectory>("~/debug/shifted_trajectory", 1);
  pub_debug_mppi_nominal_trajectory_ =
    this->create_publisher<Trajectory>("~/debug/mppi/nominal_trajectory", 1);
  pub_debug_mppi_input_trajectory_ =
    this->create_publisher<Trajectory>("~/debug/mppi/input_trajectory", 1);
  pub_debug_mppi_output_trajectory_ =
    this->create_publisher<Trajectory>("~/debug/mppi/output_trajectory", 1);
  debug_processing_time_detail_pub_ =
    this->create_publisher<autoware_utils_debug::ProcessingTimeDetail>(
      "~/debug/processing_time_detail_ms", 1);
  debug_processing_time_pub_ =
    this->create_publisher<autoware_internal_debug_msgs::msg::Float64Stamped>(
      "~/debug/processing_time_ms", 1);
  time_keeper_ =
    std::make_shared<autoware_utils_debug::TimeKeeper>(debug_processing_time_detail_pub_);

  params_ = param_listener_->get_params();

  mppi_optimizer_ = std::make_unique<autoware::mppi_optimizer::FirstOrderDubinsMppiInterface>();
  mppi_optimizer_->setCostParams(
    autoware::mppi_optimizer::get_first_order_dubins_mppi_cost_params(*this));
  mppi_optimizer_->setVehicleParams(
    autoware::mppi_optimizer::get_first_order_dubins_mppi_vehicle_params(*this));
  mppi_optimizer_->setRuntimeOptions(
    autoware::mppi_optimizer::get_first_order_dubins_mppi_runtime_options(*this));

  load_optimizer_plugins();
  load_modifier_plugins();

  path_planner_ =
    std::make_unique<PathPlanner>(get_logger(), get_clock(), time_keeper_, params_, vehicle_info_);
  map_based_stop_planner_ = std::make_unique<MapBasedStopPlanner>(get_logger(), time_keeper_);
  timer_ = rclcpp::create_timer(
    this, get_clock(), rclcpp::Rate(params_.planning_frequency_hz).period(),
    std::bind(&MinimumRuleBasedPlannerNode::on_timer, this));

  RCLCPP_INFO(get_logger(), "Minimum Rule Based Planner Node has been started.");
}

void MinimumRuleBasedPlannerNode::load_optimizer_plugins()
{
  // Create the common loader for optimizer plugins exported by autoware_trajectory_processor.
  plugin_loader_ = std::make_unique<OptimizerPluginLoader>(
    "autoware_trajectory_processor",
    "autoware::trajectory_processor::plugin::TrajectoryProcessorPluginBase");

  auto try_load_optimizer_plugin = [&](const std::string & plugin_path, const std::string & name)
    -> std::shared_ptr<OptimizerPluginInterface> {
    trajectory_processor::TrajectoryProcessorParams processor_params;
    processor_params.use_eb_smoother = true;
    try {
      auto plugin = plugin_loader_->createSharedInstance(plugin_path);
      plugin->initialize(
        plugin_path, name, this, time_keeper_, optimizer_context_, processor_params);
      pub_debug_optimizer_module_trajectories_[plugin->get_name()] =
        this->create_publisher<Trajectory>(
          "~/debug/optimizer/" + plugin->get_name() + "/trajectory", 1);
      RCLCPP_INFO(get_logger(), "Loaded trajectory %s plugin", name.c_str());
      return plugin;
    } catch (const pluginlib::PluginlibException & ex) {
      RCLCPP_ERROR(
        get_logger(), "Failed to load trajectory %s plugin: %s", name.c_str(), ex.what());
      return nullptr;
    }
  };

  path_smoother_ = try_load_optimizer_plugin(
    "autoware::trajectory_processor::plugin::TrajectoryEBSmootherOptimizer", "eb_smoother");

  // Set up velocity optimizer
  // NOTE(odashima):
  // The velocity_optimizer modifier plugin used in diffusion_planner has different processing,
  // so a separate implementation is provided here.
  {
    VelocitySmootherParams vel_params;
    vel_params.nearest_dist_threshold_m =
      declare_parameter<double>("velocity_smoother.nearest_dist_threshold_m");
    vel_params.nearest_yaw_threshold_deg =
      declare_parameter<double>("velocity_smoother.nearest_yaw_threshold_deg");
    vel_params.target_pull_out_speed_mps =
      declare_parameter<double>("velocity_smoother.target_pull_out_speed_mps");
    vel_params.target_pull_out_acc_mps2 =
      declare_parameter<double>("velocity_smoother.target_pull_out_acc_mps2");
    vel_params.max_speed_mps = declare_parameter<double>("velocity_smoother.max_speed_mps");
    vel_params.max_lateral_accel_mps2 =
      declare_parameter<double>("velocity_smoother.max_lateral_accel_mps2");
    vel_params.stop_dist_to_prohibit_engage =
      declare_parameter<double>("velocity_smoother.stop_dist_to_prohibit_engage");
    vel_params.set_engage_speed = declare_parameter<bool>("velocity_smoother.set_engage_speed");
    vel_params.limit_speed = declare_parameter<bool>("velocity_smoother.limit_speed");
    vel_params.limit_lateral_acceleration =
      declare_parameter<bool>("velocity_smoother.limit_lateral_acceleration");
    vel_params.smooth_velocities = declare_parameter<bool>("velocity_smoother.smooth_velocities");

    const auto vehicle_info =
      autoware::vehicle_info_utils::VehicleInfoUtils(*this).getVehicleInfo();
    auto jerk_filtered_smoother =
      std::make_shared<autoware::velocity_smoother::JerkFilteredSmoother>(*this, time_keeper_);

    velocity_smoother_ = std::make_unique<VelocitySmoother>(
      vel_params, get_logger(), get_clock(), vehicle_info, std::move(jerk_filtered_smoother));
  }
}

void MinimumRuleBasedPlannerNode::load_modifier_plugins()
{
  for (const auto & name : params_.plugin_names) {
    if (name.empty()) continue;
    load_plugin(name);
  }

  initialized_modifiers_ = true;
  RCLCPP_INFO(
    get_logger(), "Trajectory modifier plugins initialized: %zu plugins", modifier_plugins_.size());
}

void MinimumRuleBasedPlannerNode::load_plugin(const std::string & name)
{
  // Check if the plugin is already instantiated
  auto it = std::find_if(modifier_plugins_.begin(), modifier_plugins_.end(), [&](const auto & p) {
    return p->get_name() == name;
  });
  if (it != modifier_plugins_.end()) {
    RCLCPP_WARN(
      this->get_logger(), "The plugin '%s' is already in the plugins list.", name.c_str());
    return;
  }

  if (modifier_plugin_loader_.isClassAvailable(name)) {
    const auto plugin = modifier_plugin_loader_.createSharedInstance(name);
    plugin->initialize(name, this, time_keeper_, modifier_context_, params_);

    // Convert "autoware::...::ObstacleStop" to "obstacle_stop"
    const auto short_name = [](const std::string & plugin_name) {
      const std::string class_name = plugin_name.find("::") != std::string::npos
                                       ? plugin_name.substr(plugin_name.rfind("::") + 2)
                                       : plugin_name;
      std::string short_name;
      for (size_t i = 0; i < class_name.size(); ++i) {
        if (std::isupper(class_name[i])) {
          if (i > 0) short_name += '_';
          short_name += static_cast<char>(std::tolower(class_name[i]));
        } else {
          short_name += class_name[i];
        }
      }

      return short_name;
    }(name);

    pub_debug_modifier_module_trajectories_[plugin->get_name()] =
      this->create_publisher<Trajectory>("~/debug/modifier/" + short_name + "/trajectory", 1);
    modifier_plugins_.push_back(plugin);
    RCLCPP_DEBUG(this->get_logger(), "The plugin '%s' has been loaded", name.c_str());
  } else {
    RCLCPP_ERROR(this->get_logger(), "The plugin '%s' is not available", name.c_str());
  }
}

void MinimumRuleBasedPlannerNode::unload_plugin(const std::string & name)
{
  auto it = std::remove_if(
    modifier_plugins_.begin(), modifier_plugins_.end(),
    [&](const auto plugin) { return plugin->get_name() == name; });

  if (it == modifier_plugins_.end()) {
    RCLCPP_WARN(
      this->get_logger(), "The plugin '%s' is not in the registered modules", name.c_str());
  } else {
    modifier_plugins_.erase(it, modifier_plugins_.end());
    RCLCPP_INFO(this->get_logger(), "The scene plugin '%s' has been unloaded", name.c_str());
  }
}

void MinimumRuleBasedPlannerNode::publish_debug_trajectory(
  const std::string & plugin_name, const TrajectoryPoints & traj_points) const
{
  Trajectory traj;
  traj.points = traj_points;
  if (const auto it = pub_debug_modifier_module_trajectories_.find(plugin_name);
      it != pub_debug_modifier_module_trajectories_.end()) {
    it->second->publish(traj);
  } else if (const auto opt_it = pub_debug_optimizer_module_trajectories_.find(plugin_name);
             opt_it != pub_debug_optimizer_module_trajectories_.end()) {
    opt_it->second->publish(traj);
  } else {
    RCLCPP_WARN_ONCE(
      this->get_logger(), "No debug trajectory publisher registered for plugin '%s'",
      plugin_name.c_str());
  }
}

void MinimumRuleBasedPlannerNode::on_timer()
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);
  stop_watch_ptr_ = std::make_unique<autoware_utils_system::StopWatch<std::chrono::milliseconds>>();
  stop_watch_ptr_->tic("processing_time");

  // 1. Check data availability
  const auto input_data = take_data();
  path_planner_->set_planner_data(input_data.lanelet_map_bin_ptr, input_data.route_ptr);
  if (!is_data_ready(input_data)) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "Waiting for necessary data to plan trajectories.");
    return;
  }

  if (param_listener_->is_old(params_)) {
    update_params();
  }

  // 2. Get path
  const auto path = plan_path(input_data);

  if (!path) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Failed to plan path.");
    return;
  }

  // 3. Convert path to trajectory
  auto trajectory =
    path_planner_->convert_path_to_trajectory(*path, params_.path_planning.output.delta_arc_length);
  trajectory.header = path->header;

  // 4. Shift trajectory to ego position
  trajectory = shift_trajectory_to_ego(trajectory, input_data);

  // 5. Smooth path
  trajectory = smooth_trajectory(trajectory, input_data);

  // 6. Apply trajectory modifiers
  apply_modifiers(trajectory, input_data);

  // Create timing for MPPI without updating the velocity smoother's cross-cycle state. MPPI is
  // used as a lateral optimizer; the authoritative go/stop velocity profiles are generated below.
  const auto provisional_trajectory =
    optimize_velocity(trajectory, input_data, /*update_smoother_state=*/false);
  trajectory = optimize_with_mppi(provisional_trajectory, input_data);

  // 7. Plan the go/stop trajectories with map-defined stop points
  map_based_stop_planner_->set_planner_data(
    input_data.lanelet_map_bin_ptr, input_data.route_ptr, path_planner_->route_context());
  const auto stop_result = map_based_stop_planner_->plan(
    trajectory, input_data.odometry_ptr->pose.pose, input_data.odometry_ptr->twist.twist.linear.x,
    input_data.acceleration_ptr->accel.accel.linear.x, make_map_based_stop_params());
  pub_stop_lines_marker_->publish(stop_result.stop_line_markers);

  // 8. Velocity optimization
  // NOTE(odashima): the stop trajectory must not update the smoother's prev-output state: it
  // would drag the go trajectory's initial speed down to the stop profile on the next cycle.
  const auto velocity_optimized_go_trajectory =
    optimize_velocity(stop_result.go_trajectory, input_data, /*update_smoother_state=*/true);
  const auto & go_trajectory = velocity_optimized_go_trajectory;
  const auto stop_trajectory =
    stop_result.stop_trajectory
      ? std::make_optional(optimize_velocity(
          *stop_result.stop_trajectory, input_data, /*update_smoother_state=*/false))
      : std::nullopt;

  // 9. Create and publish CandidateTrajectories message
  publish_candidate_trajectories(go_trajectory, stop_trajectory);

  // 10. Publish debug information
  publish_debug_outputs(*path, go_trajectory, stop_trajectory);
  publish_processing_time();
}

StopSelectionParams MinimumRuleBasedPlannerNode::make_map_based_stop_params() const
{
  StopSelectionParams params;
  params.max_deceleration = params_.map_based_stop.max_deceleration;
  params.max_jerk = params_.map_based_stop.max_jerk;
  params.stop_margin_distance = params_.map_based_stop.stop_margin_distance;
  params.stop_distance_from_crosswalk = params_.map_based_stop.stop_distance_from_crosswalk;
  params.stop_distance_from_private_area = params_.map_based_stop.stop_distance_from_private_area;
  params.stop_distance_from_intersection = params_.map_based_stop.stop_distance_from_intersection;
  params.base_link_to_front = vehicle_info_.max_longitudinal_offset_m;
  params.stop_point_diff_threshold = params_.map_based_stop.stop_point_diff_threshold;
  return params;
}

void MinimumRuleBasedPlannerNode::publish_debug_outputs(
  const PathWithLaneId & path, const Trajectory & go_trajectory,
  const std::optional<Trajectory> & stop_trajectory) const
{
  if (params_.debug.enable_path) {
    pub_debug_path_->publish(path);
  }
  if (params_.debug.enable_output_trajectory) {
    pub_debug_trajectory_->publish(go_trajectory);
    if (stop_trajectory) {
      pub_debug_stop_trajectory_->publish(*stop_trajectory);
    }
  }
}

void MinimumRuleBasedPlannerNode::publish_processing_time()
{
  autoware_internal_debug_msgs::msg::Float64Stamped processing_time_msg;
  processing_time_msg.stamp = get_clock()->now();
  processing_time_msg.data = stop_watch_ptr_->toc("processing_time", true);
  debug_processing_time_pub_->publish(processing_time_msg);
}

std::optional<PathWithLaneId> MinimumRuleBasedPlannerNode::plan_path(const InputData & input_data)
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);
  if (input_data.test_path_with_lane_id_ptr) {
    return *input_data.test_path_with_lane_id_ptr;
  }
  return path_planner_->plan_path(
    input_data.odometry_ptr->pose.pose, input_data.odometry_ptr->twist.twist.linear.x,
    input_data.odometry_ptr->header.stamp);
}

Trajectory MinimumRuleBasedPlannerNode::optimize_with_mppi(
  const Trajectory & trajectory, const InputData & input_data)
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);

  if (!input_data.route_ptr) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "MPPI skipped: no route is available");
    return trajectory;
  }
  if (!input_data.steering_status_ptr) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "MPPI skipped: no steering status is available");
    return trajectory;
  }

  const auto mppi_input = resample_for_mppi(trajectory);
  if (!mppi_input) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "MPPI skipped: failed to create a future trajectory sampled at the MPPI time step");
    return trajectory;
  }

  try {
    const bool route_or_map_changed =
      mppi_route_handler_ &&
      (mppi_map_ptr_ != input_data.lanelet_map_bin_ptr || mppi_route_ptr_ != input_data.route_ptr);
    if (route_or_map_changed) {
      mppi_optimizer_ = std::make_unique<autoware::mppi_optimizer::FirstOrderDubinsMppiInterface>();
      mppi_optimizer_->setCostParams(
        autoware::mppi_optimizer::get_first_order_dubins_mppi_cost_params(*this));
      mppi_optimizer_->setVehicleParams(
        autoware::mppi_optimizer::get_first_order_dubins_mppi_vehicle_params(*this));
      mppi_optimizer_->setRuntimeOptions(
        autoware::mppi_optimizer::get_first_order_dubins_mppi_runtime_options(*this));
    }
    if (
      !mppi_route_handler_ || mppi_map_ptr_ != input_data.lanelet_map_bin_ptr ||
      mppi_route_ptr_ != input_data.route_ptr) {
      mppi_route_handler_ =
        std::make_shared<autoware::avoidance_target_detector::ExtendedRouteHandler>(
          *input_data.lanelet_map_bin_ptr, *input_data.route_ptr);
      mppi_route_handler_->create_map();
      mppi_map_ptr_ = input_data.lanelet_map_bin_ptr;
      mppi_route_ptr_ = input_data.route_ptr;
    }

    const double margin = vehicle_info_.max_longitudinal_offset_m + 1.0;
    const auto road_borders =
      mppi_route_handler_->get_road_borders_around_trajectory(trajectory, margin);
    const auto drivable_area =
      mppi_route_handler_->get_drivable_area_around_trajectory(trajectory, margin);
    const std::optional<AccelWithCovarianceStamped> acceleration{*input_data.acceleration_ptr};
    const std::optional<SteeringReport> steering_status{*input_data.steering_status_ptr};
    const auto tracked_objects = input_data.predicted_objects_ptr
                                   ? to_tracked_objects(
                                       autoware::avoidance_target_detector::filter_objects_in_range(
                                         *input_data.predicted_objects_ptr, trajectory, margin))
                                   : autoware::mppi_optimizer::TrackedObjects{};
    const auto result = mppi_optimizer_->optimizeTrajectory(
      *mppi_input, *input_data.odometry_ptr, acceleration, steering_status, tracked_objects,
      to_mppi_segments(road_borders), to_mppi_segments(drivable_area));
    pub_debug_mppi_nominal_trajectory_->publish(result.debug.nominal_trajectory);
    pub_debug_mppi_input_trajectory_->publish(result.debug.reference_trajectory);
    pub_debug_mppi_output_trajectory_->publish(result.debug.optimized_trajectory);
    if (result.debug.was_rejected) {
      return trajectory;
    }
    auto optimized_trajectory = result.trajectory;
    const size_t common_size =
      std::min(optimized_trajectory.points.size(), mppi_input->points.size());
    for (size_t i = 0; i < common_size; ++i) {
      auto & optimized = optimized_trajectory.points.at(i);
      const auto & reference = mppi_input->points.at(i);
      optimized.longitudinal_velocity_mps = reference.longitudinal_velocity_mps;
      optimized.lateral_velocity_mps = reference.lateral_velocity_mps;
      optimized.acceleration_mps2 = reference.acceleration_mps2;
      optimized.heading_rate_rps = reference.heading_rate_rps;
    }
    return optimized_trajectory;
  } catch (const std::runtime_error & error) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 5000, "MPPI optimization failed: %s", error.what());
    return trajectory;
  }
}

Trajectory MinimumRuleBasedPlannerNode::shift_trajectory_to_ego(
  const Trajectory & trajectory, const InputData & input_data) const
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);

  if (!params_.path_planning.path_shift.enable) return trajectory;

  TrajectoryShiftParams shift_params;
  shift_params.minimum_shift_length = params_.path_planning.path_shift.minimum_shift_length;
  shift_params.minimum_shift_distance = params_.path_planning.path_shift.minimum_shift_distance;
  shift_params.min_speed_for_curvature = params_.path_planning.path_shift.min_speed_for_curvature;
  shift_params.lateral_accel_limit = params_.path_planning.path_shift.lateral_accel_limit;

  const double ego_velocity = input_data.odometry_ptr->twist.twist.linear.x;
  const double ego_yaw_rate = input_data.odometry_ptr->twist.twist.angular.z;
  const auto shifted_trajectory = path_planner_->shift_trajectory_to_ego(
    trajectory, input_data.odometry_ptr->pose.pose, ego_velocity, ego_yaw_rate, shift_params,
    params_.path_planning.output.delta_arc_length);

  if (params_.debug.enable_shifted_trajectory) {
    Trajectory shifted_traj;
    shifted_traj.header = trajectory.header;
    shifted_traj.points = shifted_trajectory.points;
    pub_debug_shifted_trajectory_->publish(shifted_traj);
  }
  return shifted_trajectory;
}

Trajectory MinimumRuleBasedPlannerNode::smooth_trajectory(
  const Trajectory & trajectory, const InputData & input_data) const
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);
  auto optimizer_data = make_optimizer_data(input_data);

  auto trajectory_points = trajectory.points;
  if (path_smoother_) {
    autoware_utils_debug::ScopedTimeTrack st_path_smoother(
      path_smoother_->get_name(), *time_keeper_);
    path_smoother_->process(trajectory_points, optimizer_data);
    if (params_.debug.enable_optimizer_trajectory) {
      publish_debug_trajectory(path_smoother_->get_name(), trajectory_points);
    }
  }

  Trajectory traj;
  traj.header = trajectory.header;
  traj.points = trajectory_points;
  return traj;
}

void MinimumRuleBasedPlannerNode::apply_modifiers(
  Trajectory & trajectory, const InputData & input_data) const
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);
  const auto modifier_data = make_modifier_data(input_data);
  for (auto & modifier : modifier_plugins_) {
    autoware_utils_debug::ScopedTimeTrack st_modifier(modifier->get_name(), *time_keeper_);
    modifier->run(trajectory.points, modifier_data);
    modifier->publish_planning_factor();
    if (params_.debug.enable_modifier_trajectory) {
      publish_debug_trajectory(modifier->get_name(), trajectory.points);
    }
  }
}

Trajectory MinimumRuleBasedPlannerNode::optimize_velocity(
  const Trajectory & trajectory, const InputData & input_data,
  const bool update_smoother_state) const
{
  // Distinguish the two calls per cycle (go updates the smoother state, stop does not) in the
  // processing-time tree.
  autoware_utils_debug::ScopedTimeTrack st(
    update_smoother_state ? "optimize_velocity(go)" : "optimize_velocity(stop)", *time_keeper_);

  auto trajectory_points = trajectory.points;

  {
    autoware_utils_debug::ScopedTimeTrack st_smoother("velocity_smoother", *time_keeper_);
    velocity_smoother_->optimize(
      trajectory_points, *input_data.odometry_ptr,
      input_data.acceleration_ptr->accel.accel.linear.x, update_smoother_state);
  }

  // Post-optimization resample
  {
    autoware_utils_debug::ScopedTimeTrack st_resample("post_resample", *time_keeper_);
    autoware::velocity_smoother::resampling::ResampleParam post_resample_param;
    post_resample_param.max_trajectory_length = params_.post_resample.max_trajectory_length;
    post_resample_param.min_trajectory_length = params_.post_resample.min_trajectory_length;
    post_resample_param.resample_time = params_.post_resample.resample_time;
    post_resample_param.dense_resample_dt = params_.post_resample.dense_resample_dt;
    post_resample_param.dense_min_interval_distance =
      params_.post_resample.dense_min_interval_distance;
    post_resample_param.sparse_resample_dt = params_.post_resample.sparse_resample_dt;
    post_resample_param.sparse_min_interval_distance =
      params_.post_resample.sparse_min_interval_distance;

    const auto & ego_pose = input_data.odometry_ptr->pose.pose;
    const double v_current = input_data.odometry_ptr->twist.twist.linear.x;

    trajectory_points = autoware::velocity_smoother::resampling::resampleTrajectory(
      trajectory_points, v_current, ego_pose,
      params_.path_planning.ego_nearest_lanelet.dist_threshold,
      params_.path_planning.ego_nearest_lanelet.yaw_threshold, post_resample_param, false);

    if (!trajectory_points.empty()) {
      trajectory_points.back().longitudinal_velocity_mps = 0.0;
    }
  }

  // Use a strictly increasing time base for MPPI sampling and downstream consumers.
  assign_time_from_start(trajectory_points, input_data.odometry_ptr->pose.pose.position);

  Trajectory traj;
  traj.header = trajectory.header;
  traj.points = trajectory_points;
  return traj;
}

void MinimumRuleBasedPlannerNode::publish_candidate_trajectories(
  const Trajectory & go_trajectory, const std::optional<Trajectory> & stop_trajectory) const
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);

  CandidateTrajectories msg;

  const auto add_generator_info = [&msg](
                                    const UUID & generator_id, const std::string & generator_name) {
    autoware_internal_planning_msgs::msg::GeneratorInfo generator_info;
    generator_info.generator_id = generator_id;
    generator_info.generator_name.data = generator_name;
    msg.generator_info.push_back(generator_info);
  };

  const auto add_candidate = [&msg, &add_generator_info](
                               const UUID & generator_id, const std::string & generator_name,
                               const Trajectory & trajectory) {
    autoware_internal_planning_msgs::msg::CandidateTrajectory candidate_traj;
    candidate_traj.header = trajectory.header;
    candidate_traj.generator_id = generator_id;
    candidate_traj.points = trajectory.points;
    msg.candidate_trajectories.push_back(candidate_traj);

    add_generator_info(generator_id, generator_name);
  };

  add_candidate(go_generator_uuid_, "MinimumRuleBasedPlanner_Go", go_trajectory);

  // NOTE(odashima): the stop generator info must be published even when no distinct stop
  // trajectory exists this cycle. The downstream concatenator buffers candidates per generator id
  // and only overwrites entries whose generator info is present in the message, so the empty
  // entry is what invalidates the previously published stop trajectory; without it the stale stop
  // trajectory would stay buffered and selectable.
  if (stop_trajectory) {
    add_candidate(stop_generator_uuid_, "MinimumRuleBasedPlanner_Stop", *stop_trajectory);
  } else {
    add_generator_info(stop_generator_uuid_, "MinimumRuleBasedPlanner_Stop");
  }

  pub_trajectories_->publish(msg);
}

MinimumRuleBasedPlannerNode::InputData MinimumRuleBasedPlannerNode::take_data()
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);
  InputData input_data;

  if (const auto msg = route_subscriber_.take_data()) {
    if (!msg->segments.empty()) {
      route_ptr_ = msg;
    } else {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "input route is empty, ignoring...");
    }
  }
  input_data.route_ptr = route_ptr_;

  if (const auto msg = vector_map_subscriber_.take_data()) {
    lanelet_map_bin_ptr_ = msg;
  }
  input_data.lanelet_map_bin_ptr = lanelet_map_bin_ptr_;

  if (const auto msg = odometry_subscriber_.take_data()) {
    odometry_ptr_ = msg;
  }
  input_data.odometry_ptr = odometry_ptr_;

  if (const auto msg = acceleration_subscriber_.take_data()) {
    acceleration_ptr_ = msg;
  }
  input_data.acceleration_ptr = acceleration_ptr_;

  if (const auto msg = steering_status_subscriber_.take_data()) {
    steering_status_ptr_ = msg;
  }
  input_data.steering_status_ptr = steering_status_ptr_;

  if (const auto msg = objects_subscriber_.take_data()) {
    predicted_objects_ptr_ = msg;
  }
  input_data.predicted_objects_ptr = predicted_objects_ptr_;

  if (const auto msg = pointcloud_subscriber_.take_data()) {
    obstacle_pointcloud_ptr_ = msg;
  }
  input_data.obstacle_pointcloud_ptr = obstacle_pointcloud_ptr_;

  if (const auto msg = test_path_with_lane_id_subscriber_.take_data()) {
    test_path_with_lane_id_ptr = msg;
  }
  input_data.test_path_with_lane_id_ptr = test_path_with_lane_id_ptr;

  return input_data;
}

bool MinimumRuleBasedPlannerNode::is_data_ready(const InputData & input_data)
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);
  const auto notify_waiting = [this](const std::string & name) {
    RCLCPP_INFO_SKIPFIRST_THROTTLE(
      get_logger(), *get_clock(), 5000, "waiting for %s", name.c_str());
  };

  // NOTE(odashima): on test mode, minimum rule based planner receives path_with_lane_id topic
  if (!input_data.route_ptr && !input_data.test_path_with_lane_id_ptr) {
    notify_waiting("route");
    return false;
  }
  if (!input_data.lanelet_map_bin_ptr) {
    notify_waiting("lanelet map");
    return false;
  }
  if (!input_data.odometry_ptr) {
    notify_waiting("odometry");
    return false;
  }
  if (!input_data.acceleration_ptr) {
    notify_waiting("acceleration");
    return false;
  }
  return true;
}

void MinimumRuleBasedPlannerNode::update_params()
{
  params_ = param_listener_->get_params();
  path_planner_->update_params(params_);

  for (auto & modifier : modifier_plugins_) {
    modifier->update_params(params_);
  }
}

}  // namespace autoware::minimum_rule_based_planner

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::minimum_rule_based_planner::MinimumRuleBasedPlannerNode)
