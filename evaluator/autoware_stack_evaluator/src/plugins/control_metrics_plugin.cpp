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

#include "autoware/stack_evaluator/plugins/control_metrics_plugin.hpp"

#include "autoware/control_evaluator/metrics/deviation_metrics.hpp"
#include "autoware/control_evaluator/metrics/metrics_utils.hpp"

#include <autoware/boundary_departure_checker/conversion.hpp>
#include <autoware/boundary_departure_checker/utils.hpp>
#include <autoware/lanelet2_utils/nn_search.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>
#include <autoware_utils/geometry/boost_polygon_utils.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <nlohmann/json.hpp>

#include <boost/geometry.hpp>

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <functional>
#include <limits>
#include <optional>
#include <string>
#include <unordered_set>
#include <vector>

namespace autoware::stack_evaluator::plugin
{
namespace bg = boost::geometry;

ControlMetricsPlugin::ControlMetricsPlugin()
{
  // vehicle_info is set during set_up_params when node is available
}

ControlMetricsPlugin::~ControlMetricsPlugin()
{
  if (!output_metrics_) {
    return;
  }

  try {
    using json = nlohmann::json;
    json j;
    const std::vector<control_diagnostics::Metric> all_metrics = {
      control_diagnostics::Metric::velocity,
      control_diagnostics::Metric::acceleration,
      control_diagnostics::Metric::lateral_acceleration_abs,
      control_diagnostics::Metric::jerk,
      control_diagnostics::Metric::lateral_deviation,
      control_diagnostics::Metric::lateral_deviation_abs,
      control_diagnostics::Metric::yaw_deviation,
      control_diagnostics::Metric::yaw_deviation_abs,
      control_diagnostics::Metric::goal_longitudinal_deviation,
      control_diagnostics::Metric::goal_longitudinal_deviation_abs,
      control_diagnostics::Metric::goal_lateral_deviation,
      control_diagnostics::Metric::goal_lateral_deviation_abs,
      control_diagnostics::Metric::goal_yaw_deviation,
      control_diagnostics::Metric::goal_yaw_deviation_abs,
      control_diagnostics::Metric::left_boundary_distance,
      control_diagnostics::Metric::right_boundary_distance,
      control_diagnostics::Metric::steering_angle,
      control_diagnostics::Metric::steering_angle_abs,
      control_diagnostics::Metric::steering_rate,
      control_diagnostics::Metric::steering_acceleration,
      control_diagnostics::Metric::stop_deviation,
      control_diagnostics::Metric::stop_deviation_abs,
      control_diagnostics::Metric::closest_object_distance,
      control_diagnostics::Metric::longitudinal_velocity_deviation,
    };

    for (const auto & metric : all_metrics) {
      const auto idx = static_cast<size_t>(metric);
      if (metric_accumulators_[idx].count() == 0) {
        continue;
      }
      const std::string base_name =
        control_diagnostics::metric_to_str.at(metric) + "/";
      j[base_name + "min"] = metric_accumulators_[idx].min();
      j[base_name + "max"] = metric_accumulators_[idx].max();
      j[base_name + "mean"] = metric_accumulators_[idx].mean();
      j[base_name + "count"] = metric_accumulators_[idx].count();
      j[base_name + "description"] =
        control_diagnostics::metric_descriptions.at(metric);
    }

    j["stop_deviation/description"] =
      control_diagnostics::metric_descriptions.at(
        control_diagnostics::Metric::stop_deviation);
    for (const auto & [module_name, acc] : stop_deviation_accumulators_) {
      if (acc.count() == 0) continue;
      const std::string base_name = "stop_deviation/" + module_name + "/";
      j[base_name + "min"] = acc.min();
      j[base_name + "max"] = acc.max();
      j[base_name + "mean"] = acc.mean();
      j[base_name + "count"] = acc.count();
    }
    j["stop_deviation_abs/description"] =
      control_diagnostics::metric_descriptions.at(
        control_diagnostics::Metric::stop_deviation_abs);
    for (const auto & [module_name, acc] : stop_deviation_abs_accumulators_) {
      if (acc.count() == 0) continue;
      const std::string base_name = "stop_deviation_abs/" + module_name + "/";
      j[base_name + "min"] = acc.min();
      j[base_name + "max"] = acc.max();
      j[base_name + "mean"] = acc.mean();
      j[base_name + "count"] = acc.count();
    }

    const std::string output_folder_str =
      rclcpp::get_logging_directory().string() + "/autoware_metrics";
    if (!std::filesystem::exists(output_folder_str)) {
      if (!std::filesystem::create_directories(output_folder_str)) {
        RCLCPP_ERROR(
          get_node_ptr()->get_logger(), "Failed to create directories: %s",
          output_folder_str.c_str());
        return;
      }
    }

    std::time_t now_time_t =
      std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::tm * local_time = std::localtime(&now_time_t);
    std::ostringstream oss;
    oss << std::put_time(local_time, "%Y-%m-%d-%H-%M-%S");
    std::string cur_time_str = oss.str();

    const std::string output_file_str =
      output_folder_str + "/autoware_control_evaluator-" + cur_time_str + ".json";
    std::ofstream f(output_file_str);
    if (f.is_open()) {
      f << j.dump(4);
      f.close();
    } else {
      RCLCPP_ERROR(
        get_node_ptr()->get_logger(), "Failed to open file: %s",
        output_file_str.c_str());
    }
  } catch (const std::exception & e) {
    std::cerr << "Exception in ControlMetricsPlugin destructor: " << e.what()
              << std::endl;
  } catch (...) {
    std::cerr << "Unknown exception in ControlMetricsPlugin destructor"
              << std::endl;
  }
}

void ControlMetricsPlugin::set_up_params()
{
  auto node = get_node_ptr();
  vehicle_info_ =
    autoware::vehicle_info_utils::VehicleInfoUtils(*node).getVehicleInfo();

  std::vector<std::string> stop_deviation_modules_list =
    node->declare_parameter<std::vector<std::string>>(
      "control_metrics.planning_factor_metrics.stop_deviation.module_list");
  stop_deviation_modules_ = std::unordered_set<std::string>(
    stop_deviation_modules_list.begin(), stop_deviation_modules_list.end());

  const std::string topic_prefix =
    node->declare_parameter<std::string>(
      "control_metrics.planning_factor_metrics.topic_prefix");
  for (const auto & module_name : stop_deviation_modules_) {
    planning_factors_sub_.emplace(
      module_name,
      autoware_utils::InterProcessPollingSubscriber<PlanningFactorArray>(
        node, topic_prefix + module_name));
    stop_deviation_accumulators_.emplace(
      module_name, autoware_utils::Accumulator<double>());
    stop_deviation_abs_accumulators_.emplace(
      module_name, autoware_utils::Accumulator<double>());
  }

  metrics_pub_ = node->create_publisher<tier4_metric_msgs::msg::MetricArray>(
    "~/control/metrics", 1);

  output_metrics_ =
    node->declare_parameter<bool>("control_metrics.output_metrics");
  distance_filter_thr_m_ =
    node->declare_parameter<double>(
      "control_metrics.object_metrics.distance_filter_thr_m");

  const bool output_metrics_only_moving_enabled =
    node->declare_parameter<bool>(
      "control_metrics.output_metrics_only_moving.enabled");
  const std::vector<std::string> output_metrics_only_moving_metric_list =
    node->declare_parameter<std::vector<std::string>>(
      "control_metrics.output_metrics_only_moving.metric_list");

  is_output_metrics_only_moving.fill(false);
  if (output_metrics_only_moving_enabled) {
    for (const auto & metric_str : output_metrics_only_moving_metric_list) {
      const auto it = control_diagnostics::str_to_metric.find(metric_str);
      if (it != control_diagnostics::str_to_metric.end()) {
        is_output_metrics_only_moving[static_cast<size_t>(it->second)] = true;
      } else {
        RCLCPP_ERROR(
          node->get_logger(),
          "Unknown metric '%s' in output_metrics_only_moving.metric_list",
          metric_str.c_str());
      }
    }
  }
}

rcl_interfaces::msg::SetParametersResult ControlMetricsPlugin::on_parameter(
  const std::vector<rclcpp::Parameter> & /*parameters*/)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  return result;
}

void ControlMetricsPlugin::AddMetricMsg(
  const control_diagnostics::Metric & metric, const double & metric_value,
  const bool & accumulate_metric)
{
  auto metric_id = static_cast<size_t>(metric);
  tier4_metric_msgs::msg::Metric metric_msg;
  metric_msg.name = control_diagnostics::metric_to_str.at(metric);
  metric_msg.value = std::to_string(metric_value);
  metrics_msg_.metric_array.push_back(metric_msg);
  if (
    output_metrics_ && accumulate_metric &&
    (ego_speed_ > 0.001 || !is_output_metrics_only_moving[metric_id])) {
    metric_accumulators_[metric_id].add(metric_value);
  }
}

void ControlMetricsPlugin::AddLateralDeviationMetricMsg(
  const autoware_planning_msgs::msg::Trajectory & traj,
  const geometry_msgs::msg::Point & ego_point)
{
  const double metric_value =
    control_diagnostics::metrics::calcLateralDeviation(traj, ego_point);
  const double metric_value_abs = std::abs(metric_value);
  AddMetricMsg(control_diagnostics::Metric::lateral_deviation, metric_value);
  AddMetricMsg(
    control_diagnostics::Metric::lateral_deviation_abs, metric_value_abs);
}

void ControlMetricsPlugin::AddYawDeviationMetricMsg(
  const autoware_planning_msgs::msg::Trajectory & traj,
  const geometry_msgs::msg::Pose & ego_pose)
{
  const double metric_value =
    control_diagnostics::metrics::calcYawDeviation(traj, ego_pose);
  const double metric_value_abs = std::abs(metric_value);
  AddMetricMsg(control_diagnostics::Metric::yaw_deviation, metric_value);
  AddMetricMsg(control_diagnostics::Metric::yaw_deviation_abs, metric_value_abs);
}

void ControlMetricsPlugin::AddGoalDeviationMetricMsg(
  const nav_msgs::msg::Odometry & odom)
{
  const geometry_msgs::msg::Pose ego_pose = odom.pose.pose;
  const auto & goal_pose = route_handler_.getGoalPose();

  using namespace control_diagnostics::metrics;

  const double longitudinal_deviation_value =
    calcLongitudinalDeviation(goal_pose, ego_pose.position);
  const double longitudinal_deviation_value_abs =
    std::abs(longitudinal_deviation_value);
  const double lateral_deviation_value =
    calcLateralDeviation(goal_pose, ego_pose.position);
  const double lateral_deviation_value_abs = std::abs(lateral_deviation_value);
  const double yaw_deviation_value = calcYawDeviation(goal_pose, ego_pose);
  const double yaw_deviation_value_abs = std::abs(yaw_deviation_value);

  const bool is_ego_stopped_near_goal =
    autoware_utils::calc_distance2d(ego_pose.position, goal_pose.position) <
      3.0 &&
    ego_speed_ < 0.001;

  AddMetricMsg(
    control_diagnostics::Metric::goal_longitudinal_deviation,
    longitudinal_deviation_value, is_ego_stopped_near_goal);
  AddMetricMsg(
    control_diagnostics::Metric::goal_lateral_deviation,
    lateral_deviation_value, is_ego_stopped_near_goal);
  AddMetricMsg(
    control_diagnostics::Metric::goal_yaw_deviation, yaw_deviation_value,
    is_ego_stopped_near_goal);
  AddMetricMsg(
    control_diagnostics::Metric::goal_longitudinal_deviation_abs,
    longitudinal_deviation_value_abs, is_ego_stopped_near_goal);
  AddMetricMsg(
    control_diagnostics::Metric::goal_lateral_deviation_abs,
    lateral_deviation_value_abs, is_ego_stopped_near_goal);
  AddMetricMsg(
    control_diagnostics::Metric::goal_yaw_deviation_abs,
    yaw_deviation_value_abs, is_ego_stopped_near_goal);
}

void ControlMetricsPlugin::AddObjectMetricMsg(
  const nav_msgs::msg::Odometry & odom,
  const autoware_perception_msgs::msg::PredictedObjects & objects)
{
  if (objects.objects.empty()) {
    return;
  }

  const auto ego_polygon = [&]() -> autoware_utils::Polygon2d {
    const autoware_utils::LinearRing2d local_ego_footprint =
      vehicle_info_.createFootprint();
    const autoware_utils::LinearRing2d ego_footprint =
      autoware_utils::transform_vector(
        local_ego_footprint,
        autoware_utils::pose2transform(odom.pose.pose));

    autoware_utils::Polygon2d ego_polygon;
    ego_polygon.outer() = ego_footprint;
    bg::correct(ego_polygon);
    return ego_polygon;
  }();

  double minimum_distance = std::numeric_limits<double>::max();
  for (const auto & object : objects.objects) {
    const double center_distance = autoware_utils::calc_distance2d(
      odom.pose.pose.position,
      object.kinematics.initial_pose_with_covariance.pose.position);
    if (center_distance > distance_filter_thr_m_) {
      continue;
    }

    const auto object_polygon = autoware_utils::to_polygon2d(object);
    const auto distance = bg::distance(ego_polygon, object_polygon);
    if (distance < minimum_distance) {
      minimum_distance = distance;
    }
  }

  if (minimum_distance == std::numeric_limits<double>::max()) {
    return;
  }

  AddMetricMsg(
    control_diagnostics::Metric::closest_object_distance, minimum_distance);
}

void ControlMetricsPlugin::AddBoundaryDistanceMetricMsg(
  const autoware_internal_planning_msgs::msg::PathWithLaneId & behavior_path,
  const geometry_msgs::msg::Pose & ego_pose)
{
  if (!route_handler_.isHandlerReady()) return;

  const auto current_lanelets =
    control_diagnostics::metrics::utils::get_current_lanes(
      route_handler_, ego_pose);
  const auto local_vehicle_footprint = vehicle_info_.createFootprint();
  const auto current_vehicle_footprint = autoware_utils::transform_vector(
    local_vehicle_footprint, autoware_utils::pose2transform(ego_pose));

  if (behavior_path.left_bound.size() >= 1) {
    autoware_utils::LineString2d left_boundary;
    for (const auto & p : behavior_path.left_bound)
      left_boundary.emplace_back(p.x, p.y);
    double distance_to_left_boundary =
      control_diagnostics::metrics::utils::calc_distance_to_line(
        current_vehicle_footprint, left_boundary);

    if (control_diagnostics::metrics::utils::is_point_left_of_line(
          ego_pose.position, behavior_path.left_bound)) {
      distance_to_left_boundary *= -1.0;
    }
    AddMetricMsg(
      control_diagnostics::Metric::left_boundary_distance,
      distance_to_left_boundary);
  }

  if (behavior_path.right_bound.size() >= 1) {
    autoware_utils::LineString2d right_boundary;
    for (const auto & p : behavior_path.right_bound)
      right_boundary.emplace_back(p.x, p.y);
    double distance_to_right_boundary =
      control_diagnostics::metrics::utils::calc_distance_to_line(
        current_vehicle_footprint, right_boundary);

    if (!control_diagnostics::metrics::utils::is_point_left_of_line(
          ego_pose.position, behavior_path.right_bound)) {
      distance_to_right_boundary *= -1.0;
    }
    AddMetricMsg(
      control_diagnostics::Metric::right_boundary_distance,
      distance_to_right_boundary);
  }
}

void ControlMetricsPlugin::AddUncrossableBoundaryDistanceMetricMsg(
  const geometry_msgs::msg::Pose & ego_pose)
{
  namespace bdc_utils = autoware::boundary_departure_checker::utils;
  constexpr auto search_dist_offset{5.0};

  const auto search_distance =
    std::max(
      vehicle_info_.max_longitudinal_offset_m,
      vehicle_info_.max_lateral_offset_m) +
    search_dist_offset;

  auto nearest_left = search_distance;
  auto nearest_right = search_distance;

  if (
    const auto nearby_uncrossable_lines_opt =
      bdc_utils::get_uncrossable_linestrings_near_pose(
        route_handler_.getLaneletMapPtr(), ego_pose, search_distance)) {
    const auto & nearby_uncrossable_lines = *nearby_uncrossable_lines_opt;

    const auto transformed_pose = autoware_utils::pose2transform(ego_pose);
    const auto local_fp = vehicle_info_.createFootprint();
    const auto current_fp =
      autoware_utils::transform_vector(local_fp, transformed_pose);
    const auto side = bdc_utils::get_footprint_sides(current_fp, false, false);

    auto is_overlapping{false};
    for (const auto & nearby_ls : nearby_uncrossable_lines) {
      autoware_utils::LineString2d boundary;
      const auto & basic_ls = nearby_ls.basicLineString();
      boundary.reserve(basic_ls.size());
      for (size_t idx = 0; idx + 1 < basic_ls.size(); ++idx) {
        const auto segment =
          bdc_utils::to_segment_2d(basic_ls[idx], basic_ls[idx + 1]);

        is_overlapping = !boost::geometry::disjoint(current_fp, segment);

        if (is_overlapping) {
          nearest_left = 0.0;
          nearest_right = 0.0;
          break;
        }

        const auto dist_to_left = boost::geometry::distance(segment, side.left);
        const auto dist_to_right =
          boost::geometry::distance(segment, side.right);
        if (dist_to_left < dist_to_right) {
          nearest_left = std::min(dist_to_left, nearest_left);
        } else {
          nearest_right = std::min(dist_to_right, nearest_right);
        }
      }
      if (is_overlapping) break;
    }
  }
  AddMetricMsg(
    control_diagnostics::Metric::left_uncrossable_boundary_distance,
    nearest_left);
  AddMetricMsg(
    control_diagnostics::Metric::right_uncrossable_boundary_distance,
    nearest_right);
}

void ControlMetricsPlugin::AddLaneletInfoMsg(
  const geometry_msgs::msg::Pose & ego_pose)
{
  const auto current_lanelets =
    control_diagnostics::metrics::utils::get_current_lanes(
      route_handler_, ego_pose);
  const auto arc_coordinates =
    lanelet::utils::getArcCoordinates(current_lanelets, ego_pose);
  const auto current_lane_opt =
    autoware::experimental::lanelet2_utils::get_closest_lanelet(
      current_lanelets, ego_pose);
  if (!current_lane_opt) return;
  const auto & current_lane = current_lane_opt.value();

  const std::string base_name = "ego_lane_info/";
  tier4_metric_msgs::msg::Metric metric_msg;

  metric_msg.name = base_name + "lane_id";
  metric_msg.value = std::to_string(current_lane.id());
  metrics_msg_.metric_array.push_back(metric_msg);

  metric_msg.name = base_name + "s";
  metric_msg.value = std::to_string(arc_coordinates.length);
  metrics_msg_.metric_array.push_back(metric_msg);

  metric_msg.name = base_name + "t";
  metric_msg.value = std::to_string(arc_coordinates.distance);
  metrics_msg_.metric_array.push_back(metric_msg);
}

void ControlMetricsPlugin::AddKinematicStateMetricMsg(
  const nav_msgs::msg::Odometry & odom,
  const geometry_msgs::msg::AccelWithCovarianceStamped & accel_stamped)
{
  AddMetricMsg(control_diagnostics::Metric::velocity, odom.twist.twist.linear.x);

  const auto & acc = accel_stamped.accel.accel.linear.x;
  const auto lateral_acc = std::abs(accel_stamped.accel.accel.linear.y);
  AddMetricMsg(control_diagnostics::Metric::acceleration, acc);
  AddMetricMsg(
    control_diagnostics::Metric::lateral_acceleration_abs, lateral_acc);

  const auto jerk = [&]() {
    if (!prev_acc_stamped_.has_value()) {
      prev_acc_stamped_ = accel_stamped;
      return 0.0;
    }
    const auto t = static_cast<double>(accel_stamped.header.stamp.sec) +
                   static_cast<double>(accel_stamped.header.stamp.nanosec) *
                     1e-9;
    const auto prev_t =
      static_cast<double>(prev_acc_stamped_.value().header.stamp.sec) +
      static_cast<double>(prev_acc_stamped_.value().header.stamp.nanosec) *
        1e-9;
    const auto dt = t - prev_t;
    if (dt < std::numeric_limits<double>::epsilon()) return 0.0;

    const auto prev_acc = prev_acc_stamped_.value().accel.accel.linear.x;
    prev_acc_stamped_ = accel_stamped;
    return (acc - prev_acc) / dt;
  }();
  AddMetricMsg(control_diagnostics::Metric::jerk, jerk);
}

void ControlMetricsPlugin::AddSteeringMetricMsg(
  const autoware_vehicle_msgs::msg::SteeringReport & steering_status)
{
  double cur_steering_angle = steering_status.steering_tire_angle;
  double cur_steering_angle_abs = std::abs(cur_steering_angle);
  const double cur_t =
    static_cast<double>(steering_status.stamp.sec) +
    static_cast<double>(steering_status.stamp.nanosec) * 1e-9;
  AddMetricMsg(
    control_diagnostics::Metric::steering_angle, cur_steering_angle);
  AddMetricMsg(
    control_diagnostics::Metric::steering_angle_abs, cur_steering_angle_abs);

  if (!prev_steering_angle_timestamp_.has_value()) {
    prev_steering_angle_timestamp_ = cur_t;
    prev_steering_angle_ = cur_steering_angle;
    return;
  }

  const double dt = cur_t - prev_steering_angle_timestamp_.value();
  if (dt < std::numeric_limits<double>::epsilon()) {
    prev_steering_angle_timestamp_ = cur_t;
    prev_steering_angle_ = cur_steering_angle;
    return;
  }

  const double steering_rate =
    (cur_steering_angle - prev_steering_angle_.value()) / dt;
  AddMetricMsg(control_diagnostics::Metric::steering_rate, steering_rate);

  if (!prev_steering_rate_.has_value()) {
    prev_steering_angle_timestamp_ = cur_t;
    prev_steering_angle_ = cur_steering_angle;
    prev_steering_rate_ = steering_rate;
    return;
  }
  const double steering_acceleration =
    (steering_rate - prev_steering_rate_.value()) / dt;
  AddMetricMsg(
    control_diagnostics::Metric::steering_acceleration, steering_acceleration);

  prev_steering_angle_timestamp_ = cur_t;
  prev_steering_angle_ = cur_steering_angle;
  prev_steering_rate_ = steering_rate;
}

void ControlMetricsPlugin::AddStopDeviationMetricMsg()
{
  using PlanningFactor = autoware_internal_planning_msgs::msg::PlanningFactor;
  const auto get_min_distance_signed =
    [](const PlanningFactorArray::ConstSharedPtr & planning_factors)
    -> std::optional<double> {
    std::optional<double> min_distance = std::nullopt;
    for (const auto & factor : planning_factors->factors) {
      if (factor.behavior == PlanningFactor::STOP) {
        for (const auto & control_point : factor.control_points) {
          const auto cur_distance = control_point.distance;
          if (!min_distance || std::abs(cur_distance) < std::abs(*min_distance)) {
            min_distance = cur_distance;
          }
        }
      }
    }
    return min_distance;
  };

  std::vector<std::pair<std::string, double>> min_distances;
  for (auto & [module_name, planning_factor_sub] : planning_factors_sub_) {
    const auto planning_factors = planning_factor_sub.take_data();
    if (
      !planning_factors || planning_factors->factors.empty() ||
      stop_deviation_modules_.count(module_name) == 0) {
      continue;
    }
    const auto min_distance = get_min_distance_signed(planning_factors);
    if (min_distance) {
      min_distances.emplace_back(module_name, *min_distance);
    }
  }
  if (min_distances.empty()) return;

  const auto min_distance_pair = std::min_element(
    min_distances.begin(), min_distances.end(),
    [](const auto & a, const auto & b) {
      return std::abs(a.second) < std::abs(b.second);
    });

  const auto [closest_module_name, closest_min_distance] = *min_distance_pair;
  const bool is_ego_stopped_near_stop_decision =
    std::abs(closest_min_distance) < 3.0 && ego_speed_ < 0.001;
  if (output_metrics_ && is_ego_stopped_near_stop_decision) {
    stop_deviation_accumulators_[closest_module_name].add(
      closest_min_distance);
    stop_deviation_abs_accumulators_[closest_module_name].add(
      std::abs(closest_min_distance));
  }

  for (const auto & [module_name, min_distance] : min_distances) {
    tier4_metric_msgs::msg::Metric metric_msg;
    metric_msg.name = "stop_deviation/" + module_name;
    metric_msg.value = std::to_string(min_distance);
    metrics_msg_.metric_array.push_back(metric_msg);

    tier4_metric_msgs::msg::Metric metric_msg_abs;
    metric_msg_abs.name = "stop_deviation_abs/" + module_name;
    metric_msg_abs.value = std::to_string(std::abs(min_distance));
    metrics_msg_.metric_array.push_back(metric_msg_abs);
  }
}

void ControlMetricsPlugin::AddVelocityDeviationMetricMsg(
  const autoware_planning_msgs::msg::Trajectory & traj,
  const geometry_msgs::msg::Pose & ego_pose,
  const geometry_msgs::msg::Twist & twist)
{
  const double metric_value =
    control_diagnostics::metrics::calcLongitudinalVelocityDeviation(
      traj, ego_pose, twist.linear.x);
  AddMetricMsg(
    control_diagnostics::Metric::longitudinal_velocity_deviation, metric_value);
}

void ControlMetricsPlugin::evaluate(EvaluatorData & data)
{
  if (!data.odometry) return;

  const auto & odom = *data.odometry;
  const geometry_msgs::msg::Pose ego_pose = odom.pose.pose;
  ego_speed_ = std::abs(odom.twist.twist.linear.x);

  metrics_msg_ = tier4_metric_msgs::msg::MetricArray{};

  AddStopDeviationMetricMsg();

  if (data.objects) {
    AddObjectMetricMsg(odom, *data.objects);
  }

  if (data.acceleration) {
    AddKinematicStateMetricMsg(odom, *data.acceleration);
  }

  if (data.trajectory && !data.trajectory->points.empty()) {
    AddLateralDeviationMetricMsg(*data.trajectory, ego_pose.position);
    AddYawDeviationMetricMsg(*data.trajectory, ego_pose);
    AddVelocityDeviationMetricMsg(
      *data.trajectory, ego_pose, odom.twist.twist);
  }

  // Route/map data
  if (data.route) {
    if (data.route->segments.empty()) {
      RCLCPP_ERROR(
        get_node_ptr()->get_logger(), "input route is empty. ignored");
    } else {
      route_handler_.setRoute(*data.route);
    }
  }
  if (data.lanelet_map_bin) {
    route_handler_.setMap(*data.lanelet_map_bin);
  }

  if (route_handler_.isHandlerReady()) {
    AddLaneletInfoMsg(ego_pose);
    AddGoalDeviationMetricMsg(odom);

    if (data.behavior_path) {
      AddBoundaryDistanceMetricMsg(*data.behavior_path, ego_pose);
    }
    AddUncrossableBoundaryDistanceMetricMsg(ego_pose);
  }

  if (data.steering) {
    AddSteeringMetricMsg(*data.steering);
  }

  metrics_msg_.stamp = get_node_ptr()->now();
  metrics_pub_->publish(metrics_msg_);
}

}  // namespace autoware::stack_evaluator::plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::stack_evaluator::plugin::ControlMetricsPlugin,
  autoware::stack_evaluator::plugin::EvaluatorPluginBase)
