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

#include "autoware/stack_evaluator/plugins/planning_metrics_plugin.hpp"

#include "autoware/planning_evaluator/metrics/output_metric.hpp"

#include <autoware/lanelet2_utils/nn_search.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <nlohmann/json.hpp>

#include <boost/lexical_cast.hpp>

#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <string>
#include <unordered_set>
#include <vector>

namespace autoware::stack_evaluator::plugin
{

PlanningMetricsPlugin::PlanningMetricsPlugin()
{
}

PlanningMetricsPlugin::~PlanningMetricsPlugin()
{
  if (!output_metrics_) {
    return;
  }

  try {
    using json = nlohmann::json;
    json output_json;
    for (planning_diagnostics::OutputMetric metric : metrics_for_output_) {
      const json j = metrics_accumulator_.getOutputJson(metric);
      if (!j.empty()) {
        output_json[planning_diagnostics::output_metric_to_str.at(metric)] = j;
      }
    }

    const std::string output_folder_str =
      rclcpp::get_logging_directory().string() + "/autoware_metrics";
    if (!std::filesystem::exists(output_folder_str)) {
      if (!std::filesystem::create_directories(output_folder_str)) {
        RCLCPP_ERROR(
          get_node_ptr()->get_logger(),
          "Failed to create directories: %s", output_folder_str.c_str());
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
      output_folder_str + "/autoware_planning_evaluator-" + cur_time_str +
      ".json";
    std::ofstream f(output_file_str);
    if (f.is_open()) {
      f << output_json.dump(4);
      f.close();
    } else {
      RCLCPP_ERROR(
        get_node_ptr()->get_logger(), "Failed to open file: %s",
        output_file_str.c_str());
    }
  } catch (const std::exception & e) {
    std::cerr << "Exception in PlanningMetricsPlugin destructor: " << e.what()
              << std::endl;
  } catch (...) {
    std::cerr << "Unknown exception in PlanningMetricsPlugin destructor"
              << std::endl;
  }
}

void PlanningMetricsPlugin::set_up_params()
{
  auto node = get_node_ptr();

  vehicle_info_ =
    autoware::vehicle_info_utils::VehicleInfoUtils(*node).getVehicleInfo();

  metrics_calculator_.setVehicleInfo(vehicle_info_);
  obstacle_metrics_calculator_.setVehicleInfo(vehicle_info_);

  metrics_calculator_.parameters.trajectory.min_point_dist_m =
    node->declare_parameter<double>(
      "planning_metrics.trajectory.min_point_dist_m");
  metrics_calculator_.parameters.trajectory.lookahead.max_dist_m =
    node->declare_parameter<double>(
      "planning_metrics.trajectory.lookahead.max_dist_m");
  metrics_calculator_.parameters.trajectory.lookahead.max_time_s =
    node->declare_parameter<double>(
      "planning_metrics.trajectory.lookahead.max_time_s");
  metrics_calculator_.parameters.trajectory.evaluation_time_s =
    node->declare_parameter<double>(
      "planning_metrics.trajectory.evaluation_time_s");

  obstacle_metrics_calculator_.parameters.worst_only =
    node->declare_parameter<bool>("planning_metrics.obstacle.worst_only");
  obstacle_metrics_calculator_.parameters.use_ego_traj_vel =
    node->declare_parameter<bool>(
      "planning_metrics.obstacle.use_ego_traj_vel");
  obstacle_metrics_calculator_.parameters.collision_thr_m =
    node->declare_parameter<double>(
      "planning_metrics.obstacle.collision_thr_m");
  obstacle_metrics_calculator_.parameters.stop_velocity_mps =
    node->declare_parameter<double>(
      "planning_metrics.obstacle.stop_velocity_mps");
  obstacle_metrics_calculator_.parameters.min_time_interval_s =
    node->declare_parameter<double>(
      "planning_metrics.obstacle.min_time_interval_s");
  obstacle_metrics_calculator_.parameters.min_spatial_interval_m =
    node->declare_parameter<double>(
      "planning_metrics.obstacle.min_spatial_interval_m");
  obstacle_metrics_calculator_.parameters.limit_min_accel =
    node->declare_parameter<double>("planning_metrics.limit.min_acc");

  metrics_accumulator_.planning_factor_accumulator.parameters
    .time_count_threshold_s =
    node->declare_parameter<double>(
      "planning_metrics.stop_decision.time_count_threshold_s");
  metrics_accumulator_.planning_factor_accumulator.parameters
    .dist_count_threshold_m =
    node->declare_parameter<double>(
      "planning_metrics.stop_decision.dist_count_threshold_m");
  metrics_accumulator_.planning_factor_accumulator.parameters
    .abnormal_deceleration_threshold_mps2 =
    node->declare_parameter<double>(
      "planning_metrics.stop_decision.abnormal_deceleration_threshold_mps2");

  metrics_accumulator_.steer_accumulator.parameters.window_duration_s =
    node->declare_parameter<double>(
      "planning_metrics.steer_change_count.window_duration_s");
  metrics_accumulator_.steer_accumulator.parameters.steer_rate_margin =
    node->declare_parameter<double>(
      "planning_metrics.steer_change_count.steer_rate_margin");

  metrics_accumulator_.blinker_accumulator.parameters.window_duration_s =
    node->declare_parameter<double>(
      "planning_metrics.blinker_change_count.window_duration_s");

  output_metrics_ =
    node->declare_parameter<bool>("planning_metrics.output_metrics");
  ego_frame_str_ =
    node->declare_parameter<std::string>("planning_metrics.ego_frame");

  for (const std::string & metric_name :
       node->declare_parameter<std::vector<std::string>>(
         "planning_metrics.metrics_for_publish")) {
    auto it = planning_diagnostics::str_to_metric.find(metric_name);
    if (it != planning_diagnostics::str_to_metric.end()) {
      planning_diagnostics::Metric metric = it->second;
      metrics_for_publish_.insert(metric);
      obstacle_metrics_calculator_.setMetricNeed(metric, true);
    }
  }

  for (const std::string & metric_name :
       node->declare_parameter<std::vector<std::string>>(
         "planning_metrics.metrics_for_output")) {
    auto it =
      planning_diagnostics::str_to_output_metric.find(metric_name);
    if (it != planning_diagnostics::str_to_output_metric.end()) {
      metrics_for_output_.insert(it->second);
    }
  }

  std::vector<std::string> stop_decision_modules_list =
    node->declare_parameter<std::vector<std::string>>(
      "planning_metrics.stop_decision.module_list");
  stop_decision_modules_ = std::unordered_set<std::string>(
    stop_decision_modules_list.begin(), stop_decision_modules_list.end());

  const std::string topic_prefix =
    node->declare_parameter<std::string>(
      "planning_metrics.stop_decision.topic_prefix");
  for (const auto & module_name : stop_decision_modules_) {
    planning_factors_sub_.emplace(
      module_name,
      std::make_shared<
        autoware_utils::InterProcessPollingSubscriber<PlanningFactorArray>>(
        node, topic_prefix + module_name));
  }

  metrics_pub_ = node->create_publisher<tier4_metric_msgs::msg::MetricArray>(
    "~/planning/metrics", 1);

  // Planning-specific subscribers (not in shared EvaluatorData)
  ref_sub_ = std::make_shared<
    autoware_utils::InterProcessPollingSubscriber<Trajectory>>(
    node, "~/input/reference_trajectory");
  modified_goal_sub_ = std::make_shared<
    autoware_utils::InterProcessPollingSubscriber<PoseWithUuidStamped>>(
    node, "~/input/modified_goal");
}

rcl_interfaces::msg::SetParametersResult PlanningMetricsPlugin::on_parameter(
  const std::vector<rclcpp::Parameter> & /*parameters*/)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  return result;
}

void PlanningMetricsPlugin::AddMetricMsg(
  const planning_diagnostics::Metric & metric,
  const autoware_utils::Accumulator<double> & metric_stat)
{
  const std::string base_name =
    planning_diagnostics::metric_to_str.at(metric) + "/";
  tier4_metric_msgs::msg::Metric metric_msg;

  metric_msg.name = base_name + "min";
  metric_msg.value =
    boost::lexical_cast<decltype(metric_msg.value)>(metric_stat.min());
  metrics_msg_.metric_array.push_back(metric_msg);

  metric_msg.name = base_name + "max";
  metric_msg.value =
    boost::lexical_cast<decltype(metric_msg.value)>(metric_stat.max());
  metrics_msg_.metric_array.push_back(metric_msg);

  metric_msg.name = base_name + "mean";
  metric_msg.value =
    boost::lexical_cast<decltype(metric_msg.value)>(metric_stat.mean());
  metrics_msg_.metric_array.push_back(metric_msg);
}

void PlanningMetricsPlugin::AddObstacleMsg(
  const planning_diagnostics::Metric & metric,
  const autoware_utils::Accumulator<double> & metric_stat,
  const std::string & object_name)
{
  const std::string base_name =
    planning_diagnostics::metric_to_str.at(metric) + "/";
  tier4_metric_msgs::msg::Metric metric_msg;
  metric_msg.name = base_name + object_name;
  metric_msg.value =
    boost::lexical_cast<decltype(metric_msg.value)>(metric_stat.min());
  metrics_msg_.metric_array.push_back(metric_msg);
}

void PlanningMetricsPlugin::AddLaneletMetricMsg(
  const nav_msgs::msg::Odometry & odometry)
{
  const auto & ego_pose = odometry.pose.pose;
  const auto current_lanelets = [&]() {
    lanelet::ConstLanelet closest_route_lanelet;
    route_handler_.getClosestLaneletWithinRoute(
      ego_pose, &closest_route_lanelet);
    const auto shoulder_lanelets =
      route_handler_.getShoulderLaneletsAtPose(ego_pose);
    lanelet::ConstLanelets closest_lanelets{closest_route_lanelet};
    closest_lanelets.insert(
      closest_lanelets.end(), shoulder_lanelets.begin(),
      shoulder_lanelets.end());
    return closest_lanelets;
  }();
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

void PlanningMetricsPlugin::AddKinematicStateMetricMsg(
  const geometry_msgs::msg::AccelWithCovarianceStamped & accel_stamped,
  const nav_msgs::msg::Odometry & odometry)
{
  const std::string base_name = "kinematic_state/";
  tier4_metric_msgs::msg::Metric metric_msg;

  metric_msg.name = base_name + "vel";
  metric_msg.value = std::to_string(odometry.twist.twist.linear.x);
  metrics_msg_.metric_array.push_back(metric_msg);

  metric_msg.name = base_name + "acc";
  const auto & acc = accel_stamped.accel.accel.linear.x;
  metric_msg.value = std::to_string(acc);
  metrics_msg_.metric_array.push_back(metric_msg);

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
  metric_msg.name = base_name + "jerk";
  metric_msg.value = std::to_string(jerk);
  metrics_msg_.metric_array.push_back(metric_msg);
}

bool PlanningMetricsPlugin::isFinite(
  const autoware_planning_msgs::msg::TrajectoryPoint & point)
{
  const auto & o = point.pose.orientation;
  const auto & p = point.pose.position;
  const auto & v = point.longitudinal_velocity_mps;
  const auto & w = point.lateral_velocity_mps;
  const auto & a = point.acceleration_mps2;
  const auto & z = point.heading_rate_rps;
  const auto & f = point.front_wheel_angle_rad;
  const auto & r = point.rear_wheel_angle_rad;

  return std::isfinite(o.x) && std::isfinite(o.y) && std::isfinite(o.z) &&
         std::isfinite(o.w) && std::isfinite(p.x) && std::isfinite(p.y) &&
         std::isfinite(p.z) && std::isfinite(v) && std::isfinite(w) &&
         std::isfinite(a) && std::isfinite(z) && std::isfinite(f) &&
         std::isfinite(r);
}

void PlanningMetricsPlugin::evaluate(EvaluatorData & data)
{
  if (!data.odometry) return;

  metrics_msg_ = tier4_metric_msgs::msg::MetricArray{};

  const auto & odom = *data.odometry;

  // Set ego state on all calculators
  metrics_calculator_.setEgoPose(odom);
  metrics_accumulator_.setEgoPose(odom);
  obstacle_metrics_calculator_.setEgoPose(odom);

  // Route data
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
    AddLaneletMetricMsg(odom);
  }

  if (data.acceleration) {
    AddKinematicStateMetricMsg(*data.acceleration, odom);
  }

  // Objects
  if (data.objects) {
    obstacle_metrics_calculator_.setPredictedObjects(*data.objects);
  }

  // Reference trajectory
  {
    const auto ref_traj_msg = ref_sub_->take_data();
    if (ref_traj_msg) {
      metrics_calculator_.setReferenceTrajectory(*ref_traj_msg);
    }
  }

  // Trajectory metrics
  if (data.trajectory && !data.trajectory->points.empty()) {
    for (planning_diagnostics::Metric metric : metrics_for_publish_) {
      const auto metric_stat =
        metrics_calculator_.calculate(metric, *data.trajectory);
      if (!metric_stat || metric_stat->count() <= 0) continue;
      AddMetricMsg(metric, *metric_stat);
      if (output_metrics_) {
        const planning_diagnostics::OutputMetric output_metric =
          planning_diagnostics::str_to_output_metric.at(
            planning_diagnostics::metric_to_str.at(metric));
        metrics_accumulator_.accumulate(output_metric, *metric_stat);
      }
    }

    metrics_calculator_.setPreviousTrajectory(*data.trajectory);

    obstacle_metrics_calculator_.setTrajectory(*data.trajectory);
    obstacle_metrics_calculator_.calculateMetrics();

    for (const auto & metric :
         obstacle_metrics_calculator_.obstacle_metric_types) {
      std::vector<
        std::pair<std::string, autoware_utils::Accumulator<double>>>
        metrics_per_object =
          obstacle_metrics_calculator_.getMetric(metric);
      if (metrics_per_object.empty()) continue;

      for (const auto & [obj_id, metric_stat] : metrics_per_object) {
        AddObstacleMsg(metric, metric_stat, obj_id);
        if (output_metrics_ && obj_id == "worst") {
          const planning_diagnostics::OutputMetric output_metric =
            planning_diagnostics::str_to_output_metric.at(
              planning_diagnostics::metric_to_str.at(metric));
          metrics_accumulator_.accumulate(output_metric, metric_stat);
        }
      }
    }

    obstacle_metrics_calculator_.clearData();
  }

  // Modified goal
  {
    const auto modified_goal_msg = modified_goal_sub_->take_data();
    if (modified_goal_msg) {
      const auto is_ego_stopped_near_goal =
        std::abs(odom.twist.twist.linear.x) < 0.001 &&
        autoware_utils::calc_distance2d(
          odom.pose.pose.position, modified_goal_msg->pose.position) < 3.0;

      for (planning_diagnostics::Metric metric : metrics_for_publish_) {
        const auto metric_stat = metrics_calculator_.calculate(
          metric, modified_goal_msg->pose, odom.pose.pose);
        if (!metric_stat || metric_stat->count() <= 0) continue;
        AddMetricMsg(metric, *metric_stat);
        if (output_metrics_ && is_ego_stopped_near_goal) {
          const planning_diagnostics::OutputMetric output_metric =
            planning_diagnostics::str_to_output_metric.at(
              planning_diagnostics::metric_to_str.at(metric));
          metrics_accumulator_.accumulate(output_metric, *metric_stat);
        }
      }
    }
  }

  // Steering
  if (data.steering) {
    metrics_accumulator_.setSteerData(*data.steering);
    if (metrics_for_publish_.count(
          planning_diagnostics::Metric::steer_change_count) != 0) {
      metrics_accumulator_.addMetricMsg(
        planning_diagnostics::Metric::steer_change_count, metrics_msg_);
    }
  }

  // Blinker
  if (data.blinker) {
    metrics_accumulator_.setBlinkerData(*data.blinker);
    if (metrics_for_publish_.count(
          planning_diagnostics::Metric::blinker_change_count) != 0) {
      metrics_accumulator_.addMetricMsg(
        planning_diagnostics::Metric::blinker_change_count, metrics_msg_);
    }
  }

  // Planning factors (per-module)
  for (auto & [module_name, planning_factor_sub] : planning_factors_sub_) {
    const auto planning_factors = planning_factor_sub->take_data();
    if (!planning_factors || planning_factors->factors.empty()) continue;
    metrics_accumulator_.setPlanningFactors(
      module_name, *planning_factors);

    if (metrics_for_publish_.count(
          planning_diagnostics::Metric::stop_decision) != 0) {
      metrics_accumulator_.addMetricMsg(
        planning_diagnostics::Metric::stop_decision, metrics_msg_,
        module_name);
    }
    if (metrics_for_publish_.count(
          planning_diagnostics::Metric::abnormal_stop_decision) != 0) {
      metrics_accumulator_.addMetricMsg(
        planning_diagnostics::Metric::abnormal_stop_decision,
        metrics_msg_, module_name);
    }
  }

  metrics_msg_.stamp = get_node_ptr()->now();
  metrics_pub_->publish(metrics_msg_);
}

}  // namespace autoware::stack_evaluator::plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::stack_evaluator::plugin::PlanningMetricsPlugin,
  autoware::stack_evaluator::plugin::EvaluatorPluginBase)
