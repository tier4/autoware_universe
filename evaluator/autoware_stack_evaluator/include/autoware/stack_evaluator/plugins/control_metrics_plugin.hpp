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

#ifndef AUTOWARE__STACK_EVALUATOR__PLUGINS__CONTROL_METRICS_PLUGIN_HPP_
#define AUTOWARE__STACK_EVALUATOR__PLUGINS__CONTROL_METRICS_PLUGIN_HPP_

#include "autoware/control_evaluator/metrics/metric.hpp"
#include "autoware/stack_evaluator/evaluator_plugin_base.hpp"

#include <autoware/route_handler/route_handler.hpp>
#include <autoware_utils/math/accumulator.hpp>
#include <autoware_utils/ros/polling_subscriber.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>

#include <autoware_internal_planning_msgs/msg/planning_factor_array.hpp>
#include <tier4_metric_msgs/msg/metric_array.hpp>

#include <array>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace autoware::stack_evaluator::plugin
{

class ControlMetricsPlugin : public EvaluatorPluginBase
{
public:
  ControlMetricsPlugin();
  ~ControlMetricsPlugin() override;

  void evaluate(EvaluatorData & data) override;
  void set_up_params() override;
  rcl_interfaces::msg::SetParametersResult on_parameter(
    const std::vector<rclcpp::Parameter> & parameters) override;

private:
  void AddMetricMsg(
    const control_diagnostics::Metric & metric, const double & metric_value,
    const bool & accumulate_metric = true);
  void AddLateralDeviationMetricMsg(
    const autoware_planning_msgs::msg::Trajectory & traj,
    const geometry_msgs::msg::Point & ego_point);
  void AddYawDeviationMetricMsg(
    const autoware_planning_msgs::msg::Trajectory & traj,
    const geometry_msgs::msg::Pose & ego_pose);
  void AddGoalDeviationMetricMsg(const nav_msgs::msg::Odometry & odom);
  void AddObjectMetricMsg(
    const nav_msgs::msg::Odometry & odom,
    const autoware_perception_msgs::msg::PredictedObjects & objects);
  void AddBoundaryDistanceMetricMsg(
    const autoware_internal_planning_msgs::msg::PathWithLaneId & behavior_path,
    const geometry_msgs::msg::Pose & ego_pose);
  void AddUncrossableBoundaryDistanceMetricMsg(
    const geometry_msgs::msg::Pose & ego_pose);
  void AddLaneletInfoMsg(const geometry_msgs::msg::Pose & ego_pose);
  void AddKinematicStateMetricMsg(
    const nav_msgs::msg::Odometry & odom,
    const geometry_msgs::msg::AccelWithCovarianceStamped & accel_stamped);
  void AddSteeringMetricMsg(
    const autoware_vehicle_msgs::msg::SteeringReport & steering_status);
  void AddStopDeviationMetricMsg();
  void AddVelocityDeviationMetricMsg(
    const autoware_planning_msgs::msg::Trajectory & traj,
    const geometry_msgs::msg::Pose & ego_pose,
    const geometry_msgs::msg::Twist & twist);

  bool output_metrics_{false};
  double distance_filter_thr_m_{30.0};
  float ego_speed_{0.0};

  std::array<bool, static_cast<size_t>(control_diagnostics::Metric::SIZE)>
    is_output_metrics_only_moving{};

  autoware::vehicle_info_utils::VehicleInfo vehicle_info_;
  autoware::route_handler::RouteHandler route_handler_;

  using PlanningFactorArray =
    autoware_internal_planning_msgs::msg::PlanningFactorArray;
  std::unordered_map<
    std::string, autoware_utils::InterProcessPollingSubscriber<PlanningFactorArray>>
    planning_factors_sub_;
  std::unordered_map<std::string, autoware_utils::Accumulator<double>>
    stop_deviation_accumulators_;
  std::unordered_map<std::string, autoware_utils::Accumulator<double>>
    stop_deviation_abs_accumulators_;
  std::unordered_set<std::string> stop_deviation_modules_;

  std::array<autoware_utils::Accumulator<double>,
    static_cast<size_t>(control_diagnostics::Metric::SIZE)>
    metric_accumulators_;

  rclcpp::Publisher<tier4_metric_msgs::msg::MetricArray>::SharedPtr metrics_pub_;
  tier4_metric_msgs::msg::MetricArray metrics_msg_;

  std::optional<geometry_msgs::msg::AccelWithCovarianceStamped> prev_acc_stamped_;
  std::optional<double> prev_steering_angle_;
  std::optional<double> prev_steering_rate_;
  std::optional<double> prev_steering_angle_timestamp_;
};

}  // namespace autoware::stack_evaluator::plugin

#endif  // AUTOWARE__STACK_EVALUATOR__PLUGINS__CONTROL_METRICS_PLUGIN_HPP_
