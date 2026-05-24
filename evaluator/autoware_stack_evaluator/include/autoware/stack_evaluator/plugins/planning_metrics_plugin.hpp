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

#ifndef AUTOWARE__STACK_EVALUATOR__PLUGINS__PLANNING_METRICS_PLUGIN_HPP_
#define AUTOWARE__STACK_EVALUATOR__PLUGINS__PLANNING_METRICS_PLUGIN_HPP_

#include "autoware/planning_evaluator/metrics/metric.hpp"
#include "autoware/planning_evaluator/metrics/output_metric.hpp"
#include "autoware/planning_evaluator/metrics_accumulator.hpp"
#include "autoware/planning_evaluator/metrics_calculator.hpp"
#include "autoware/planning_evaluator/obstacle_metrics_calculator.hpp"
#include "autoware/stack_evaluator/evaluator_plugin_base.hpp"

#include <autoware/route_handler/route_handler.hpp>
#include <autoware_utils/ros/polling_subscriber.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>

#include <autoware_internal_planning_msgs/msg/planning_factor_array.hpp>
#include <autoware_planning_msgs/msg/pose_with_uuid_stamped.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <tier4_metric_msgs/msg/metric_array.hpp>

#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace autoware::stack_evaluator::plugin
{

class PlanningMetricsPlugin : public EvaluatorPluginBase
{
public:
  PlanningMetricsPlugin();
  ~PlanningMetricsPlugin() override;

  void evaluate(EvaluatorData & data) override;
  void set_up_params() override;
  rcl_interfaces::msg::SetParametersResult on_parameter(
    const std::vector<rclcpp::Parameter> & parameters) override;

private:
  void AddMetricMsg(
    const planning_diagnostics::Metric & metric,
    const autoware_utils::Accumulator<double> & metric_stat);
  void AddObstacleMsg(
    const planning_diagnostics::Metric & metric,
    const autoware_utils::Accumulator<double> & metric_stat,
    const std::string & object_name);
  void AddLaneletMetricMsg(const nav_msgs::msg::Odometry & odometry);
  void AddKinematicStateMetricMsg(
    const geometry_msgs::msg::AccelWithCovarianceStamped & accel_stamped,
    const nav_msgs::msg::Odometry & odometry);

  static bool isFinite(
    const autoware_planning_msgs::msg::TrajectoryPoint & p);

  bool output_metrics_{false};
  std::string ego_frame_str_;

  autoware::vehicle_info_utils::VehicleInfo vehicle_info_;
  planning_diagnostics::MetricsCalculator metrics_calculator_;
  planning_diagnostics::ObstacleMetricsCalculator obstacle_metrics_calculator_;
  planning_diagnostics::MetricsAccumulator metrics_accumulator_;
  autoware::route_handler::RouteHandler route_handler_;

  std::unordered_set<planning_diagnostics::Metric> metrics_for_publish_;
  std::unordered_set<planning_diagnostics::OutputMetric> metrics_for_output_;
  std::unordered_set<std::string> stop_decision_modules_;

  rclcpp::Publisher<tier4_metric_msgs::msg::MetricArray>::SharedPtr metrics_pub_;
  tier4_metric_msgs::msg::MetricArray metrics_msg_;

  std::optional<geometry_msgs::msg::AccelWithCovarianceStamped> prev_acc_stamped_;

  using Trajectory = autoware_planning_msgs::msg::Trajectory;
  using PoseWithUuidStamped = autoware_planning_msgs::msg::PoseWithUuidStamped;
  using PlanningFactorArray =
    autoware_internal_planning_msgs::msg::PlanningFactorArray;

  // Planning-specific subscribers only (not in shared EvaluatorData)
  std::shared_ptr<
    autoware_utils::InterProcessPollingSubscriber<Trajectory>>
    ref_sub_;
  std::shared_ptr<
    autoware_utils::InterProcessPollingSubscriber<PoseWithUuidStamped>>
    modified_goal_sub_;

  std::unordered_map<
    std::string,
    std::shared_ptr<
      autoware_utils::InterProcessPollingSubscriber<PlanningFactorArray>>>
    planning_factors_sub_;
};

}  // namespace autoware::stack_evaluator::plugin

#endif  // AUTOWARE__STACK_EVALUATOR__PLUGINS__PLANNING_METRICS_PLUGIN_HPP_
