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

#include "autoware/trajectory_validator/filters/safety/uncrossable_boundary_departure_filter.hpp"

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::trajectory_validator::plugin::safety
{
UncrossableBoundaryDepartureFilter::result_t UncrossableBoundaryDepartureFilter::is_feasible(
  const TrajectoryPoints & traj_points, const FilterContext & context)
{
  if (const auto has_invalid_input = is_invalid_input(traj_points, context)) {
    return tl::make_unexpected(*has_invalid_input);
  }

  if (!uncrossable_boundary_departure_checker_ptr_) {
    uncrossable_boundary_departure_checker_ptr_ =
      std::make_unique<boundary_departure_checker::UncrossableBoundaryDepartureChecker>(
        clock_, context.lanelet_map, *vehicle_info_ptr_);
  }

  auto departure_data = uncrossable_boundary_departure_checker_ptr_->get_departure_data(
    traj_points, traj_points, context.odometry->pose, context.odometry->twist.twist.linear.x,
    context.acceleration->accel.accel.linear.x);

  if (!departure_data) {
    warn_throttle("%s", departure_data.error().c_str());
    return tl::make_unexpected(departure_data.error());
  }

  bool found_critical_depature = !departure_data->critical_departure_points.empty();

  std::vector<TrajectoryMetricStatus> metrics{
    autoware_internal_planning_msgs::build<TrajectoryMetricStatus>()
      .name("check_critical_departure")
      .level(found_critical_depature ? TrajectoryMetricStatus::ERROR : TrajectoryMetricStatus::OK)
      .score(0.0)};  // To be updated

  return autoware_internal_planning_msgs::build<TrajectoryValidationStatus>()
    .name(get_name())
    .level(
      found_critical_depature ? TrajectoryValidationStatus::ERROR : TrajectoryValidationStatus::OK)
    .metrics(std::move(metrics));
}

void UncrossableBoundaryDepartureFilter::set_parameters([[maybe_unused]] rclcpp::Node & node)
{
  using autoware_utils_rclcpp::get_or_declare_parameter;

  const auto dist_to_boundary =
    get_or_declare_parameter<double>(node, "boundary_departure.lateral_gap_to_boundary_m");
  param_.th_trigger.th_dist_to_boundary_m.left.min = dist_to_boundary;
  param_.th_trigger.th_dist_to_boundary_m.right.min = dist_to_boundary;
  param_.th_trigger.th_acc_mps2.max =
    get_or_declare_parameter<double>(node, "boundary_departure.max_deceleration_mps2");
  param_.th_trigger.th_jerk_mps3.max =
    get_or_declare_parameter<double>(node, "boundary_departure.max_jerk_mps3");
  param_.th_cutoff_time_departure_s =
    get_or_declare_parameter<double>(node, "boundary_departure.cutoff_time_s");
  param_.th_trigger.brake_delay_s =
    get_or_declare_parameter<double>(node, "boundary_departure.brake_delay_s");
  param_.min_braking_distance =
    get_or_declare_parameter<double>(node, "boundary_departure.longitudinal_gap_to_boundary_m");

  if (uncrossable_boundary_departure_checker_ptr_) {
    uncrossable_boundary_departure_checker_ptr_->set_param(param_);
  }
}

void UncrossableBoundaryDepartureFilter::update_parameters(
  const std::vector<rclcpp::Parameter> & parameters)
{
  auto dist_to_boundary = param_.th_trigger.th_dist_to_boundary_m.left.min;
  autoware_utils_rclcpp::update_param(
    parameters, "boundary_departure.lateral_gap_to_boundary_m", dist_to_boundary);
  param_.th_trigger.th_dist_to_boundary_m.left.min = dist_to_boundary;
  param_.th_trigger.th_dist_to_boundary_m.right.min = dist_to_boundary;

  autoware_utils_rclcpp::update_param(
    parameters, "boundary_departure.max_deceleration_mps2", param_.th_trigger.th_acc_mps2.max);
  autoware_utils_rclcpp::update_param(
    parameters, "boundary_departure.max_jerk_mps3", param_.th_trigger.th_jerk_mps3.max);
  autoware_utils_rclcpp::update_param(
    parameters, "boundary_departure.cutoff_time_s", param_.th_cutoff_time_departure_s);
  autoware_utils_rclcpp::update_param(
    parameters, "boundary_departure.brake_delay_s", param_.th_trigger.brake_delay_s);
  autoware_utils_rclcpp::update_param(
    parameters, "boundary_departure.longitudinal_gap_to_boundary_m", param_.min_braking_distance);

  if (uncrossable_boundary_departure_checker_ptr_) {
    uncrossable_boundary_departure_checker_ptr_->set_param(param_);
  }
}

std::optional<std::string> UncrossableBoundaryDepartureFilter::is_invalid_input(
  const TrajectoryPoints & traj_points, const FilterContext & context) const
{
  if (traj_points.empty()) {
    return "Trajectory points are empty.";
  }

  if (!context.lanelet_map) {
    return "Lanelet map is not available in the context.";
  }

  if (!vehicle_info_ptr_) {
    return "Vehicle info is not set.";
  }

  return std::nullopt;
}
}  // namespace autoware::trajectory_validator::plugin::safety

#include <pluginlib/class_list_macros.hpp>
namespace safety = autoware::trajectory_validator::plugin::safety;

PLUGINLIB_EXPORT_CLASS(
  safety::UncrossableBoundaryDepartureFilter,
  autoware::trajectory_validator::plugin::ValidatorInterface)
