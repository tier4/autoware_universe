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

#include "autoware/planning_validator_boundary_departure_checker/boundary_departure_checker.hpp"

#include <autoware_utils/ros/parameter.hpp>

#include <memory>
#include <string>
#include <vector>

namespace autoware::planning_validator
{
using autoware_utils::get_or_declare_parameter;

void BoundaryDepartureChecker::init(
  rclcpp::Node & node, const std::string & name,
  const std::shared_ptr<PlanningValidatorContext> & context)
{
  module_name_ = name;
  clock_ = node.get_clock();
  logger_ = node.get_logger();
  context_ = context;

  enable_ = get_or_declare_parameter<bool>(node, "boundary_departure_checker.enable");
  handling_type_ = get_handling_type(
    get_or_declare_parameter<int>(node, "boundary_departure_checker.handling_type"));
  params_.boundary_types_to_detect = get_or_declare_parameter<std::vector<std::string>>(
    node, "boundary_departure_checker.boundary_types");
  params_.lateral_margin_m =
    get_or_declare_parameter<double>(node, "boundary_departure_checker.lateral_margin_m");
  params_.longitudinal_margin_m =
    get_or_declare_parameter<double>(node, "boundary_departure_checker.longitudinal_margin_m");
  params_.max_deceleration_mps2 =
    get_or_declare_parameter<double>(node, "boundary_departure_checker.max_deceleration_mps2");
  params_.max_jerk_mps3 =
    get_or_declare_parameter<double>(node, "boundary_departure_checker.max_jerk_mps3");
  params_.brake_delay_s =
    get_or_declare_parameter<double>(node, "boundary_departure_checker.brake_delay_s");
  params_.time_to_departure_cutoff_s = get_or_declare_parameter<double>(
    node, "boundary_departure_checker.time_to_departure_cutoff_s");
  params_.on_time_buffer_s =
    get_or_declare_parameter<double>(node, "boundary_departure_checker.on_time_buffer_s");
  params_.off_time_buffer_s =
    get_or_declare_parameter<double>(node, "boundary_departure_checker.off_time_buffer_s");

  setup_diag();
}

void BoundaryDepartureChecker::validate()
{
  auto & status = context_->validation_status;
  is_valid_ = true;
  status->is_valid_boundary_departure = true;

  if (!enable_) {
    return;
  }

  const auto & data = context_->data;
  if (!data->current_trajectory || !data->current_kinematics || !data->current_acceleration) {
    return;
  }
  if (!data->route_handler || !data->route_handler->isHandlerReady()) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 3000, "[boundary_departure] map/route is not ready, skip check");
    return;
  }

  const auto lanelet_map = data->route_handler->getLaneletMapPtr();
  if (!lanelet_map || lanelet_map->lineStringLayer.empty()) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 3000, "[boundary_departure] lanelet map has no linestrings, skip check");
    return;
  }

  if (!checker_) {
    checker_ = std::make_unique<autoware::boundary_departure_checker::UncrossableBoundaryChecker>(
      lanelet_map, params_, context_->vehicle_info);
  }

  autoware::boundary_departure_checker::EgoDynamicState ego_state;
  ego_state.pose_with_cov = data->current_kinematics->pose;
  ego_state.velocity = data->current_kinematics->twist.twist.linear.x;
  ego_state.acceleration = data->current_acceleration->accel.accel.linear.x;
  ego_state.current_time_s = rclcpp::Time(data->current_kinematics->header.stamp).seconds();

  const auto result =
    checker_->update_departure_status(data->current_trajectory->points, ego_state);
  const bool is_critical =
    result.status == autoware::boundary_departure_checker::DepartureType::CRITICAL;

  is_valid_ = !is_critical;
  status->is_valid_boundary_departure = is_valid_;

  if (is_critical) {
    context_->set_handling(handling_type_);
    RCLCPP_ERROR_THROTTLE(
      logger_, *clock_, 1000,
      "[boundary_departure] CRITICAL uncrossable-boundary departure on planned trajectory");
  }
}

void BoundaryDepartureChecker::setup_diag()
{
  if (!context_->diag_updater) {
    return;
  }

  context_->diag_updater->add("boundary_departure", [this](auto & stat) {
    const std::string msg = "planned trajectory crosses an uncrossable boundary";
    set_diag_status(stat, is_valid_, msg);
  });
}

void BoundaryDepartureChecker::set_diag_status(
  DiagnosticStatusWrapper & stat, const bool & is_ok, const std::string & msg) const
{
  if (is_ok) {
    stat.summary(DiagnosticStatus::OK, "validated.");
    return;
  }

  const auto invalid_count = context_->validation_status->invalid_count;
  const auto count_threshold = context_->params.diag_error_count_threshold;
  if (invalid_count < count_threshold) {
    const auto warn_msg = msg + " (invalid count is less than error threshold: " +
                          std::to_string(invalid_count) + " < " +
                          std::to_string(count_threshold) + ")";
    stat.summary(DiagnosticStatus::WARN, warn_msg);
    return;
  }

  stat.summary(DiagnosticStatus::ERROR, msg);
}

}  // namespace autoware::planning_validator

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::planning_validator::BoundaryDepartureChecker,
  autoware::planning_validator::PluginInterface)
