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

#include <autoware/boundary_departure_checker/debug.hpp>

#include <string>
#include <vector>

namespace autoware::trajectory_validator::plugin::safety
{
UncrossableBoundaryDepartureFilter::result_t UncrossableBoundaryDepartureFilter::is_feasible(
  const TrajectoryPoints & traj_points, const FilterContext & context)
{
  if (const auto has_invalid_input = is_invalid_input(context)) {
    return tl::make_unexpected(*has_invalid_input);
  }

  if (!is_initialized_) {
    uncrossable_boundary_checker_.set_lanelet_map(context.lanelet_map);
    if (const auto init = uncrossable_boundary_checker_.initialize(); !init) {
      return tl::make_unexpected(init.error());
    }
    is_initialized_ = true;
  }

  boundary_departure_checker::EgoDynamicState ego_state;
  ego_state.pose_with_cov = context.odometry->pose;
  ego_state.velocity = context.odometry->twist.twist.linear.x;
  ego_state.acceleration = context.acceleration->accel.accel.linear.x;
  ego_state.current_time_s = rclcpp::Time(context.odometry->header.stamp).seconds();

  const auto departure_data =
    uncrossable_boundary_checker_.check_departure(traj_points, *vehicle_info_ptr_, ego_state);

  if (!departure_data) {
    return tl::make_unexpected(departure_data.error());
  }

  const bool is_feasible = departure_data->status != boundary_departure_checker::DepartureType::CRITICAL;
  if (!is_feasible) {
    debug_markers_ = boundary_departure_checker::debug::create_debug_markers(
      *departure_data, context.odometry->header.stamp, context.odometry->pose.pose.position.z);
    return tl::make_unexpected("Found critical departure");
  }

  std::vector<MetricReport> metrics{
    autoware_trajectory_validator::build<MetricReport>()
      .validator_name(get_name())
      .validator_category(category())
      .metric_name("check_critical_departure")
      .metric_value(0.0)
      .level(found_critical_departure ? MetricReport::ERROR : MetricReport::OK)};

  return ValidationResult{is_feasible, std::move(metrics)};
}

void UncrossableBoundaryDepartureFilter::update_parameters(const validator::Params & params)
{
  params_.lateral_margin_m = params.boundary_departure.lateral_gap_to_boundary_m;
  params_.longitudinal_margin_m = params.boundary_departure.longitudinal_gap_to_boundary_m;
  params_.max_deceleration_mps2 = params.boundary_departure.max_deceleration_mps2;
  params_.max_jerk_mps3 = params.boundary_departure.max_jerk_mps3;
  params_.brake_delay_s = params.boundary_departure.brake_delay_s;
  params_.time_to_departure_cutoff_s = params.boundary_departure.cutoff_time_s;
  params_.on_time_buffer_s = params.boundary_departure.on_time_buffer_s;
  params_.off_time_buffer_s = params.boundary_departure.off_time_buffer_s;
  params_.boundary_types_to_detect = params.boundary_departure.boundary_types;

  uncrossable_boundary_checker_.set_param(params_);
}

std::optional<std::string> UncrossableBoundaryDepartureFilter::is_invalid_input(
  const FilterContext & context) const
{
  if (!context.lanelet_map || context.lanelet_map->lineStringLayer.empty()) {
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
