// Copyright 2023-2026 TIER IV, Inc.
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

#ifndef AUTOWARE__PREDICTED_TRAJECTORY_VALIDATOR__UNCROSSABLE_BOUND_DEPARTURE_VALIDATOR_HPP_
#define AUTOWARE__PREDICTED_TRAJECTORY_VALIDATOR__UNCROSSABLE_BOUND_DEPARTURE_VALIDATOR_HPP_

#include "autoware/predicted_trajectory_validator/hazard_warning/hazard_warning_sources.hpp"
#include "autoware_vehicle_info_utils/vehicle_info.hpp"

#include <autoware/boundary_departure_checker/uncrossable_boundary_checker.hpp>
#include <autoware_predicted_trajectory_validator/msg/validation_result.hpp>
#include <autoware_predicted_trajectory_validator/predicted_trajectory_validator_parameters.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_control_msgs/msg/hazard_sector.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <lanelet2_core/LaneletMap.h>

#include <cstdint>
#include <memory>

namespace autoware::predicted_trajectory_validator
{
namespace boundary_departure_checker = autoware::boundary_departure_checker;

using autoware_internal_control_msgs::msg::HazardSector;
using autoware_planning_msgs::msg::Trajectory;
using autoware_predicted_trajectory_validator::msg::ValidationResult;
using geometry_msgs::msg::AccelWithCovarianceStamped;
using nav_msgs::msg::Odometry;

/**
 * @class UncrossableBoundDepartureValidator
 * @brief Validates that the control predicted trajectory does not critically depart an uncrossable
 * map boundary (e.g. road_border), reusing the shared boundary departure checker library.
 */
class UncrossableBoundDepartureValidator
{
public:
  UncrossableBoundDepartureValidator(
    const rclcpp::Logger & logger, const ::predicted_trajectory_validator::Params & params)
  : logger_{logger}
  {
    update_parameters(params);
  }

  void update_parameters(const ::predicted_trajectory_validator::Params & params)
  {
    enable_ = params.uncrossable_bound_departure_validator.enable;
    params_.critical_departure_lateral_th_m =
      params.uncrossable_bound_departure_validator.lateral_margin_m;
    params_.longitudinal_margin_m =
      params.uncrossable_bound_departure_validator.longitudinal_margin_m;
    params_.max_deceleration_mps2 =
      params.uncrossable_bound_departure_validator.max_deceleration_mps2;
    params_.max_jerk_mps3 = params.uncrossable_bound_departure_validator.max_jerk_mps3;
    params_.brake_delay_s = params.uncrossable_bound_departure_validator.brake_delay_s;
    params_.time_to_departure_cutoff_s =
      params.uncrossable_bound_departure_validator.time_to_departure_cutoff_s;
    params_.on_time_buffer_s = params.uncrossable_bound_departure_validator.on_time_buffer_s;
    params_.off_time_buffer_s = params.uncrossable_bound_departure_validator.off_time_buffer_s;
    params_.enable_align_to_ego_pose =
      params.uncrossable_bound_departure_validator.enable_align_to_ego_pose;
    params_.enable_developer_marker =
      params.uncrossable_bound_departure_validator.enable_developer_marker;
    params_.boundary_types_to_detect = params.uncrossable_bound_departure_validator.boundary_types;
    const auto & p = params.uncrossable_bound_departure_validator;
    enable_warning_ = p.enable_warning;
    warning_params_ = params_;
    warning_params_.critical_departure_lateral_th_m = p.warning_lateral_th_m;
    warning_params_.time_to_departure_cutoff_s = p.warning_time_to_departure_cutoff_s;
    warning_params_.brake_delay_s = p.warning_brake_delay_s;
    warning_params_.on_time_buffer_s = p.warning_on_time_buffer_s;
    warning_params_.off_time_buffer_s = p.warning_off_time_buffer_s;
    departure_risk_level_ = static_cast<uint8_t>(p.departure_risk_level);
    if (checker_) {
      checker_->update_parameters(params_);
    }
    if (warning_checker_) {
      warning_checker_->update_parameters(warning_params_);
    }
  }

  /**
   * @brief Validate the predicted trajectory against uncrossable boundaries.
   * @param result is_valid is left untouched when the predicted trajectory has less than 2 points
   * @param hazard_sources departure positions of the warning checker's hysteresis are written to
   *   its departure field when the check ran and the departure warning is enabled; left untouched
   *   when the predicted trajectory has less than 2 points
   */
  void validate(
    ValidationResult & result, const Trajectory & predicted_trajectory, const Odometry & kinematics,
    const AccelWithCovarianceStamped & acceleration, const lanelet::LaneletMapPtr & lanelet_map,
    const vehicle_info_utils::VehicleInfo & vehicle_info,
    visualization_msgs::msg::MarkerArray & debug_markers, HazardWarningSources & hazard_sources);

private:
  bool enable_{true};
  rclcpp::Logger logger_;
  boundary_departure_checker::UncrossableBoundaryDepartureParam params_;
  std::unique_ptr<boundary_departure_checker::UncrossableBoundaryChecker> checker_;
  boundary_departure_checker::HysteresisState hysteresis_state_;
  bool enable_warning_{true};
  boundary_departure_checker::UncrossableBoundaryDepartureParam warning_params_;
  std::unique_ptr<boundary_departure_checker::UncrossableBoundaryChecker> warning_checker_;
  boundary_departure_checker::HysteresisState warning_hysteresis_state_;
  uint8_t departure_risk_level_{HazardSector::NONE};
};
}  // namespace autoware::predicted_trajectory_validator

#endif  // AUTOWARE__PREDICTED_TRAJECTORY_VALIDATOR__UNCROSSABLE_BOUND_DEPARTURE_VALIDATOR_HPP_
