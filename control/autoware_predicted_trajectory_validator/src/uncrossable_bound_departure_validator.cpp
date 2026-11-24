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

#include "autoware/predicted_trajectory_validator/uncrossable_bound_departure_validator.hpp"

#include "autoware/predicted_trajectory_validator/utils.hpp"

#include <autoware_utils_geometry/boost_geometry.hpp>

#include <iterator>
#include <memory>
#include <utility>
#include <vector>

namespace autoware::predicted_trajectory_validator
{

void UncrossableBoundDepartureValidator::validate(
  ValidationResult & result, const Trajectory & predicted_trajectory, const Odometry & kinematics,
  const AccelWithCovarianceStamped & acceleration, const lanelet::LaneletMapPtr & lanelet_map,
  const vehicle_info_utils::VehicleInfo & vehicle_info,
  visualization_msgs::msg::MarkerArray & debug_markers, HazardWarningSources & hazard_sources)
{
  if (predicted_trajectory.points.size() < 2) {
    if (enable_warning_) {
      hazard_sources.departure.emplace();
    }
    return;
  }

  if (!lanelet_map || lanelet_map->lineStringLayer.empty()) {
    RCLCPP_DEBUG(logger_, "Lanelet map is not available yet. Skipping boundary departure check.");
    result.is_valid = true;
    return;
  }

  boundary_departure_checker::EgoDynamicState ego_state;
  ego_state.pose_with_cov = kinematics.pose;
  ego_state.velocity = kinematics.twist.twist.linear.x;
  ego_state.acceleration = acceleration.accel.accel.linear.x;
  ego_state.current_time_s = rclcpp::Time(kinematics.header.stamp).seconds();

  if (!enable_) {
    result.is_valid = true;
  } else {
    if (!checker_) {
      checker_ = std::make_unique<boundary_departure_checker::UncrossableBoundaryChecker>(
        lanelet_map, params_, vehicle_info);
    }

    auto status =
      checker_->update_departure_status(predicted_trajectory.points, ego_state, hysteresis_state_);

    result.is_valid = status.status != boundary_departure_checker::DepartureType::CRITICAL;

    if (!result.is_valid) {
      std::move(
        status.debug_markers.markers.begin(), status.debug_markers.markers.end(),
        std::back_inserter(debug_markers.markers));
    }
  }

  if (enable_warning_) {
    if (!warning_checker_) {
      warning_checker_ = std::make_unique<boundary_departure_checker::UncrossableBoundaryChecker>(
        lanelet_map, warning_params_, vehicle_info);
    }
    warning_checker_->update_departure_status(
      predicted_trajectory.points, ego_state, warning_hysteresis_state_);

    std::vector<HazardSource> departure;
    warning_hysteresis_state_.critical_departure_history.for_each_side([&](const auto & side) {
      for (const auto & projection : side) {
        autoware_utils_geometry::Polygon2d point;
        point.outer().push_back(projection.pt_on_bound);
        departure.push_back(
          HazardSource{
            transform_polygon_to_base_link(point, kinematics.pose.pose), departure_risk_level_,
            HazardSector::SOURCE_BOUNDARY_DEPARTURE, kinematics.header.stamp});
      }
    });
    hazard_sources.departure = std::move(departure);
  }
}

}  // namespace autoware::predicted_trajectory_validator
