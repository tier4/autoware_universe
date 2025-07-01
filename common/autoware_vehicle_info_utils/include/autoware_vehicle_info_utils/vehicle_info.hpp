// Copyright 2015-2021 Autoware Foundation
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

#ifndef AUTOWARE_VEHICLE_INFO_UTILS__VEHICLE_INFO_HPP_
#define AUTOWARE_VEHICLE_INFO_UTILS__VEHICLE_INFO_HPP_

#include <autoware_utils/geometry/boost_geometry.hpp>

namespace autoware::vehicle_info_utils
{
/// Data class for vehicle info
struct VehicleInfo
{
  // Base parameters. These describe the vehicle's bounding box and the
  // position and radius of the wheels.
  double wheel_radius_m{};       //<! should be positive
  double wheel_width_m{};        //<! should be positive
  double wheel_base_m{};         //<! should be positive
  double wheel_tread_m{};        //<! should be positive
  double front_overhang_m{};     //<! should be positive
  double rear_overhang_m{};      //<! should be positive
  double left_overhang_m{};      //<! should be positive
  double right_overhang_m{};     //<! should be positive
  double vehicle_height_m{};     //<! should be positive
  double max_steer_angle_rad{};  //<! should be positive

  // Derived parameters, i.e. calculated from base parameters
  // The offset values are relative to the base frame origin, which is located
  // on the ground below the middle of the rear axle, and can be negative.
  double vehicle_length_m{};
  double vehicle_width_m{};
  double min_longitudinal_offset_m{};
  double max_longitudinal_offset_m{};
  double min_lateral_offset_m{};
  double max_lateral_offset_m{};
  double min_height_offset_m{};
  double max_height_offset_m{};

  static constexpr size_t FrontLeftIndex = 0;   //<! the point index of front-left edge == 0
  static constexpr size_t FrontRightIndex = 1;  //<! the point index of front-right > front-left
  static constexpr size_t RearRightIndex = 3;   //<! the point index of rear-right > front-right
  static constexpr size_t RearLeftIndex = 4;    //<! the point index of rear-left > rear-right

  /**
   * @brief calculate the vehicle footprint in clockwise manner starting from the front-left edge,
   * through front-right edge, center-right point, to front-left edge again to form a enclosed
   * polygon
   * @param margin the longitudinal and lateral inflation margin
   */
  autoware_utils::LinearRing2d createFootprint(const double margin = 0.0) const;

  /**
   * @brief calculate the vehicle footprint in clockwise manner starting from the front-left edge,
   * through front-right edge, center-right point, to front-left edge again to form a enclosed
   * polygon
   * @param margin the longitudinal and lateral inflation margin
   */
  autoware_utils::LinearRing2d createFootprint(
    const double lat_margin, const double lon_margin) const;

  /**
   * @brief Calculate the vehicle footprint in a clockwise manner, starting from the front-left
   * edge. The polygon is formed by tracing from front-left edge → front-right edge → center-right
   * point → rear-right edge → rear-left edge → center-left point → front-left edge to form a closed
   * shape.
   * @param front_lat_margin lateral inflation margin at the front section
   * @param center_lat_margin lateral inflation margin at the center section
   * @param rear_lat_margin lateral inflation margin at the rear section
   * @param front_lon_margin longitudinal inflation margin at the front section
   * @param rear_lon_margin longitudinal inflation margin at the rear section
   * @param center_at_base_link if true, center point is aligned at base_link (x=0), otherwise
   * placed at wheelbase center
   */
  [[nodiscard]] autoware_utils::LinearRing2d createFootprint(
    const double front_lat_margin, const double center_lat_margin, const double rear_lat_margin,
    const double front_lon_margin, const double rear_lon_margin,
    const bool center_at_base_link = false) const;

  double calcMaxCurvature() const;
  double calcCurvatureFromSteerAngle(const double steer_angle) const;
  double calcSteerAngleFromCurvature(const double curvature) const;
};

/// Create vehicle info from base parameters
VehicleInfo createVehicleInfo(
  const double wheel_radius_m, const double wheel_width_m, const double wheel_base_m,
  const double wheel_tread_m, const double front_overhang_m, const double rear_overhang_m,
  const double left_overhang_m, const double right_overhang_m, const double vehicle_height_m,
  const double max_steer_angle_rad);

}  // namespace autoware::vehicle_info_utils

#endif  // AUTOWARE_VEHICLE_INFO_UTILS__VEHICLE_INFO_HPP_
