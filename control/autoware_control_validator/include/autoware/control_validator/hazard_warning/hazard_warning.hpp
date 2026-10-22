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

#ifndef AUTOWARE__CONTROL_VALIDATOR__HAZARD_WARNING__HAZARD_WARNING_HPP_
#define AUTOWARE__CONTROL_VALIDATOR__HAZARD_WARNING__HAZARD_WARNING_HPP_

#include "autoware/control_validator/hazard_warning/alert_latch.hpp"
#include "autoware/control_validator/hazard_warning/hazard_warning_sources.hpp"

#include <autoware_control_validator/control_validator_parameters.hpp>
#include <autoware_control_validator/msg/hazard_alert.hpp>
#include <autoware_utils_debug/time_keeper.hpp>
#include <autoware_utils_geometry/boost_geometry.hpp>
#include <autoware_vehicle_info_utils/vehicle_info.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/time.hpp>

#include <visualization_msgs/msg/marker_array.hpp>

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>

namespace autoware::control_validator::hazard
{
using autoware_control_validator::msg::HazardAlert;
using autoware_utils_geometry::Point2d;
using autoware_utils_geometry::Polygon2d;

constexpr size_t kNumSectors = 8;

struct HazardWarningParam
{
  bool enable{true};
  double min_alert_duration{0.5};
  double sector_radius_m{60.0};
  double sector_boundary_tolerance_deg{1.0};
  /// Bearing ranges [deg] in HazardAlert sector order (FRONT, FRONT_RIGHT, ... FRONT_LEFT).
  std::array<std::array<double, 2>, kNumSectors> sector_ranges{};
};

/**
 * @brief Sector assignment and latching of the hazard sources, and assembly of HazardAlert.
 * Collision and departure sources share one latch per sector.
 */
class HazardWarning
{
public:
  /**
   * @param node when given, the debug publisher is created on it; null publishes nothing
   */
  HazardWarning(
    const rclcpp::Logger & logger, const ::control_validator::Params & params,
    rclcpp::Node * node = nullptr,
    std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper =
      std::make_shared<autoware_utils_debug::TimeKeeper>());

  void update_parameters(const ::control_validator::Params & params);

  /// false when the parameters are invalid; make_alert() then returns nullopt.
  bool enabled() const { return enable_; }

  /**
   * @param vehicle_info footprint whose centroid is the apex of the sector wedges
   * @return nullopt when disabled or when neither collision nor departure was evaluated
   */
  std::optional<HazardAlert> make_alert(
    const HazardWarningSources & hazard_sources,
    const vehicle_info_utils::VehicleInfo & vehicle_info, const rclcpp::Time & now);

  /// Sector wedges in base_link.
  const std::array<Polygon2d, kNumSectors> & wedges() const { return wedges_; }

private:
  bool validate_parameters() const;
  void build_wedges(const Point2d & apex);
  void assign_sectors(
    const HazardWarningSources & hazard_sources,
    std::array<uint8_t, kNumSectors> & max_level) const;
  void publish_debug(const HazardWarningSources & hazard_sources, const HazardAlert & alert) const;

  rclcpp::Logger logger_;
  std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper_;
  HazardWarningParam param_;
  bool enable_{false};
  bool publish_markers_{true};
  std::optional<Point2d> wedge_apex_;
  std::array<Polygon2d, kNumSectors> wedges_;
  std::array<AlertLatch, kNumSectors> sector_latches_;
  std::array<uint8_t, kNumSectors> last_level_{};
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;
};
}  // namespace autoware::control_validator::hazard

#endif  // AUTOWARE__CONTROL_VALIDATOR__HAZARD_WARNING__HAZARD_WARNING_HPP_
