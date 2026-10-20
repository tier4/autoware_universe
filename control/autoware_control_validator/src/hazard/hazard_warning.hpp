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

#ifndef HAZARD__HAZARD_WARNING_HPP_
#define HAZARD__HAZARD_WARNING_HPP_

#include "hazard/alert_latch.hpp"
#include "hazard/collision_sweep.hpp"

#include <autoware/boundary_departure_checker/detail/data_structs.hpp>
#include <autoware_control_validator/msg/hazard_alert.hpp>
#include <autoware_control_validator/msg/hazard_candidate_array.hpp>
#include <autoware_utils_geometry/boost_geometry.hpp>
#include <autoware_vehicle_info_utils/vehicle_info.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/time.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <vector>

namespace autoware::control_validator::hazard
{
using autoware_control_validator::msg::HazardAlert;
using autoware_control_validator::msg::HazardCandidate;
using autoware_control_validator::msg::HazardCandidateArray;
using autoware_utils_geometry::Polygon2d;

constexpr size_t kNumSectors = 8;

struct HazardWarningParam
{
  std::vector<double> reaction_times;
  double braking_deceleration{6.0};
  double collision_cos_threshold{-1.0};  // -1 disables the collision angle condition
  double min_alert_duration{0.5};
  double rear_axle_ratio{0.3};
  bool enable_road_border{true};
  double sector_radius_m{60.0};
  double sector_boundary_tolerance_deg{1.0};
  /// Bearing ranges [deg] in HazardAlert sector order (FRONT, FRONT_RIGHT, ... FRONT_LEFT).
  std::array<std::array<double, 2>, kNumSectors> sector_ranges{};
};

struct CollisionEvaluation
{
  std::vector<SweepCandidate> candidates;
  std::vector<Point2d> points;  // every point handed to the sweep (map), for debug output
  size_t n_clusters{0};
  rclcpp::Time stamp;
  builtin_interfaces::msg::Time objects_stamp;
  builtin_interfaces::msg::Time pointcloud_stamp;
};

/// Judgement of one candidate, in the same order as CollisionEvaluation::candidates.
struct CandidateJudgement
{
  bool raised{false};
  std::vector<uint8_t> sectors;
  uint8_t level{0};
};

/**
 * @brief Threshold judgement, sector assignment and latching of the hazard warning.
 * Evaluation (sweep, TTC, collision angle) is done by the caller and handed over as const.
 */
class HazardWarning
{
public:
  HazardWarning(
    const rclcpp::Logger & logger, const HazardWarningParam & param,
    const vehicle_info_utils::VehicleInfo & vehicle_info);

  void update_parameters(const HazardWarningParam & param);

  /// false when the parameters are undecided (NaN) or inconsistent; run() then raises nothing.
  bool enabled() const { return enabled_; }

  /**
   * @param judgements when given, receives one entry per candidate (sectors it raised, level)
   */
  HazardAlert run(
    const CollisionEvaluation & evaluation,
    const std::optional<boundary_departure_checker::DepartureResult> & departure,
    const nav_msgs::msg::Odometry & odom, const rclcpp::Time & now,
    std::vector<CandidateJudgement> * judgements = nullptr);

  /// Sector wedges in base_link.
  const std::array<Polygon2d, kNumSectors> & wedges() const { return wedges_; }

  /// Candidate polygon (map) expressed in base_link of the given ego pose.
  static Polygon2d to_base_link(
    const Polygon2d & polygon_map, const geometry_msgs::msg::Pose & ego_pose);

private:
  bool validate_parameters() const;
  void build_wedges();
  bool passes_thresholds(const SweepCandidate & candidate) const;

  rclcpp::Logger logger_;
  HazardWarningParam param_;
  vehicle_info_utils::VehicleInfo vehicle_info_;
  bool enabled_{false};
  std::array<AlertLatch, kNumSectors> sector_latches_;
  AlertLatch departure_latch_;
  std::array<Polygon2d, kNumSectors> wedges_;
  std::array<uint8_t, kNumSectors> last_level_{};
};

/**
 * @brief Numeric debug output of the candidates. Each entry carries the header.stamp of the
 * objects / pointcloud message it came from.
 */
HazardCandidateArray create_candidate_array(
  const CollisionEvaluation & evaluation, const std::vector<CandidateJudgement> & judgements,
  const rclcpp::Time & now);

/**
 * @brief Debug markers (map frame): points handed to the sweep (grey), candidate polygons and
 * colliding points coloured by level, raised sector wedges and the TTC / cos text of each
 * candidate. Marker stamps follow the input the candidate came from (pointcloud or objects);
 * the wedges use the judgement time.
 */
visualization_msgs::msg::MarkerArray create_hazard_markers(
  const CollisionEvaluation & evaluation, const std::vector<CandidateJudgement> & judgements,
  const HazardAlert & alert, const std::array<Polygon2d, kNumSectors> & wedges,
  const nav_msgs::msg::Odometry & odom);
}  // namespace autoware::control_validator::hazard

#endif  // HAZARD__HAZARD_WARNING_HPP_
