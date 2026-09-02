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

#ifndef AUTOWARE__TRAJECTORY_PROCESSOR__TIME_SEQUENCE_RAW__ROAD_BORDER_AVOIDANCE_HPP_
#define AUTOWARE__TRAJECTORY_PROCESSOR__TIME_SEQUENCE_RAW__ROAD_BORDER_AVOIDANCE_HPP_

#include <autoware/vehicle_info_utils/vehicle_info.hpp>
#include <autoware_utils_geometry/boost_geometry.hpp>

#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <lanelet2_core/LaneletMap.h>

#include <cstddef>
#include <vector>

namespace autoware::trajectory_processor::time_sequence_raw
{
using autoware_planning_msgs::msg::Trajectory;

struct RoadBorderAvoidanceParams
{
  bool enable{false};
  double footprint_margin_m{0.2};
  double search_radius_m{120.0};
  double shift_step_m{0.1};
  double max_lateral_shift_m{1.5};
  bool propagate_shift{true};
};

struct RoadBorderAvoidanceResult
{
  Trajectory trajectory;
  size_t num_shifted_points{0};
  size_t num_unresolved_points{0};

  [[nodiscard]] bool modified() const { return num_shifted_points + num_unresolved_points > 0; }
};

class RoadBorderAvoidance
{
public:
  RoadBorderAvoidance(
    const RoadBorderAvoidanceParams & params,
    const autoware::vehicle_info_utils::VehicleInfo & vehicle_info);

  void set_map(const lanelet::LaneletMap & lanelet_map);

  void set_road_borders(std::vector<autoware_utils_geometry::LineString2d> road_borders);

  [[nodiscard]] RoadBorderAvoidanceResult adjust(
    const Trajectory & raw_trajectory, const geometry_msgs::msg::Pose & ego_pose) const;

private:
  RoadBorderAvoidanceParams params_;
  autoware_utils_geometry::LinearRing2d base_footprint_;
  std::vector<autoware_utils_geometry::LineString2d> road_borders_;
};

}  // namespace autoware::trajectory_processor::time_sequence_raw

#endif  // AUTOWARE__TRAJECTORY_PROCESSOR__TIME_SEQUENCE_RAW__ROAD_BORDER_AVOIDANCE_HPP_
