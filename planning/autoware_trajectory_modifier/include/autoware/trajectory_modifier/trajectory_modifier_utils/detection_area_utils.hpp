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

#ifndef AUTOWARE__TRAJECTORY_MODIFIER__TRAJECTORY_MODIFIER_UTILS__DETECTION_AREA_UTILS_HPP_
#define AUTOWARE__TRAJECTORY_MODIFIER__TRAJECTORY_MODIFIER_UTILS__DETECTION_AREA_UTILS_HPP_

#include <autoware_lanelet2_extension/regulatory_elements/detection_area.hpp>

#include <autoware_perception_msgs/msg/predicted_object.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_planning_msgs/msg/trajectory_point.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <lanelet2_core/LaneletMap.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <rclcpp/rclcpp.hpp>

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace autoware::trajectory_modifier::utils::detection_area
{
using Trajectory =
  autoware::experimental::trajectory::Trajectory<autoware_planning_msgs::msg::TrajectoryPoint>;
using PointCloud = pcl::PointCloud<pcl::PointXYZ>;

struct TargetFiltering
{
  bool pointcloud{true};
  bool unknown{false};
  bool car{false};
  bool truck{false};
  bool bus{false};
  bool trailer{false};
  bool motorcycle{false};
  bool bicycle{false};
  bool pedestrian{false};
  bool animal{false};
  bool hazard{false};
  bool over_drivable{false};
  bool under_drivable{false};
};

std::optional<double> get_stop_point(
  const Trajectory & path, const lanelet::ConstLineString3d & stop_line, double margin,
  double vehicle_offset);

std::vector<geometry_msgs::msg::Point> get_obstacle_points(
  const lanelet::ConstPolygons3d & detection_areas, const PointCloud & points);

std::optional<autoware_perception_msgs::msg::PredictedObject> get_detected_object(
  const lanelet::ConstPolygons3d & detection_areas,
  const autoware_perception_msgs::msg::PredictedObjects & predicted_objects,
  const TargetFiltering & target_filtering);

bool is_target_object(
  const std::vector<autoware_perception_msgs::msg::ObjectClassification> & classifications,
  const TargetFiltering & target_filtering);

std::string object_label_to_string(uint8_t label);

bool can_clear_stop_state(
  const std::optional<rclcpp::Time> & last_obstacle_found_time, const rclcpp::Time & now,
  double state_clear_time);

double feasible_stop_distance_by_max_acceleration(double current_velocity, double max_acceleration);
}  // namespace autoware::trajectory_modifier::utils::detection_area

#endif  // AUTOWARE__TRAJECTORY_MODIFIER__TRAJECTORY_MODIFIER_UTILS__DETECTION_AREA_UTILS_HPP_
