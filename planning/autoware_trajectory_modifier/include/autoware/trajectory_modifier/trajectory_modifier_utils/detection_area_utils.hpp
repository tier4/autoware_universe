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

#include <autoware/object_recognition_utils/object_classification.hpp>
#include <autoware_lanelet2_extension/regulatory_elements/detection_area.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>
#include <autoware_utils/geometry/boost_polygon_utils.hpp>
#include <autoware_utils/geometry/geometry.hpp>

#include <autoware_perception_msgs/msg/predicted_object.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_planning_msgs/msg/trajectory_point.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <boost/geometry/algorithms/intersects.hpp>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/Polygon.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/trajectory/trajectory_point.hpp>
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

template <typename Filtering>
bool is_target_object(
  const std::vector<autoware_perception_msgs::msg::ObjectClassification> & classifications,
  const Filtering & target_filtering)
{
  using ObjectClassification = autoware_perception_msgs::msg::ObjectClassification;
  if (classifications.empty()) return false;
  const auto label = autoware::object_recognition_utils::getHighestProbLabel(classifications);
  switch (label) {
    case ObjectClassification::UNKNOWN:
      return target_filtering.unknown;
    case ObjectClassification::CAR:
      return target_filtering.car;
    case ObjectClassification::TRUCK:
      return target_filtering.truck;
    case ObjectClassification::BUS:
      return target_filtering.bus;
    case ObjectClassification::TRAILER:
      return target_filtering.trailer;
    case ObjectClassification::MOTORCYCLE:
      return target_filtering.motorcycle;
    case ObjectClassification::BICYCLE:
      return target_filtering.bicycle;
    case ObjectClassification::PEDESTRIAN:
      return target_filtering.pedestrian;
    case ObjectClassification::ANIMAL:
      return target_filtering.animal;
    case ObjectClassification::HAZARD:
      return target_filtering.hazard;
    case ObjectClassification::OVER_DRIVABLE:
      return target_filtering.over_drivable;
    case ObjectClassification::UNDER_DRIVABLE:
      return target_filtering.under_drivable;
    default:
      return false;
  }
}

template <typename Filtering>
std::optional<autoware_perception_msgs::msg::PredictedObject> get_detected_object(
  const lanelet::ConstPolygons3d & detection_areas,
  const autoware_perception_msgs::msg::PredictedObjects & predicted_objects,
  const Filtering & target_filtering)
{
  for (const auto & object : predicted_objects.objects) {
    if (!is_target_object(object.classification, target_filtering)) continue;
    const auto & pose = object.kinematics.initial_pose_with_covariance.pose;
    const auto object_polygon = autoware_utils::to_polygon2d(pose, object.shape);
    for (const auto & detection_area : detection_areas) {
      const auto detection_polygon = lanelet::utils::to2D(detection_area).basicPolygon();
      if (boost::geometry::intersects(object_polygon, detection_polygon)) return object;
    }
  }
  return std::nullopt;
}

std::string object_label_to_string(uint8_t label);

bool can_clear_stop_state(
  const std::optional<rclcpp::Time> & last_obstacle_found_time, const rclcpp::Time & now,
  double state_clear_time);

double feasible_stop_distance_by_max_acceleration(double current_velocity, double max_acceleration);
}  // namespace autoware::trajectory_modifier::utils::detection_area

#endif  // AUTOWARE__TRAJECTORY_MODIFIER__TRAJECTORY_MODIFIER_UTILS__DETECTION_AREA_UTILS_HPP_
