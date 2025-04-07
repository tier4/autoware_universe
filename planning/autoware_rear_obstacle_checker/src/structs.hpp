// Copyright 2025 TIER IV, Inc.
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

#ifndef STRUCTS_HPP_
#define STRUCTS_HPP_

#include <autoware_utils/ros/marker_helper.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::rear_obstacle_checker
{

using PointCloud = pcl::PointCloud<pcl::PointXYZ>;

using DetectionArea = std::pair<lanelet::BasicPolygon3d, lanelet::ConstLanelets>;

using DetectionAreas = std::vector<DetectionArea>;

struct PointCloudObject
{
  rclcpp::Time last_update_time;

  geometry_msgs::msg::Point position;

  lanelet::Id base_lane_id;

  double absolute_distance;

  double relative_distance;

  double velocity;
};

using PointCloudObjects = std::vector<PointCloudObject>;

struct DebugData
{
  lanelet::BasicPolygon3d predicted_stop_pose_footprint;

  lanelet::ConstLanelets detection_lanes_for_objects;

  lanelet::BasicPolygons3d detection_areas_for_pointcloud;

  behavior_path_planner::utils::path_safety_checker::CollisionCheckDebugMap collision_check;

  PointCloudObjects pointcloud_objects;

  DetectionAreas detection_areas;

  std::vector<geometry_msgs::msg::Point> obstacle_pointcloud;

  std::pair<std::string, std_msgs::msg::ColorRGBA> text{
    "NONE", autoware_utils::create_marker_color(1.0, 1.0, 1.0, 0.999)};

  auto get_detection_polygons() const -> lanelet::BasicPolygons3d
  {
    lanelet::BasicPolygons3d ret{};
    for (const auto & [polygon, lanes] : detection_areas) {
      ret.push_back(polygon);
    }
    return ret;
  }

  auto get_detection_lanes() const -> lanelet::ConstLanelets
  {
    lanelet::ConstLanelets ret{};
    for (const auto & [polygon, lanes] : detection_areas) {
      ret.insert(ret.end(), lanes.begin(), lanes.end());
    }
    return ret;
  }
};
}  // namespace autoware::rear_obstacle_checker

#endif  // STRUCTS_HPP_
