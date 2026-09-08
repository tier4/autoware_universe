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

#ifndef AUTOWARE__TRAJECTORY_PROCESSOR__TRAJECTORY_PROCESSOR_DATA_HPP_
#define AUTOWARE__TRAJECTORY_PROCESSOR__TRAJECTORY_PROCESSOR_DATA_HPP_

#include "autoware/trajectory_processor/semantic_speed_tracker.hpp"

#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_perception_msgs/msg/tracked_objects.hpp>
#include <autoware_perception_msgs/msg/traffic_light_group_array.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <autoware_vehicle_msgs/msg/steering_report.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>

#include <lanelet2_core/LaneletMap.h>

#include <cstddef>
#include <memory>

namespace autoware::trajectory_processor
{

/// @brief Runtime inputs and mutable state shared while processing one candidate trajectory.
///
/// A fresh instance must be created per candidate so mutable pipeline state is not shared between
/// candidates. Inputs that are not required by every plugin remain optional.
struct TrajectoryProcessorData
{
  nav_msgs::msg::Odometry::ConstSharedPtr current_odometry{nullptr};
  geometry_msgs::msg::AccelWithCovarianceStamped::ConstSharedPtr current_acceleration{nullptr};
  autoware_vehicle_msgs::msg::SteeringReport::ConstSharedPtr current_steering{nullptr};
  autoware_perception_msgs::msg::PredictedObjects::ConstSharedPtr predicted_objects{nullptr};
  autoware_perception_msgs::msg::TrackedObjects::ConstSharedPtr tracked_objects{nullptr};
  sensor_msgs::msg::PointCloud2::ConstSharedPtr obstacle_pointcloud{nullptr};
  std::shared_ptr<lanelet::LaneletMap> lanelet_map{nullptr};
  autoware_map_msgs::msg::LaneletMapBin::ConstSharedPtr lanelet_map_bin{nullptr};
  autoware_planning_msgs::msg::LaneletRoute::ConstSharedPtr route{nullptr};
  autoware_perception_msgs::msg::TrafficLightGroupArray::ConstSharedPtr traffic_light_signals{
    nullptr};
  std_msgs::msg::Header candidate_header{};
  std::size_t candidate_index{0U};
  std::size_t candidate_count{0U};
  SemanticSpeedTracker semantic_speed_tracker;
};

}  // namespace autoware::trajectory_processor

#endif  // AUTOWARE__TRAJECTORY_PROCESSOR__TRAJECTORY_PROCESSOR_DATA_HPP_
