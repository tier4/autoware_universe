// Copyright 2022 Tier IV, Inc.
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

#include "autoware/predicted_trajectory_validator/debug_marker.hpp"

#include <autoware/motion_utils/marker/marker_helper.hpp>
#include <autoware_utils/ros/marker_helper.hpp>

#include <string>

using visualization_msgs::msg::Marker;

PredictedTrajectoryValidatorDebugMarkerPublisher::PredictedTrajectoryValidatorDebugMarkerPublisher(
  rclcpp::Node * node)
: node_(node)
{
  virtual_wall_pub_ =
    node_->create_publisher<visualization_msgs::msg::MarkerArray>("~/virtual_wall", 1);
}

void PredictedTrajectoryValidatorDebugMarkerPublisher::clear_markers()
{
  marker_array_virtual_wall_.markers.clear();
}

void PredictedTrajectoryValidatorDebugMarkerPublisher::push_virtual_wall(
  const geometry_msgs::msg::Pose & pose, const std::string & msg)
{
  const auto now = node_->get_clock()->now();
  std::string display_msg = "predicted_trajectory_validator \n(" + msg + ")";
  const auto stop_wall_marker =
    autoware::motion_utils::createStopVirtualWallMarker(pose, display_msg, now, 0);
  autoware_utils::append_marker_array(stop_wall_marker, &marker_array_virtual_wall_, now);
}

void PredictedTrajectoryValidatorDebugMarkerPublisher::publish()
{
  virtual_wall_pub_->publish(marker_array_virtual_wall_);
}
