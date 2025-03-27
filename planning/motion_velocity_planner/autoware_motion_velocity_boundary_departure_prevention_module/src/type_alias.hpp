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

#ifndef TYPE_ALIAS_HPP_
#define TYPE_ALIAS_HPP_

#include <autoware/boundary_departure_checker/parameters.hpp>
#include <autoware/motion_utils/marker/virtual_wall_marker_creator.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils/ros/debug_publisher.hpp>
#include <autoware_utils/ros/marker_helper.hpp>
#include <autoware_utils/ros/polling_subscriber.hpp>
#include <autoware_utils/ros/processing_time_publisher.hpp>
#include <autoware_vehicle_info_utils/vehicle_info.hpp>
#include <diagnostic_updater/diagnostic_status_wrapper.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>

#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <autoware_planning_msgs/msg/trajectory_point.hpp>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <vector>

namespace autoware::motion_velocity_planner
{
using autoware_map_msgs::msg::LaneletMapBin;
using autoware_planning_msgs::msg::Trajectory;
using autoware_planning_msgs::msg::TrajectoryPoint;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::PoseWithCovariance;
using nav_msgs::msg::Odometry;
using TrajectoryPoints = std::vector<TrajectoryPoint>;
using autoware_adapi_v1_msgs::msg::OperationModeState;
using BoundaryThreshold = lane_departure_checker::Side<double>;
using autoware_utils::LinearRing2d;
using autoware_utils::Segment2d;
using lane_departure_checker::EgoSide;
using lane_departure_checker::EgoSides;
using lane_departure_checker::SideToBoundPojections;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

namespace polling_policy = autoware_utils::polling_policy;  // NOLINT

using autoware_utils::create_default_marker;          // NOLINT
using autoware_utils::create_marker_color;            // NOLINT
using autoware_utils::create_marker_scale;            // NOLINT
using autoware_utils::ProcessingTimePublisher;        // NOLINT
using autoware_utils::to_msg;                         // NOLINT
using autoware_utils::Point2d;               // NOLINT
using lane_departure_checker::Output;                 // NOLINT
using lane_departure_checker::Projection;             // NOLINT
using lane_departure_checker::ProjectionWithSegment;  // NOLINT
using vehicle_info_utils::VehicleInfo;                // NOLINT

}  // namespace autoware::motion_velocity_planner

#endif  // TYPE_ALIAS_HPP_
