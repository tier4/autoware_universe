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

#ifndef AUTOWARE__ML_PLANNER__UTILS__MARKER_UTILS_HPP_
#define AUTOWARE__ML_PLANNER__UTILS__MARKER_UTILS_HPP_

#include <Eigen/Dense>
#include <autoware_vehicle_info_utils/vehicle_info.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <xtensor/xarray.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/detail/color_rgba__struct.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace autoware::ml_planner::utils
{
using rclcpp::Duration;
using rclcpp::Time;
using std_msgs::msg::ColorRGBA;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

/**
 * @brief Creates a visualization marker array representing a lane.
 *
 * This function generates a MarkerArray for visualizing a lane using the provided lane vector and
 * shape. The markers can be customized with color, frame, and lifetime, and optionally colored for
 * traffic lights.
 *
 * @param lane_vector A vector of floats representing the lane geometry or points.
 * @param shape A vector of longs specifying the shape or dimensions of the lane data.
 * @param stamp The timestamp to assign to the marker messages.
 * @param lifetime The duration for which the markers should remain visible.
 * @param colors An array of 4 floats specifying the RGBA color of the lane markers (default: green
 * with alpha 0.8).
 * @param frame_id The coordinate frame in which to publish the markers (default: "base_link").
 * @return MarkerArray containing the generated lane markers.
 */
MarkerArray create_lane_marker(
  const Eigen::Matrix4d & transform_ego_to_map, const xt::xarray<float> & lane_vector,
  const std::vector<int64_t> & shape, const Time & stamp, const rclcpp::Duration & lifetime,
  const std::array<float, 4> colors = {0.0f, 1.0f, 0.0f, 0.8f},
  const std::string & frame_id = "base_link");

/**
 * @brief Creates a visualization marker array representing map polylines.
 *
 * @param transform_ego_to_map The transformation matrix to convert points from ego frame to map
 * frame.
 * @param polyline_vector A vector of floats representing the polyline geometry or points.
 * @param shape A vector of longs specifying the shape or dimensions of the polyline data.
 * @param stamp The timestamp to assign to the marker messages.
 * @param lifetime The duration for which the markers should remain visible.
 * @param colors An array of 4 floats specifying the RGBA marker color.
 * @param marker_namespace The namespace assigned to the markers.
 * @param frame_id The coordinate frame in which to publish the markers (default: "base_link").
 * @return MarkerArray containing the generated polyline markers.
 */
MarkerArray create_map_polyline_marker(
  const Eigen::Matrix4d & transform_ego_to_map, const xt::xarray<float> & polyline_vector,
  const std::vector<int64_t> & shape, const Time & stamp, const rclcpp::Duration & lifetime,
  const std::array<float, 4> & colors, const std::string & marker_namespace,
  const std::string & frame_id = "base_link");
/**
 * @brief Creates a marker of the vehicle body at a pose.
 *
 * @param pose Pose of base_link in the map frame.
 * @param vehicle_info Vehicle dimensions, for the box drawn without a mesh.
 * @param mesh_resource Body mesh modelled around base_link (package:// or file:// URI); empty
 * draws a box of the vehicle size.
 * @param stamp The timestamp to assign to the marker.
 * @param lifetime The duration for which the marker should remain visible.
 * @param colors An array of 4 floats specifying the RGBA marker color.
 * @param marker_namespace The namespace assigned to the marker.
 * @return Marker of the vehicle body in the map frame.
 */
Marker create_vehicle_marker(
  const geometry_msgs::msg::Pose & pose,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info, const std::string & mesh_resource,
  const Time & stamp, const rclcpp::Duration & lifetime, const std::array<float, 4> & colors,
  const std::string & marker_namespace);

/**
 * @brief Finds the visual mesh of a link in a robot description.
 *
 * @param robot_description The URDF of the vehicle.
 * @param link_name The link whose visual mesh is looked up.
 * @return The mesh URI (package:// or file://); empty when the description cannot be parsed or the
 * link has no visual mesh.
 */
std::string find_link_mesh(const std::string & robot_description, const std::string & link_name);

}  // namespace autoware::ml_planner::utils
#endif  // AUTOWARE__ML_PLANNER__UTILS__MARKER_UTILS_HPP_
