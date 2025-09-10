// Copyright 2023 Autoware Foundation. All rights reserved.
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
//
// Authors: Mamoru Sobue

// NOLINTBEGIN(readability-identifier-naming)

#include <autoware_lanelet2_extension/utility/query.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>

#include <boost/python.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>

#include <lanelet2_core/geometry/Polygon.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_python/internal/converter.h>

#include <iostream>
#include <string>

namespace bp = boost::python;

namespace
{

/*
 * utilities.cpp
 */
lanelet::Optional<lanelet::ConstPolygon3d> lineStringWithWidthToPolygon(
  const lanelet::ConstLineString3d & linestring)
{
  lanelet::ConstPolygon3d poly{};
  if (lanelet::utils::lineStringWithWidthToPolygon(linestring, &poly)) {
    return poly;
  }
  return {};
}

lanelet::Optional<lanelet::ConstPolygon3d> lineStringToPolygon(
  const lanelet::ConstLineString3d & linestring)
{
  lanelet::ConstPolygon3d poly{};
  if (lanelet::utils::lineStringToPolygon(linestring, &poly)) {
    return poly;
  }
  return {};
}

lanelet::ArcCoordinates getArcCoordinates(
  const lanelet::ConstLanelets & lanelet_sequence, const std::string & pose_byte)
{
  rclcpp::SerializedMessage serialized_msg;
  static constexpr size_t message_header_length = 8u;
  serialized_msg.reserve(message_header_length + pose_byte.size());
  serialized_msg.get_rcl_serialized_message().buffer_length = pose_byte.size();
  for (size_t i = 0; i < pose_byte.size(); ++i) {
    // NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    serialized_msg.get_rcl_serialized_message().buffer[i] = pose_byte[i];
    // NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic)
  }
  geometry_msgs::msg::Pose pose;
  static rclcpp::Serialization<geometry_msgs::msg::Pose> serializer;
  serializer.deserialize_message(&serialized_msg, &pose);
  return lanelet::utils::getArcCoordinates(lanelet_sequence, pose);
}

double getLaneletAngle(const lanelet::ConstLanelet & lanelet, const std::string & point_byte)
{
  rclcpp::SerializedMessage serialized_msg;
  static constexpr size_t message_header_length = 8u;
  serialized_msg.reserve(message_header_length + point_byte.size());
  serialized_msg.get_rcl_serialized_message().buffer_length = point_byte.size();
  for (size_t i = 0; i < point_byte.size(); ++i) {
    // NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    serialized_msg.get_rcl_serialized_message().buffer[i] = point_byte[i];
    // NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic)
  }
  geometry_msgs::msg::Point point;
  static rclcpp::Serialization<geometry_msgs::msg::Point> serializer;
  serializer.deserialize_message(&serialized_msg, &point);
  return lanelet::utils::getLaneletAngle(lanelet, point);
}

bool isInLanelet(
  const std::string & pose_byte, const lanelet::ConstLanelet & lanelet, const double radius = 0.0)
{
  rclcpp::SerializedMessage serialized_msg;
  static constexpr size_t message_header_length = 8u;
  serialized_msg.reserve(message_header_length + pose_byte.size());
  serialized_msg.get_rcl_serialized_message().buffer_length = pose_byte.size();
  for (size_t i = 0; i < pose_byte.size(); ++i) {
    // NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    serialized_msg.get_rcl_serialized_message().buffer[i] = pose_byte[i];
    // NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic)
  }
  geometry_msgs::msg::Pose pose;
  static rclcpp::Serialization<geometry_msgs::msg::Pose> serializer;
  serializer.deserialize_message(&serialized_msg, &pose);
  return lanelet::utils::isInLanelet(pose, lanelet, radius);
}

std::vector<double> getClosestCenterPose(
  const lanelet::ConstLanelet & lanelet, const std::string & search_point_byte)
{
  rclcpp::SerializedMessage serialized_point_msg;
  static constexpr size_t message_header_length = 8u;
  serialized_point_msg.reserve(message_header_length + search_point_byte.size());
  serialized_point_msg.get_rcl_serialized_message().buffer_length = search_point_byte.size();
  for (size_t i = 0; i < search_point_byte.size(); ++i) {
    // NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    serialized_point_msg.get_rcl_serialized_message().buffer[i] = search_point_byte[i];
    // NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic)
  }
  geometry_msgs::msg::Point search_point;
  static rclcpp::Serialization<geometry_msgs::msg::Point> serializer_point;
  serializer_point.deserialize_message(&serialized_point_msg, &search_point);
  const geometry_msgs::msg::Pose pose = lanelet::utils::getClosestCenterPose(lanelet, search_point);
  // NOTE: it was difficult to return the deserialized pose_byte and serialize the pose_byte on
  // python-side. So this function returns [*position, *quaternion] as double array
  const auto & xyz = pose.position;
  const auto & quat = pose.orientation;
  return std::vector<double>({xyz.x, xyz.y, xyz.z, quat.x, quat.y, quat.z, quat.w});
}

double getLateralDistanceToCenterline(
  const lanelet::ConstLanelet & lanelet, const std::string & pose_byte)
{
  rclcpp::SerializedMessage serialized_msg;
  static constexpr size_t message_header_length = 8u;
  serialized_msg.reserve(message_header_length + pose_byte.size());
  serialized_msg.get_rcl_serialized_message().buffer_length = pose_byte.size();
  for (size_t i = 0; i < pose_byte.size(); ++i) {
    // NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    serialized_msg.get_rcl_serialized_message().buffer[i] = pose_byte[i];
    // NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic)
  }
  geometry_msgs::msg::Pose pose;
  static rclcpp::Serialization<geometry_msgs::msg::Pose> serializer;
  serializer.deserialize_message(&serialized_msg, &pose);
  return lanelet::utils::getLateralDistanceToCenterline(lanelet, pose);
}

double getLateralDistanceToClosestLanelet(
  const lanelet::ConstLanelets & lanelet_sequence, const std::string & pose_byte)
{
  rclcpp::SerializedMessage serialized_msg;
  static constexpr size_t message_header_length = 8u;
  serialized_msg.reserve(message_header_length + pose_byte.size());
  serialized_msg.get_rcl_serialized_message().buffer_length = pose_byte.size();
  for (size_t i = 0; i < pose_byte.size(); ++i) {
    // NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    serialized_msg.get_rcl_serialized_message().buffer[i] = pose_byte[i];
    // NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic)
  }
  geometry_msgs::msg::Pose pose;
  static rclcpp::Serialization<geometry_msgs::msg::Pose> serializer;
  serializer.deserialize_message(&serialized_msg, &pose);
  return lanelet::utils::getLateralDistanceToClosestLanelet(lanelet_sequence, pose);
}

/*
 * query.cpp
 */

lanelet::ConstLanelets subtypeLanelets(
  const lanelet::ConstLanelets & lls, const std::string & subtype)
{
  return lanelet::utils::query::subtypeLanelets(lls, subtype.c_str());
}

lanelet::Optional<lanelet::ConstLanelet> getLinkedLanelet(
  const lanelet::ConstLineString3d & parking_space,
  const lanelet::ConstLanelets & all_road_lanelets,
  const lanelet::ConstPolygons3d & all_parking_lots)
{
  lanelet::ConstLanelet linked_lanelet;
  if (lanelet::utils::query::getLinkedLanelet(
        parking_space, all_road_lanelets, all_parking_lots, &linked_lanelet)) {
    return linked_lanelet;
  }
  return {};
}

lanelet::Optional<lanelet::ConstLanelet> getLinkedLanelet(
  const lanelet::ConstLineString3d & parking_space,
  const lanelet::LaneletMapConstPtr & lanelet_map_ptr)
{
  lanelet::ConstLanelet linked_lanelet;
  if (lanelet::utils::query::getLinkedLanelet(parking_space, lanelet_map_ptr, &linked_lanelet)) {
    return linked_lanelet;
  }
  return {};
}

lanelet::Optional<lanelet::ConstPolygon3d> getLinkedParkingLot(
  const lanelet::ConstLanelet & lanelet, const lanelet::ConstPolygons3d & all_parking_lots)
{
  lanelet::ConstPolygon3d linked_parking_lot;
  if (lanelet::utils::query::getLinkedParkingLot(lanelet, all_parking_lots, &linked_parking_lot)) {
    return linked_parking_lot;
  }
  return {};
}

lanelet::Optional<lanelet::ConstPolygon3d> getLinkedParkingLot(
  const lanelet::BasicPoint2d & current_position, const lanelet::ConstPolygons3d & all_parking_lots)
{
  lanelet::ConstPolygon3d linked_parking_lot;
  if (lanelet::utils::query::getLinkedParkingLot(
        current_position, all_parking_lots, &linked_parking_lot)) {
    return linked_parking_lot;
  }
  return {};
}

lanelet::Optional<lanelet::ConstPolygon3d> getLinkedParkingLot(
  const lanelet::ConstLineString3d & parking_space,
  const lanelet::ConstPolygons3d & all_parking_lots)
{
  lanelet::ConstPolygon3d linked_parking_lot;
  if (lanelet::utils::query::getLinkedParkingLot(
        parking_space, all_parking_lots, &linked_parking_lot)) {
    return linked_parking_lot;
  }
  return {};
}

lanelet::ConstLanelets getLaneletsWithinRange_point(
  const lanelet::ConstLanelets & lanelets, const std::string & point_byte, const double range)
{
  rclcpp::SerializedMessage serialized_msg;
  static constexpr size_t message_header_length = 8u;
  serialized_msg.reserve(message_header_length + point_byte.size());
  serialized_msg.get_rcl_serialized_message().buffer_length = point_byte.size();
  for (size_t i = 0; i < point_byte.size(); ++i) {
    // NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    serialized_msg.get_rcl_serialized_message().buffer[i] = point_byte[i];
    // NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic)
  }
  geometry_msgs::msg::Point point;
  static rclcpp::Serialization<geometry_msgs::msg::Point> serializer;
  serializer.deserialize_message(&serialized_msg, &point);
  return lanelet::utils::query::getLaneletsWithinRange(lanelets, point, range);
}

lanelet::ConstLanelets getLaneChangeableNeighbors_point(
  const lanelet::routing::RoutingGraphPtr & graph, const lanelet::ConstLanelets & road_lanelets,
  const std::string & point_byte)
{
  rclcpp::SerializedMessage serialized_msg;
  static constexpr size_t message_header_length = 8u;
  serialized_msg.reserve(message_header_length + point_byte.size());
  serialized_msg.get_rcl_serialized_message().buffer_length = point_byte.size();
  for (size_t i = 0; i < point_byte.size(); ++i) {
    // NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    serialized_msg.get_rcl_serialized_message().buffer[i] = point_byte[i];
    // NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic)
  }
  geometry_msgs::msg::Point point;
  static rclcpp::Serialization<geometry_msgs::msg::Point> serializer;
  serializer.deserialize_message(&serialized_msg, &point);
  return lanelet::utils::query::getLaneChangeableNeighbors(graph, road_lanelets, point);
}

lanelet::ConstLanelets getAllNeighbors_point(
  const lanelet::routing::RoutingGraphPtr & graph, const lanelet::ConstLanelets & road_lanelets,
  const std::string & point_byte)
{
  rclcpp::SerializedMessage serialized_msg;
  static constexpr size_t message_header_length = 8u;
  serialized_msg.reserve(message_header_length + point_byte.size());
  serialized_msg.get_rcl_serialized_message().buffer_length = point_byte.size();
  for (size_t i = 0; i < point_byte.size(); ++i) {
    // NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    serialized_msg.get_rcl_serialized_message().buffer[i] = point_byte[i];
    // NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic)
  }
  geometry_msgs::msg::Point point;
  static rclcpp::Serialization<geometry_msgs::msg::Point> serializer;
  serializer.deserialize_message(&serialized_msg, &point);
  return lanelet::utils::query::getAllNeighbors(graph, road_lanelets, point);
}

lanelet::Optional<lanelet::ConstLanelet> getClosestLanelet(
  const lanelet::ConstLanelets & lanelets, const std::string & pose_byte)
{
  rclcpp::SerializedMessage serialized_msg;
  static constexpr size_t message_header_length = 8u;
  serialized_msg.reserve(message_header_length + pose_byte.size());
  serialized_msg.get_rcl_serialized_message().buffer_length = pose_byte.size();
  for (size_t i = 0; i < pose_byte.size(); ++i) {
    // NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    serialized_msg.get_rcl_serialized_message().buffer[i] = pose_byte[i];
    // NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic)
  }
  geometry_msgs::msg::Pose pose;
  static rclcpp::Serialization<geometry_msgs::msg::Pose> serializer;
  serializer.deserialize_message(&serialized_msg, &pose);
  lanelet::ConstLanelet closest_lanelet{};
  if (lanelet::utils::query::getClosestLanelet(lanelets, pose, &closest_lanelet)) {
    return closest_lanelet;
  }
  return {};
}

lanelet::Optional<lanelet::ConstLanelet> getClosestLaneletWithConstrains(
  const lanelet::ConstLanelets & lanelets, const std::string & pose_byte,
  const double dist_threshold = std::numeric_limits<double>::max(),
  const double yaw_threshold = std::numeric_limits<double>::max())
{
  rclcpp::SerializedMessage serialized_msg;
  static constexpr size_t message_header_length = 8u;
  serialized_msg.reserve(message_header_length + pose_byte.size());
  serialized_msg.get_rcl_serialized_message().buffer_length = pose_byte.size();
  for (size_t i = 0; i < pose_byte.size(); ++i) {
    // NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    serialized_msg.get_rcl_serialized_message().buffer[i] = pose_byte[i];
    // NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic)
  }
  geometry_msgs::msg::Pose pose;
  static rclcpp::Serialization<geometry_msgs::msg::Pose> serializer;
  serializer.deserialize_message(&serialized_msg, &pose);
  lanelet::ConstLanelet closest_lanelet{};
  if (lanelet::utils::query::getClosestLaneletWithConstrains(
        lanelets, pose, &closest_lanelet, dist_threshold, yaw_threshold)) {
    return closest_lanelet;
  }
  return {};
}

lanelet::ConstLanelets getCurrentLanelets_point(
  const lanelet::ConstLanelets & lanelets, const std::string & point_byte)
{
  rclcpp::SerializedMessage serialized_msg;
  static constexpr size_t message_header_length = 8u;
  serialized_msg.reserve(message_header_length + point_byte.size());
  serialized_msg.get_rcl_serialized_message().buffer_length = point_byte.size();
  for (size_t i = 0; i < point_byte.size(); ++i) {
    // NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    serialized_msg.get_rcl_serialized_message().buffer[i] = point_byte[i];
    // NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic)
  }
  geometry_msgs::msg::Point point;
  static rclcpp::Serialization<geometry_msgs::msg::Point> serializer;
  serializer.deserialize_message(&serialized_msg, &point);
  lanelet::ConstLanelets current_lanelets{};
  lanelet::utils::query::getCurrentLanelets(lanelets, point, &current_lanelets);
  return current_lanelets;
}

lanelet::ConstLanelets getCurrentLanelets_pose(
  const lanelet::ConstLanelets & lanelets, const std::string & pose_byte)
{
  rclcpp::SerializedMessage serialized_msg;
  static constexpr size_t message_header_length = 8u;
  serialized_msg.reserve(message_header_length + pose_byte.size());
  serialized_msg.get_rcl_serialized_message().buffer_length = pose_byte.size();
  for (size_t i = 0; i < pose_byte.size(); ++i) {
    // NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    serialized_msg.get_rcl_serialized_message().buffer[i] = pose_byte[i];
    // NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic)
  }
  geometry_msgs::msg::Pose pose;
  static rclcpp::Serialization<geometry_msgs::msg::Pose> serializer;
  serializer.deserialize_message(&serialized_msg, &pose);
  lanelet::ConstLanelets current_lanelets{};
  lanelet::utils::query::getCurrentLanelets(lanelets, pose, &current_lanelets);
  return current_lanelets;
}

}  // namespace

// for handling functions with default arguments
/// utilities.cpp
// NOLINTBEGIN(google-explicit-constructor)
BOOST_PYTHON_FUNCTION_OVERLOADS(
  generateFineCenterline_overload, lanelet::utils::generateFineCenterline, 1, 2)
BOOST_PYTHON_FUNCTION_OVERLOADS(
  getCenterlineWithOffset_overload, lanelet::utils::getCenterlineWithOffset, 2, 3)
BOOST_PYTHON_FUNCTION_OVERLOADS(
  getRightBoundWithOffset_overload, lanelet::utils::getRightBoundWithOffset, 2, 3)
BOOST_PYTHON_FUNCTION_OVERLOADS(
  getLeftBoundWithOffset_overload, lanelet::utils::getLeftBoundWithOffset, 2, 3)
BOOST_PYTHON_FUNCTION_OVERLOADS(
  overwriteLaneletsCenterline_overload, lanelet::utils::overwriteLaneletsCenterline, 1, 3)
BOOST_PYTHON_FUNCTION_OVERLOADS(isInLanelet_overload, ::isInLanelet, 2, 3)

/// query.cpp
BOOST_PYTHON_FUNCTION_OVERLOADS(
  stopSignStopLines_overload, lanelet::utils::query::stopSignStopLines, 1, 2)
BOOST_PYTHON_FUNCTION_OVERLOADS(
  getClosestLaneletWithConstrains_overload, ::getClosestLaneletWithConstrains, 2, 4)
BOOST_PYTHON_FUNCTION_OVERLOADS(
  getPrecedingLaneletSequences_overload, lanelet::utils::query::getPrecedingLaneletSequences, 3, 4)
// NOLINTEND(google-explicit-constructor)

BOOST_PYTHON_MODULE(_autoware_lanelet2_extension_python_boost_python_utility)
{
  /*
   * utilities.cpp
   */
  bp::def("combineLaneletsShape", lanelet::utils::combineLaneletsShape);
  bp::def(
    "generateFineCenterline", lanelet::utils::generateFineCenterline,
    generateFineCenterline_overload());
  bp::def(
    "getCenterlineWithOffset", lanelet::utils::getCenterlineWithOffset,
    getCenterlineWithOffset_overload());
  bp::def(
    "getRightBoundWithOffset", lanelet::utils::getRightBoundWithOffset,
    getRightBoundWithOffset_overload());
  bp::def(
    "getLeftBoundWithOffset", lanelet::utils::getLeftBoundWithOffset,
    getLeftBoundWithOffset_overload());
  bp::def("getExpandedLanelet", lanelet::utils::getExpandedLanelet);
  bp::def("getExpandedLanelets", lanelet::utils::getExpandedLanelets);
  bp::def(
    "overwriteLaneletsCenterline", lanelet::utils::overwriteLaneletsCenterline,
    overwriteLaneletsCenterline_overload());
  bp::def("getConflictingLanelets", lanelet::utils::getConflictingLanelets);
  bp::def("lineStringWithWidthToPolygon", ::lineStringWithWidthToPolygon);
  bp::def("lineStringToPolygon", ::lineStringToPolygon);
  bp::def<double(const lanelet::ConstLanelet &)>(
    "getLaneletLength2d", lanelet::utils::getLaneletLength2d);
  bp::def<double(const lanelet::ConstLanelet &)>(
    "getLaneletLength3d", lanelet::utils::getLaneletLength3d);
  bp::def<double(const lanelet::ConstLanelets &)>(
    "getLaneletLength2d", lanelet::utils::getLaneletLength2d);
  bp::def<double(const lanelet::ConstLanelets &)>(
    "getLaneletLength3d", lanelet::utils::getLaneletLength3d);
  bp::def("getArcCoordinates", ::getArcCoordinates);  // depends ros msg
  bp::def("getClosestSegment", lanelet::utils::getClosestSegment);
  bp::def("getPolygonFromArcLength", lanelet::utils::getPolygonFromArcLength);
  bp::def("getLaneletAngle", ::getLaneletAngle);                  // depends on ros msg
  bp::def("isInLanelet", ::isInLanelet, isInLanelet_overload());  // depends ros msg
  bp::def("getClosestCenterPose", ::getClosestCenterPose);        // depends ros msg
  // NOTE: required for the return-value of getClosestCenterPose
  bp::class_<std::vector<double>>("[position, quaternion]")
    .def(bp::vector_indexing_suite<std::vector<double>>());
  bp::def("getLateralDistanceToCenterline", ::getLateralDistanceToCenterline);  // depends ros msg
  bp::def(
    "getLateralDistanceToClosestLanelet", ::getLateralDistanceToClosestLanelet);  // depends ros msg

  /*
   * query.cpp
   */
  bp::def("laneletLayer", lanelet::utils::query::laneletLayer);
  bp::def("subtypeLanelets", ::subtypeLanelets);
  bp::def("crosswalkLanelets", lanelet::utils::query::crosswalkLanelets);
  bp::def("walkwayLanelets", lanelet::utils::query::walkwayLanelets);
  bp::def("roadLanelets", lanelet::utils::query::roadLanelets);
  bp::def("shoulderLanelets", lanelet::utils::query::shoulderLanelets);
  bp::def("trafficLights", lanelet::utils::query::trafficLights);

  bp::def("autowareTrafficLights", lanelet::utils::query::autowareTrafficLights);
  converters::VectorToListConverter<std::vector<lanelet::AutowareTrafficLightConstPtr>>();

  bp::def("detectionAreas", lanelet::utils::query::detectionAreas);
  converters::VectorToListConverter<std::vector<lanelet::DetectionAreaConstPtr>>();

  bp::def("noStoppingAreas", lanelet::utils::query::noStoppingAreas);
  converters::VectorToListConverter<std::vector<lanelet::NoStoppingAreaConstPtr>>();

  bp::def("noParkingAreas", lanelet::utils::query::noParkingAreas);
  converters::VectorToListConverter<std::vector<lanelet::NoParkingAreaConstPtr>>();

  bp::def("speedBumps", lanelet::utils::query::speedBumps);
  converters::VectorToListConverter<std::vector<lanelet::SpeedBumpConstPtr>>();

  bp::def("crosswalks", lanelet::utils::query::crosswalks);
  converters::VectorToListConverter<std::vector<lanelet::CrosswalkConstPtr>>();

  bp::def("curbstones", lanelet::utils::query::curbstones);
  bp::def("getAllPolygonsByType", lanelet::utils::query::getAllPolygonsByType);
  bp::def("getAllObstaclePolygons", lanelet::utils::query::getAllObstaclePolygons);
  bp::def("getAllParkingLots", lanelet::utils::query::getAllParkingLots);
  bp::def("getAllPartitions", lanelet::utils::query::getAllPartitions);
  bp::def("getAllFences", lanelet::utils::query::getAllFences);
  bp::def(
    "getAllPedestrianPolygonMarkings", lanelet::utils::query::getAllPedestrianPolygonMarkings);
  bp::def("getAllPedestrianLineMarkings", lanelet::utils::query::getAllPedestrianLineMarkings);
  bp::def("getAllParkingSpaces", lanelet::utils::query::getAllParkingSpaces);

  bp::def<lanelet::ConstLineStrings3d(
    const lanelet::ConstLanelet &, const lanelet::LaneletMapConstPtr &)>(
    "getLinkedParkingSpaces", lanelet::utils::query::getLinkedParkingSpaces);
  bp::def<lanelet::ConstLineStrings3d(
    const lanelet::ConstLanelet &, const lanelet::ConstLineStrings3d &,
    const lanelet::ConstPolygons3d &)>(
    "getLinkedParkingSpaces", lanelet::utils::query::getLinkedParkingSpaces);
  // NOTE: required for iterating the return-value of getLinkedParkingSpaces/getAllParkingLots, but
  // this causes RuntimeWarning for duplicate to-Python converter
  bp::class_<lanelet::ConstLineStrings3d>("lanelet::ConstLineStrings3d")
    .def(bp::vector_indexing_suite<lanelet::ConstLineStrings3d>());
  bp::class_<lanelet::ConstPolygons3d>("lanelet::ConstPolygons3d")
    .def(bp::vector_indexing_suite<lanelet::ConstPolygons3d>());

  bp::def<lanelet::Optional<lanelet::ConstLanelet>(
    const lanelet::ConstLineString3d &, const lanelet::ConstLanelets &,
    const lanelet::ConstPolygons3d &)>("getLinkedLanelet", ::getLinkedLanelet);
  bp::def<lanelet::Optional<lanelet::ConstLanelet>(
    const lanelet::ConstLineString3d &, const lanelet::LaneletMapConstPtr &)>(
    "getLinkedLanelet", ::getLinkedLanelet);
  bp::def<lanelet::ConstLanelets(
    const lanelet::ConstLineString3d &, const lanelet::ConstLanelets &,
    const lanelet::ConstPolygons3d &)>(
    "getLinkedLanelets", lanelet::utils::query::getLinkedLanelets);
  bp::def<lanelet::ConstLanelets(
    const lanelet::ConstLineString3d &, const lanelet::LaneletMapConstPtr &)>(
    "getLinkedLanelets", lanelet::utils::query::getLinkedLanelets);
  bp::def<lanelet::Optional<lanelet::ConstPolygon3d>(
    const lanelet::ConstLanelet &, const lanelet::ConstPolygons3d &)>(
    "getLinkedParkingLot", ::getLinkedParkingLot);
  bp::def<lanelet::Optional<lanelet::ConstPolygon3d>(
    const lanelet::BasicPoint2d &, const lanelet::ConstPolygons3d &)>(
    "getLinkedParkingLot", ::getLinkedParkingLot);
  bp::def<lanelet::Optional<lanelet::ConstPolygon3d>(
    const lanelet::ConstLineString3d &, const lanelet::ConstPolygons3d &)>(
    "getLinkedParkingLot", ::getLinkedParkingLot);
  bp::def<lanelet::ConstLineStrings3d(
    const lanelet::ConstPolygon3d & parking_lot,
    const lanelet::ConstLineStrings3d & all_parking_spaces)>(
    "getLinkedParkingSpaces", lanelet::utils::query::getLinkedParkingSpaces);
  bp::def<lanelet::ConstLanelets(
    const lanelet::ConstPolygon3d & parking_lot, const lanelet::ConstLanelets & all_road_lanelets)>(
    "getLinkedLanelets", lanelet::utils::query::getLinkedLanelets);
  bp::def("stopLinesLanelets", lanelet::utils::query::stopLinesLanelets);
  bp::def("stopLinesLanelet", lanelet::utils::query::stopLinesLanelet);
  bp::def(
    "stopSignStopLines", lanelet::utils::query::stopSignStopLines, stopSignStopLines_overload());
  bp::def<lanelet::ConstLanelets(
    const lanelet::ConstLanelets &, const lanelet::BasicPoint2d &, const double)>(
    "getLaneletsWithinRange", lanelet::utils::query::getLaneletsWithinRange);
  bp::def<lanelet::ConstLanelets(
    const lanelet::ConstLanelets &, const std::string &, const double)>(
    "getLaneletsWithinRange_point", ::getLaneletsWithinRange_point);  // depends on ros msg
  bp::def<lanelet::ConstLanelets(
    const lanelet::routing::RoutingGraphPtr &, const lanelet::ConstLanelet &)>(
    "getLaneChangeableNeighbors", lanelet::utils::query::getLaneChangeableNeighbors);
  bp::def<lanelet::ConstLanelets(
    const lanelet::routing::RoutingGraphPtr &, const lanelet::ConstLanelets &,
    const std::string &)>(
    "getLaneChangeableNeighbors_point", ::getLaneChangeableNeighbors_point);  // depends on ros msg
  bp::def<lanelet::ConstLanelets(
    const lanelet::routing::RoutingGraphPtr &, const lanelet::ConstLanelet &)>(
    "getAllNeighbors", lanelet::utils::query::getAllNeighbors);
  bp::def<lanelet::ConstLanelets(
    const lanelet::routing::RoutingGraphPtr &, const lanelet::ConstLanelets &,
    const std::string &)>("getAllNeighbors_point", ::getAllNeighbors_point);  // depends on ros msg
  bp::def("getAllNeighborsLeft", lanelet::utils::query::getAllNeighborsLeft);
  bp::def("getAllNeighborsRight", lanelet::utils::query::getAllNeighborsRight);
  bp::def("getClosestLanelet", ::getClosestLanelet);  // depends on ros msg
  bp::def(
    "getClosestLaneletWithConstrains", ::getClosestLaneletWithConstrains,
    getClosestLaneletWithConstrains_overload());                    // depends on ros msg
  bp::def("getCurrentLanelets_point", ::getCurrentLanelets_point);  // depends on ros msg
  bp::def("getCurrentLanelets_pose", ::getCurrentLanelets_pose);    // depends on ros msg
  // NOTE: this is required for iterating getCurrentLanelets return value directly
  bp::class_<lanelet::ConstLanelets>("lanelet::ConstLanelets")
    .def(bp::vector_indexing_suite<lanelet::ConstLanelets>());
  // NOTE: this is required for return-type of getSucceeding/PrecedingLaneletSequences
  bp::class_<std::vector<lanelet::ConstLanelets>>("std::vector<lanelet::ConstLanelets>")
    .def(bp::vector_indexing_suite<std::vector<lanelet::ConstLanelets>>());
  bp::def("getSucceedingLaneletSequences", lanelet::utils::query::getSucceedingLaneletSequences);
  bp::def(
    "getPrecedingLaneletSequences", lanelet::utils::query::getPrecedingLaneletSequences,
    getPrecedingLaneletSequences_overload());
}

// NOLINTEND(readability-identifier-naming)
