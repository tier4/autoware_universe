// Copyright 2022 TIER IV, Inc.
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

#ifndef LANELET_FILTER__LANELET_FILTER_BASE_HPP_
#define LANELET_FILTER__LANELET_FILTER_BASE_HPP_

#include "autoware/detected_object_validation/utils/utils.hpp"
#include "autoware_lanelet2_extension/utility/utilities.hpp"
#include "autoware_utils/geometry/geometry.hpp"
#include "autoware_utils/system/stop_watch.hpp"
#include "autoware_utils_debug/debug_publisher.hpp"
#include "autoware_utils_debug/published_time_publisher.hpp"

#include <agnocast/agnocast.hpp>
#include <agnocast/node/tf2/tf2.hpp>
#include <rclcpp/rclcpp.hpp>

#include "autoware_map_msgs/msg/lanelet_map_bin.hpp"
#include <visualization_msgs/msg/marker_array.hpp>

#include <boost/geometry/index/rtree.hpp>

#include <lanelet2_core/Forward.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <limits>
#include <memory>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

namespace autoware::detected_object_validation
{
namespace lanelet_filter
{
using autoware_utils::LinearRing2d;
using autoware_utils::MultiPoint2d;
using autoware_utils::Polygon2d;

template <typename NodeT>
inline constexpr bool is_rclcpp_node_v = std::is_same_v<NodeT, rclcpp::Node>;

namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;
using Point2d = bg::model::point<double, 2, bg::cs::cartesian>;
using Box = boost::geometry::model::box<Point2d>;

struct PolygonAndLanelet
{
  lanelet::BasicPolygon2d polygon;
  lanelet::ConstLanelet lanelet;
};
using BoxAndLanelet = std::pair<Box, PolygonAndLanelet>;
using RtreeAlgo = bgi::rstar<16>;

template <typename ObjsMsgType, typename ObjMsgType, typename NodeType = rclcpp::Node>
class ObjectLaneletFilterBase : public NodeType
{
public:
  explicit ObjectLaneletFilterBase(
    const std::string & node_name, const rclcpp::NodeOptions & node_options);

private:
  using ObjectCallbackArg = std::conditional_t<
    is_rclcpp_node_v<NodeType>, typename ObjsMsgType::ConstSharedPtr,
    agnocast::ipc_shared_ptr<ObjsMsgType const>>;

  using MapCallbackArg = std::conditional_t<
    is_rclcpp_node_v<NodeType>, autoware_map_msgs::msg::LaneletMapBin::ConstSharedPtr,
    agnocast::ipc_shared_ptr<autoware_map_msgs::msg::LaneletMapBin const>>;

  void objectCallback(const ObjectCallbackArg msg);
  void mapCallback(const MapCallbackArg msg);

  void publishDebugMarkers(
    rclcpp::Time stamp, const LinearRing2d & hull, const std::vector<BoxAndLanelet> & lanelets);

  std::conditional_t<
    is_rclcpp_node_v<NodeType>, typename rclcpp::Publisher<ObjsMsgType>::SharedPtr,
    typename agnocast::Publisher<ObjsMsgType>::SharedPtr>
    object_pub_;

  std::conditional_t<
    is_rclcpp_node_v<NodeType>, rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr,
    agnocast::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr>
    viz_pub_;

  std::conditional_t<
    is_rclcpp_node_v<NodeType>,
    rclcpp::Subscription<autoware_map_msgs::msg::LaneletMapBin>::SharedPtr,
    agnocast::Subscription<autoware_map_msgs::msg::LaneletMapBin>::SharedPtr>
    map_sub_;

  std::conditional_t<
    is_rclcpp_node_v<NodeType>, typename rclcpp::Subscription<ObjsMsgType>::SharedPtr,
    typename agnocast::Subscription<ObjsMsgType>::SharedPtr>
    object_sub_;

  std::unique_ptr<autoware_utils_debug::BasicDebugPublisher<NodeType>> debug_publisher_{nullptr};
  std::unique_ptr<autoware_utils::StopWatch<std::chrono::milliseconds>> stop_watch_ptr_;

  lanelet::LaneletMapPtr lanelet_map_ptr_;
  std::string lanelet_frame_id_;

  std::conditional_t<is_rclcpp_node_v<NodeType>, tf2_ros::Buffer, agnocast::Buffer> tf_buffer_;
  std::conditional_t<
    is_rclcpp_node_v<NodeType>, std::unique_ptr<tf2_ros::TransformListener>,
    std::unique_ptr<agnocast::TransformListener>>
    tf_listener_;

  utils::FilterTargetLabel filter_target_;
  double ego_base_height_ = 0.0;
  struct FilterSettings
  {
    bool lanelet_xy_overlap_filter;

    bool lanelet_direction_filter;
    double lanelet_direction_filter_velocity_yaw_threshold;
    double lanelet_direction_filter_object_speed_threshold;

    bool lanelet_object_elevation_filter;
    double max_elevation_threshold = std::numeric_limits<double>::infinity();
    double min_elevation_threshold = -std::numeric_limits<double>::infinity();

    double lanelet_extra_margin;
    bool debug;
  } filter_settings_;

  bool filterObject(
    const ObjMsgType & transformed_object, const ObjMsgType & input_object,
    const bg::index::rtree<BoxAndLanelet, RtreeAlgo> & local_rtree,
    ObjsMsgType & output_object_msg);
  LinearRing2d getConvexHull(const ObjsMsgType &);
  Polygon2d getConvexHullFromObjectFootprint(const ObjMsgType & object);
  std::vector<BoxAndLanelet> getIntersectedLanelets(const LinearRing2d &);
  bool isObjectOverlapLanelets(
    const ObjMsgType & object, const Polygon2d & polygon,
    const std::vector<BoxAndLanelet> & lanelet_candidates);
  bool isPolygonOverlapLanelets(
    const Polygon2d & polygon, const std::vector<BoxAndLanelet> & lanelet_candidates);
  bool isSameDirectionWithLanelets(
    const ObjMsgType & object, const std::vector<BoxAndLanelet> & lanelet_candidates);
  bool isObjectAboveLanelet(
    const ObjMsgType & object, const std::vector<BoxAndLanelet> & lanelet_candidates);
  geometry_msgs::msg::Polygon setFootprint(const ObjMsgType &);

  lanelet::BasicPolygon2d getPolygon(const lanelet::ConstLanelet & lanelet);
  std::unique_ptr<autoware_utils_debug::BasicPublishedTimePublisher<NodeType>>
    published_time_publisher_;
};

}  // namespace lanelet_filter
}  // namespace autoware::detected_object_validation

#endif  // LANELET_FILTER__LANELET_FILTER_BASE_HPP_
