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

#ifndef AUTOWARE__ML_PLANNER__PREPROCESSING__ITEMS__MAP_HPP_
#define AUTOWARE__ML_PLANNER__PREPROCESSING__ITEMS__MAP_HPP_

#include "autoware/ml_planner/preprocessing/items/traffic_signals.hpp"
#include "autoware/traffic_light_utils/traffic_light_utils.hpp"

#include <Eigen/Core>
#include <xtensor/xarray.hpp>

#include <autoware_perception_msgs/msg/traffic_light_group.hpp>
#include <autoware_perception_msgs/msg/traffic_light_group_array.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <geometry_msgs/msg/detail/point__struct.hpp>

#include <Eigen/src/Core/Matrix.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/CompoundPolygon.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/LineString.h>
#include <lanelet2_core/utility/Optional.h>

#include <cstddef>
#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace autoware::ml_planner
{

enum LineType {
  LINE_TYPE_CROSSWALK = 0,
  LINE_TYPE_CURBSTONE = 1,
  LINE_TYPE_GUARD_RAIL = 2,
  LINE_TYPE_LINE_THICK = 3,
  LINE_TYPE_LINE_THIN = 4,
  LINE_TYPE_PEDESTRIAN_MARKING = 5,
  LINE_TYPE_ROAD_BORDER = 6,
  LINE_TYPE_ROAD_SHOULDER = 7,
  LINE_TYPE_VIRTUAL = 8,
  LINE_TYPE_ZEBRA_MARKING = 9,
  LINE_TYPE_NUM = 10
};

const std::map<std::string, LineType> LINE_TYPE_MAP = {
  {"crosswalk", LINE_TYPE_CROSSWALK},     {"curbstone", LINE_TYPE_CURBSTONE},
  {"guard_rail", LINE_TYPE_GUARD_RAIL},   {"line_thick", LINE_TYPE_LINE_THICK},
  {"line_thin", LINE_TYPE_LINE_THIN},     {"pedestrian_marking", LINE_TYPE_PEDESTRIAN_MARKING},
  {"road_border", LINE_TYPE_ROAD_BORDER}, {"road_shoulder", LINE_TYPE_ROAD_SHOULDER},
  {"virtual", LINE_TYPE_VIRTUAL},         {"zebra_marking", LINE_TYPE_ZEBRA_MARKING}};

const std::set<std::string> ACCEPTABLE_LANE_SUBTYPES = {
  "bicycle_lane", "crosswalk", "highway", "pedestrian_lane", "road", "road_shoulder", "walkway"};

using LanePoint = Eigen::Vector3d;
using Polyline = std::vector<LanePoint>;

struct MapPolyline
{
  std::vector<LanePoint> points;
};

struct LaneSegment
{
  int64_t id;
  Polyline centerline;
  Polyline left_boundary;
  Polyline right_boundary;
  LanePoint mean_point;
  LineType left_line_type;
  LineType right_line_type;
  std::optional<float> speed_limit_mps{std::nullopt};
  int64_t turn_direction;
  int64_t traffic_light_id;

  static constexpr int64_t TURN_DIRECTION_NONE = -1;
  static constexpr int64_t TURN_DIRECTION_STRAIGHT = 0;
  static constexpr int64_t TURN_DIRECTION_LEFT = 1;
  static constexpr int64_t TURN_DIRECTION_RIGHT = 2;

  static constexpr int64_t TRAFFIC_LIGHT_ID_NONE = -1;

  LaneSegment(
    const int64_t id, const Polyline & centerline, const Polyline & left_boundary,
    const Polyline & right_boundary, const LanePoint & mean_point, const LineType left_line_type,
    const LineType right_line_type, const std::optional<float> speed_limit_mps,
    const int64_t turn_direction, const int64_t traffic_light_id)
  : id(id),
    centerline(centerline),
    left_boundary(left_boundary),
    right_boundary(right_boundary),
    mean_point(mean_point),
    left_line_type(left_line_type),
    right_line_type(right_line_type),
    speed_limit_mps(speed_limit_mps),
    turn_direction(turn_direction),
    traffic_light_id(traffic_light_id)
  {
  }
};

struct LaneletMap
{
  std::vector<LaneSegment> lane_segments;
  std::vector<MapPolyline> intersection_areas;
  std::vector<MapPolyline> stop_lines;
  std::vector<MapPolyline> road_borders;
};

/**
 * @brief Convert a lanelet map to line segment data
 * @param lanelet_map_ptr Pointer of loaded lanelet map.
 * @return LaneletMap
 */
[[nodiscard]] LaneletMap convert_to_internal_lanelet_map(
  const lanelet::LaneletMapConstPtr lanelet_map_ptr, double line_string_max_step_m = 5.0);

}  // namespace autoware::ml_planner

namespace autoware::ml_planner::preprocess
{
using autoware::ml_planner::LanePoint;
using autoware::ml_planner::LaneSegment;
using autoware_planning_msgs::msg::LaneletRoute;
using autoware_planning_msgs::msg::LaneletSegment;

/**
 * @brief Context class that encapsulates static lane segment processing data and operations.
 *
 * This class combines the commonly used static parameters (matrix, mappings, and lanelet map)
 * that are determined at initialization time, separate from dynamic per-frame data like
 * transform_matrix and traffic_light_id_map.
 */
class LaneSegmentContext
{
public:
  /**
   * @brief Constructor that initializes the context with static data determined at initialization.
   *
   * @param lanelet_map_ptr Shared pointer to the lanelet map.
   */
  explicit LaneSegmentContext(
    const std::shared_ptr<const lanelet::LaneletMap> & lanelet_map_ptr,
    double line_string_max_step_m = 5.0);

  /**
   * @brief Select route segment indices based on route and constraints.
   *
   * @param route The lanelet route to process.
   * @param center_x X-coordinate of the center point.
   * @param center_y Y-coordinate of the center point.
   * @param center_z Z-coordinate of the center point.
   * @param max_segments Maximum number of segments to select.
   * @return Vector of lane segment indices.
   */
  std::vector<int64_t> select_route_segment_indices(
    const LaneletRoute & route, const double center_x, const double center_y, const double center_z,
    const int64_t max_segments) const;

  /**
   * @brief Select lane segment indices based on distances and constraints.
   *
   * @param center_x X-coordinate of the center point.
   * @param center_y Y-coordinate of the center point.
   * @param max_segments Maximum number of segments to select.
   * @return Vector of lane segment indices.
   */
  std::vector<int64_t> select_lane_segment_indices(
    const Eigen::Matrix4d & transform_matrix, const double center_x, const double center_y,
    const int64_t max_segments) const;

  /**
   * @brief Create tensor data from selected segment indices.
   *
   * @param transform_matrix Transformation matrix to apply to the points.
   * @param segment_indices Vector of segment indices to process.
   * @param max_segments Maximum number of segments for output tensor.
   * @return Lane geometry, lane type, and speed limit tensors.
   */
  std::tuple<xt::xarray<float>, xt::xarray<float>, xt::xarray<float>>
  create_tensor_data_from_indices(
    const Eigen::Matrix4d & transform_matrix, const std::vector<int64_t> & segment_indices,
    const int64_t max_segments) const;

  /**
   * @brief Get (traffic light id, turn_direction) of the given lane segments.
   *
   * @param segment_indices Lane segment indices (as used for the lane tensor).
   * @return One pair per index; traffic light id is
   *         LaneSegment::TRAFFIC_LIGHT_ID_NONE for segments without a light.
   */
  std::vector<std::pair<lanelet::Id, int64_t>> get_traffic_light_ids(
    const std::vector<int64_t> & segment_indices) const;

  /**
   * @brief Get the first traffic light on the route from ego position forward.
   *
   * @param route The lanelet route.
   * @param center_x X-coordinate of ego center.
   * @param center_y Y-coordinate of ego center.
   * @param center_z Z-coordinate of ego center.
   * @param traffic_light_id_map Map of traffic light IDs to signal data.
   * @return TrafficLightGroup: cached signal if perception available, UNKNOWN element if not, or
   *         empty (traffic_light_group_id=0, elements empty) if no traffic light on route.
   */
  autoware_perception_msgs::msg::TrafficLightGroup get_first_traffic_light_on_route(
    const LaneletRoute & route, const double center_x, const double center_y, const double center_z,
    const std::map<lanelet::Id, TrafficSignalStamped> & traffic_light_id_map) const;

  /**
   * @brief Get the mapping from lanelet ID to array index.
   *
   * @return Map of lanelet IDs to their corresponding array indices.
   */
  const std::map<lanelet::Id, size_t> & get_lanelet_id_to_array_index() const
  {
    return lanelet_id_to_array_index_;
  }

  // Create intersection-area and map-polyline tensor data
  xt::xarray<float> create_intersection_area_tensor(
    const Eigen::Matrix4d & transform_matrix, double center_x, double center_y) const;
  xt::xarray<float> create_stop_line_tensor(
    const Eigen::Matrix4d & transform_matrix, double center_x, double center_y) const;
  xt::xarray<float> create_road_border_tensor(
    const Eigen::Matrix4d & transform_matrix, double center_x, double center_y) const;

private:
  const autoware::ml_planner::LaneletMap lanelet_map_;
  const std::map<lanelet::Id, size_t> lanelet_id_to_array_index_;
};

/**
 * @brief Effective state of a traffic light group for a specific lane.
 */
struct TrafficLightStatus
{
  uint8_t color{autoware_perception_msgs::msg::TrafficLightElement::UNKNOWN};
  bool is_arrow{false};  ///< True when the decisive element is an arrow (not a circle).
};

/**
 * @brief Identify the effective light state of a traffic light group for a
 * lane with the given turn direction.
 */
TrafficLightStatus identify_current_light_status(
  const int64_t turn_direction,
  const std::vector<autoware_perception_msgs::msg::TrafficLightElement> & traffic_light_elements);

}  // namespace autoware::ml_planner::preprocess

#endif  // AUTOWARE__ML_PLANNER__PREPROCESSING__ITEMS__MAP_HPP_
