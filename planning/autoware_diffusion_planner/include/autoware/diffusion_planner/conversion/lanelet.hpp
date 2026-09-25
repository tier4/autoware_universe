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

#ifndef AUTOWARE__DIFFUSION_PLANNER__CONVERSION__LANELET_HPP_
#define AUTOWARE__DIFFUSION_PLANNER__CONVERSION__LANELET_HPP_

#include <Eigen/Core>

#include <lanelet2_core/LaneletMap.h>

#include <cmath>
#include <cstdint>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace autoware::diffusion_planner
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

enum LineStringType {
  LINE_STRING_TYPE_STOP_LINE = 0,
  LINE_STRING_TYPE_ROAD_BORDER = 1,
  LINE_STRING_TYPE_NUM = 2
};

const std::map<std::string, LineStringType> LINE_STRING_TYPE_MAP = {
  {"stop_line", LINE_STRING_TYPE_STOP_LINE}, {"road_border", LINE_STRING_TYPE_ROAD_BORDER}};

enum PolygonType {
  POLYGON_TYPE_INTERSECTION_AREA = 0,
  // The upstream model's polygon tensor has one type, so POLYGON_TYPE_NUM
  // (which sizes POLYGONS_SHAPE) stays 1. A crosswalk polygon only exists
  // under MapConversionOptions::crosswalk_polygons, whose models declare two.
  POLYGON_TYPE_NUM = 1,
  POLYGON_TYPE_CROSSWALK = 1,
};

const std::map<std::string, PolygonType> POLYGON_TYPE_MAP = {
  {"intersection_area", POLYGON_TYPE_INTERSECTION_AREA}};

const std::set<std::string> ACCEPTABLE_LANE_SUBTYPES = {
  "bicycle_lane", "crosswalk", "highway", "pedestrian_lane", "road", "road_shoulder", "walkway"};

// The lanelets the OnePlanner converter (e2e-data-producer) keeps as lanes.
const std::set<std::string> DRIVABLE_LANE_SUBTYPES = {
  "bicycle_lane", "highway", "road", "road_shoulder"};

/**
 * @brief How a lanelet map becomes the model's vector-map tensors.
 *
 * The defaults are the upstream diffusion planner's. OnePlanner models are
 * trained on the e2e-data-producer conversion, which differs in four places;
 * `oneplanner_derived_v10()` reproduces it, point for point.
 */
struct MapConversionOptions
{
  // Each switch is one point where the producer's conversion differs.
  //! Centerline as the midpoint of the two resampled bounds, instead of
  //! lanelet2's centerline3d() (which returns a mapped centerline if present).
  bool centerline_from_bounds{false};
  //! Keep only DRIVABLE_LANE_SUBTYPES as lanes (else ACCEPTABLE_LANE_SUBTYPES).
  bool drivable_lanes_only{false};
  //! Crosswalk lanelets as POLYGON_TYPE_CROSSWALK polygons, outlined as the
  //! left bound followed by the right bound reversed.
  bool crosswalk_polygons{false};
  //! Resample line strings linearly in arc length (else an Akima spline).
  bool linear_line_strings{false};
  //! Order slots by (distance in whole mm, map id[, piece]) -- the producer's
  //! slot_order_key -- instead of by raw distance: forked lanelets tie exactly
  //! (88 % of frames hold one), and std::sort leaves a tie's order undefined.
  bool producer_slot_order{false};
  //! Speed limit through the producer's km/h -> mph -> m/s (x 0.621371 x 0.44704).
  bool producer_speed_limit{false};
  double line_string_max_step_m{5.0};

  //! The step is left at its default for the caller to set from its parameter.
  static MapConversionOptions oneplanner_derived_v10()
  {
    MapConversionOptions options;
    options.centerline_from_bounds = true;
    options.drivable_lanes_only = true;
    options.crosswalk_polygons = true;
    options.linear_line_strings = true;
    options.producer_slot_order = true;
    options.producer_speed_limit = true;
    return options;
  }
};

using LanePoint = Eigen::Vector3d;

//! The producer's slot_order_key distance: whole millimetres, floor(x + 0.5).
inline int64_t slot_order_mm(const double distance_m)
{
  return static_cast<int64_t>(std::floor(distance_m * 1000.0 + 0.5));
}
using Polyline = std::vector<LanePoint>;

struct Polygon
{
  std::vector<LanePoint> points;
  PolygonType type;
  int64_t id{0};  //!< Map id: the polygon's, or the crosswalk lanelet's.
};

struct LineString
{
  std::vector<LanePoint> points;
  LineStringType type;
  int64_t id{0};     //!< The source line string's map id.
  int64_t piece{0};  //!< Which equal piece of it (resample_line_string*).
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
  std::vector<Polygon> polygons;
  std::vector<LineString> line_strings;
};

/**
 * @brief Convert a lanelet map to line segment data
 * @param lanelet_map_ptr Pointer of loaded lanelet map.
 * @param options Conversion rules; the defaults are the upstream planner's.
 * @return LaneletMap
 */
[[nodiscard]] LaneletMap convert_to_internal_lanelet_map(
  const lanelet::LaneletMapConstPtr lanelet_map_ptr, const MapConversionOptions & options = {});

}  // namespace autoware::diffusion_planner

#endif  // AUTOWARE__DIFFUSION_PLANNER__CONVERSION__LANELET_HPP_
