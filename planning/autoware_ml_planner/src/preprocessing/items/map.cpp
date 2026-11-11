// Copyright 2024 TIER IV, Inc.
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

#include "autoware/ml_planner/preprocessing/items/map.hpp"

#include "autoware/ml_planner/constants.hpp"
#include "autoware/ml_planner/dimensions.hpp"
#include "autoware_utils_math/normalization.hpp"
#include "autoware_utils_math/unit_conversion.hpp"

#include <autoware_lanelet2_extension/regulatory_elements/Forward.hpp>
#include <autoware_lanelet2_extension/regulatory_elements/road_marking.hpp>  // for lanelet::autoware::RoadMarking
#include <xtensor/xbuilder.hpp>

#include <geometry_msgs/msg/detail/point__struct.hpp>

#include <Eigen/src/Core/Matrix.h>
#include <lanelet2_core/Forward.h>
#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_core/geometry/Point.h>
#include <lanelet2_core/geometry/Polygon.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <iterator>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace autoware::ml_planner
{

namespace
{

std::vector<LanePoint> interpolate_points(const std::vector<LanePoint> & input, size_t num_points)
{
  if (input.size() < 2 || num_points < 2) {
    std::cerr << "Need at least 2 input points\n";
    return input;
  }
  // Step 1: Compute cumulative distances
  std::vector<double> arc_lengths(input.size(), 0.0);
  for (size_t i = 1; i < input.size(); ++i) {
    arc_lengths[i] = arc_lengths[i - 1] + (input[i] - input[i - 1]).norm();
  }
  const double total_length = arc_lengths.back();

  // Step 2: Generate target arc lengths
  std::vector<LanePoint> result;
  result.reserve(num_points);

  // Always include the first point
  result.push_back(input.front());

  // Generate interior points
  if (num_points == 2) {
    // Always include the last point
    result.push_back(input.back());
    return result;
  }

  const double step = total_length / static_cast<double>(num_points - 1);
  size_t seg_idx = 0;

  for (size_t i = 1; i < num_points - 1; ++i) {
    const double target = static_cast<double>(i) * step;

    // Find the correct segment containing the target arc length
    while (seg_idx + 1 < arc_lengths.size() && arc_lengths[seg_idx + 1] < target) {
      ++seg_idx;
    }

    // Ensure we don't go past the last segment
    if (seg_idx >= arc_lengths.size() - 1) {
      seg_idx = arc_lengths.size() - 2;
    }

    // Interpolate between input[seg_idx] and input[seg_idx + 1]
    const double seg_start = arc_lengths[seg_idx];
    const double seg_end = arc_lengths[seg_idx + 1];
    const double seg_length = seg_end - seg_start;

    // Calculate interpolation parameter, handling zero-length segments
    constexpr double epsilon = 1e-6;
    const double safe_seg_length = std::max(seg_length, epsilon);
    const double t = std::clamp((target - seg_start) / safe_seg_length, 0.0, 1.0);
    const LanePoint new_point = input[seg_idx] + t * (input[seg_idx + 1] - input[seg_idx]);
    result.push_back(new_point);
  }
  // Always include the last point
  result.push_back(input.back());

  return result;
}

/**
 * @brief Split a line string into pieces of exactly num_points points, keeping every original
 * point.
 *
 * 1. Every original edge is divided evenly into the fewest steps of at most max_step_m.
 * 2. The steps are divided as evenly as possible into the fewest pieces of at most
 *    num_points - 1 steps; consecutive pieces share their end point and never overlap.
 * 3. Each piece is filled up to num_points - 1 steps: one step at a time goes to the part of an
 *    original edge in the piece with the longest step, and that part is divided evenly again.
 *
 * Every step is at most max_step_m, and steps within one original edge stay equal, so the
 * spacing stays close to max_step_m except where the original points are closer than that.
 */
std::vector<Polyline> split_line_string(
  const Polyline & input, const size_t num_points, const double max_step_m)
{
  if (input.empty() || num_points < 2) {
    return {};
  }

  constexpr double k_epsilon = 1e-6;
  Polyline points{input.front()};
  for (const auto & point : input) {
    if ((point - points.back()).norm() > k_epsilon) {
      points.push_back(point);
    }
  }
  if (points.size() == 1) {
    return {Polyline(num_points, points.front())};
  }

  // A straight part of the line string, divided evenly into n_steps steps.
  struct Part
  {
    LanePoint from;
    LanePoint to;
    size_t n_steps;

    double step_length() const { return (to - from).norm() / static_cast<double>(n_steps); }
    void append_to(Polyline & polyline) const
    {
      for (size_t step = 1; step < n_steps; ++step) {
        const double t = static_cast<double>(step) / static_cast<double>(n_steps);
        polyline.emplace_back(from + t * (to - from));
      }
      polyline.push_back(to);
    }
  };

  // 1. Divide every original edge into steps of at most max_step_m.
  const double safe_max_step_m = std::max(max_step_m, k_epsilon);
  Polyline dense{points.front()};
  std::vector<size_t> edge_of_step;
  for (size_t i = 1; i < points.size(); ++i) {
    const Part edge{
      points[i - 1], points[i],
      static_cast<size_t>(
        std::max(1.0, std::ceil((points[i] - points[i - 1]).norm() / safe_max_step_m)))};
    edge.append_to(dense);
    edge_of_step.insert(edge_of_step.end(), edge.n_steps, i - 1);
  }

  // 2. Fewest pieces such that each holds at most num_points - 1 steps.
  const size_t n_steps = edge_of_step.size();
  const size_t n_pieces = (n_steps + num_points - 2) / (num_points - 1);
  std::vector<Polyline> result;
  result.reserve(n_pieces);
  for (size_t piece_index = 0; piece_index < n_pieces; ++piece_index) {
    const size_t begin = piece_index * n_steps / n_pieces;
    const size_t end = (piece_index + 1) * n_steps / n_pieces;
    std::vector<Part> parts;
    for (size_t step = begin; step < end; ++step) {
      if (!parts.empty() && edge_of_step[step] == edge_of_step[step - 1]) {
        parts.back().to = dense[step + 1];
        ++parts.back().n_steps;
      } else {
        parts.push_back(Part{dense[step], dense[step + 1], 1});
      }
    }

    // 3. Fill up to num_points - 1 steps, dividing the part with the longest step again.
    for (size_t step_count = end - begin; step_count < num_points - 1; ++step_count) {
      const auto longest = std::max_element(
        parts.begin(), parts.end(),
        [](const Part & a, const Part & b) { return a.step_length() < b.step_length(); });
      ++longest->n_steps;
    }

    Polyline piece{parts.front().from};
    piece.reserve(num_points);
    for (const auto & part : parts) {
      part.append_to(piece);
    }
    result.push_back(std::move(piece));
  }
  return result;
}

template <typename T>
std::vector<LanePoint> convert_to_polyline(const T & line_string) noexcept
{
  std::vector<LanePoint> output;
  output.reserve(line_string.size());
  for (const auto & point : line_string) {
    output.emplace_back(point.x(), point.y(), point.z());
  }
  return output;
}
}  // namespace

LaneletMap convert_to_internal_lanelet_map(const lanelet::LaneletMapConstPtr lanelet_map_ptr)
{
  LaneletMap lanelet_map;
  lanelet_map.lane_segments.reserve(lanelet_map_ptr->laneletLayer.size());
  lanelet_map.intersection_areas.reserve(lanelet_map_ptr->polygonLayer.size());
  lanelet_map.stop_lines.reserve(lanelet_map_ptr->lineStringLayer.size());
  lanelet_map.road_borders.reserve(lanelet_map_ptr->lineStringLayer.size());

  // parse lanelet layers
  for (const auto & lanelet : lanelet_map_ptr->laneletLayer) {
    if (!lanelet.hasAttribute("subtype")) {
      continue;
    }
    const auto lanelet_subtype = lanelet.attribute("subtype").as<std::string>();
    if (!lanelet_subtype || ACCEPTABLE_LANE_SUBTYPES.count(lanelet_subtype.value()) == 0) {
      continue;
    }
    const Polyline centerline(
      interpolate_points(convert_to_polyline(lanelet.centerline3d()), POINTS_PER_SEGMENT));
    const Polyline left_boundary(
      interpolate_points(convert_to_polyline(lanelet.leftBound3d()), POINTS_PER_SEGMENT));
    const Polyline right_boundary(
      interpolate_points(convert_to_polyline(lanelet.rightBound3d()), POINTS_PER_SEGMENT));

    LanePoint mean_point(0.0, 0.0, 0.0);
    for (const LanePoint & p : centerline) {
      mean_point += p;
    }
    mean_point /= static_cast<double>(centerline.size());

    const std::string left_line_type_str = lanelet.leftBound().attributeOr("type", "");
    const std::string right_line_type_str = lanelet.rightBound().attributeOr("type", "");
    const LineType left_line_type =
      (LINE_TYPE_MAP.count(left_line_type_str) ? LINE_TYPE_MAP.at(left_line_type_str)
                                               : LINE_TYPE_VIRTUAL);
    const LineType right_line_type =
      (LINE_TYPE_MAP.count(right_line_type_str) ? LINE_TYPE_MAP.at(right_line_type_str)
                                                : LINE_TYPE_VIRTUAL);

    const lanelet::AttributeMap & attrs = lanelet.attributes();
    const std::optional<float> speed_limit_mps =
      attrs.find("speed_limit") != attrs.end()
        ? std::make_optional(
            autoware_utils_math::kmph2mps(std::stof(attrs.at("speed_limit").value())))
        : std::nullopt;

    int64_t turn_direction = LaneSegment::TURN_DIRECTION_NONE;
    const std::map<std::string, int64_t> turn_direction_map = {
      {"straight", LaneSegment::TURN_DIRECTION_STRAIGHT},
      {"left", LaneSegment::TURN_DIRECTION_LEFT},
      {"right", LaneSegment::TURN_DIRECTION_RIGHT}};
    if (attrs.find("turn_direction") != attrs.end()) {
      const std::string turn_direction_str = attrs.at("turn_direction").value();
      const auto itr = turn_direction_map.find(turn_direction_str);
      if (itr != turn_direction_map.end()) {
        turn_direction = itr->second;
      }
    }

    const std::vector<lanelet::format_v2::TrafficLightConstPtr> traffic_light_list =
      lanelet.regulatoryElementsAs<const lanelet::TrafficLight>();

    // According to the definition, the number of elements in the traffic_light_list should be
    // either 0 or 1; however, this is not always the case with older map data. Therefore, if there
    // are multiple elements, we only use the first element.
    const int64_t traffic_light_id =
      (traffic_light_list.empty() ? LaneSegment::TRAFFIC_LIGHT_ID_NONE
                                  : traffic_light_list.front()->id());

    lanelet_map.lane_segments.emplace_back(
      lanelet.id(), centerline, left_boundary, right_boundary, mean_point, left_line_type,
      right_line_type, speed_limit_mps, turn_direction, traffic_light_id);
  }

  // parse polygon layers
  for (const auto & polygon : lanelet_map_ptr->polygonLayer) {
    const std::string polygon_type = polygon.attributeOr("type", "");
    if (polygon_type != "intersection_area") {
      continue;
    }
    const std::vector<LanePoint> points(interpolate_points(
      convert_to_polyline(polygon.basicLineString()), POINTS_PER_INTERSECTION_AREA));
    lanelet_map.intersection_areas.push_back(MapPolyline{points});
  }

  // parse line string layers
  for (const auto & line_string : lanelet_map_ptr->lineStringLayer) {
    const std::string line_string_type = line_string.attributeOr("type", "");
    const Polyline points = convert_to_polyline(line_string);
    if (line_string_type == "stop_line") {
      if (points.size() >= 2) {
        lanelet_map.stop_lines.push_back(MapPolyline{{points.front(), points.back()}});
      }
    } else if (line_string_type == "road_border") {
      const auto segments =
        split_line_string(points, POINTS_PER_ROAD_BORDER, constants::ROAD_BORDER_MAX_STEP_M);
      for (const auto & segment : segments) {
        lanelet_map.road_borders.push_back(MapPolyline{segment});
      }
    }
  }

  return lanelet_map;
}
}  // namespace autoware::ml_planner

namespace autoware::ml_planner::preprocess
{

// Internal functions declaration
namespace
{
using autoware_perception_msgs::msg::TrafficLightElement;

std::map<lanelet::Id, size_t> create_lane_id_to_array_index_map(
  const std::vector<LaneSegment> & lane_segments);
bool is_segment_inside(const LaneSegment & segment, const double center_x, const double center_y);
bool is_on_ego_level(const Polyline & points_in_ego_frame);
std::optional<size_t> find_closest_segment_index(
  const std::vector<LaneSegment> & lane_segments, const std::vector<int64_t> & segment_indices,
  const lanelet::BasicPoint2d & point, const double yaw, const double yaw_threshold);

xt::xarray<float> create_polyline_tensor(
  const std::vector<MapPolyline> & elements, const Eigen::Matrix4d & transform_matrix,
  double center_x, double center_y, int64_t num_elements, int64_t num_points);
}  // namespace

// LaneSegmentContext implementation
LaneSegmentContext::LaneSegmentContext(
  const std::shared_ptr<const lanelet::LaneletMap> & lanelet_map_ptr)
: lanelet_map_(convert_to_internal_lanelet_map(lanelet_map_ptr)),
  lanelet_id_to_array_index_(create_lane_id_to_array_index_map(lanelet_map_.lane_segments))
{
  if (lanelet_map_.lane_segments.empty()) {
    throw std::runtime_error("No lane segments found in the map");
  }
}

xt::xarray<float> LaneSegmentContext::create_intersection_area_tensor(
  const Eigen::Matrix4d & transform_matrix, const double center_x, const double center_y) const
{
  return create_polyline_tensor(
    lanelet_map_.intersection_areas, transform_matrix, center_x, center_y, NUM_INTERSECTION_AREAS,
    POINTS_PER_INTERSECTION_AREA);
}

xt::xarray<float> LaneSegmentContext::create_stop_line_tensor(
  const Eigen::Matrix4d & transform_matrix, const double center_x, const double center_y) const
{
  return create_polyline_tensor(
    lanelet_map_.stop_lines, transform_matrix, center_x, center_y, NUM_STOP_LINES,
    POINTS_PER_STOP_LINE);
}

xt::xarray<float> LaneSegmentContext::create_road_border_tensor(
  const Eigen::Matrix4d & transform_matrix, const double center_x, const double center_y) const
{
  return create_polyline_tensor(
    lanelet_map_.road_borders, transform_matrix, center_x, center_y, NUM_ROAD_BORDERS,
    POINTS_PER_ROAD_BORDER);
}

std::vector<int64_t> LaneSegmentContext::select_route_segment_indices(
  const LaneletRoute & route, const double center_x, const double center_y,
  [[maybe_unused]] const double center_z, const double center_yaw, const int64_t max_segments) const
{
  std::vector<int64_t> array_indices;
  for (const auto & route_segment : route.segments) {
    // add index
    const int64_t lanelet_id = route_segment.preferred_primitive.id;
    if (lanelet_id_to_array_index_.count(lanelet_id) == 0) {
      continue;
    }
    array_indices.push_back(static_cast<int64_t>(lanelet_id_to_array_index_.at(lanelet_id)));
  }

  // calculate closest index
  // prefer lanelets aligned with ego yaw to avoid picking a crossing part of the route
  const lanelet::BasicPoint2d center(center_x, center_y);
  auto closest_index = find_closest_segment_index(
    lanelet_map_.lane_segments, array_indices, center, center_yaw,
    autoware::ml_planner::constants::MAX_ROUTE_SEGMENT_YAW_DIFF_RAD);
  if (!closest_index) {
    closest_index = find_closest_segment_index(
      lanelet_map_.lane_segments, array_indices, center, center_yaw,
      std::numeric_limits<double>::max());
  }

  std::vector<int64_t> selected_indices;
  bool has_entered_valid_region = false;

  // Select route segment indices
  // closest_index is used to index array_indices, which excludes
  // route segments that are unavailable in the processed map.
  for (size_t i = closest_index.value_or(0); i < array_indices.size(); ++i) {
    const int64_t segment_idx = array_indices[i];

    if (!is_segment_inside(lanelet_map_.lane_segments[segment_idx], center_x, center_y)) {
      if (has_entered_valid_region) {
        break;
      }
      continue;
    }

    has_entered_valid_region = true;

    selected_indices.push_back(segment_idx);
    if (selected_indices.size() >= static_cast<size_t>(max_segments)) {
      break;
    }
  }

  return selected_indices;
}

autoware_perception_msgs::msg::TrafficLightGroup
LaneSegmentContext::get_first_traffic_light_on_route(
  const LaneletRoute & route, const double center_x, const double center_y, const double center_z,
  const double center_yaw,
  const std::map<lanelet::Id, TrafficSignalStamped> & traffic_light_id_map) const
{
  autoware_perception_msgs::msg::TrafficLightGroup result;
  result.traffic_light_group_id = 0;

  const std::vector<int64_t> segment_indices = select_route_segment_indices(
    route, center_x, center_y, center_z, center_yaw, NUM_SEGMENTS_IN_ROUTE);

  for (const int64_t segment_idx : segment_indices) {
    const LaneSegment & segment = lanelet_map_.lane_segments[segment_idx];
    if (segment.traffic_light_id == LaneSegment::TRAFFIC_LIGHT_ID_NONE) {
      continue;
    }

    const auto it = traffic_light_id_map.find(segment.traffic_light_id);
    if (it != traffic_light_id_map.end()) {
      result = it->second.signal;
    } else {
      result.traffic_light_group_id = static_cast<int64_t>(segment.traffic_light_id);
      autoware_perception_msgs::msg::TrafficLightElement unknown_element;
      unknown_element.color = TrafficLightElement::UNKNOWN;
      unknown_element.shape = TrafficLightElement::UNKNOWN;
      unknown_element.status = TrafficLightElement::UNKNOWN;
      unknown_element.confidence = 0.0f;
      result.elements = {unknown_element};
    }
    return result;
  }

  return result;
}

std::vector<std::pair<lanelet::Id, int64_t>> LaneSegmentContext::get_traffic_light_ids(
  const std::vector<int64_t> & segment_indices) const
{
  std::vector<std::pair<lanelet::Id, int64_t>> result;
  result.reserve(segment_indices.size());
  for (const int64_t segment_idx : segment_indices) {
    const LaneSegment & segment = lanelet_map_.lane_segments[segment_idx];
    result.emplace_back(segment.traffic_light_id, segment.turn_direction);
  }
  return result;
}

std::vector<int64_t> LaneSegmentContext::select_lane_segment_indices(
  const Eigen::Matrix4d & transform_matrix, const double center_x, const double center_y,
  const int64_t max_segments) const
{
  struct ColWithDistance
  {
    int64_t index;           //!< Column index in the input matrix.
    float distance_squared;  //!< Squared distance from the center.
  };

  auto to_ego_frame = [&](const LanePoint & point) {
    const Eigen::Vector4d transformed_point =
      transform_matrix * Eigen::Vector4d(point.x(), point.y(), point.z(), 1.0);
    return LanePoint(transformed_point.x(), transformed_point.y(), transformed_point.z());
  };
  auto calc_distance = [&](const LanePoint & point) {
    const LanePoint transformed_point = to_ego_frame(point);
    return std::hypot(transformed_point.x(), transformed_point.y());
  };

  // Step 1: Compute distances
  std::vector<ColWithDistance> distances;
  distances.reserve(lanelet_map_.lane_segments.size());

  for (size_t i = 0; i < lanelet_map_.lane_segments.size(); ++i) {
    const LaneSegment & segment = lanelet_map_.lane_segments[i];

    if (!is_segment_inside(segment, center_x, center_y)) {
      continue;
    }

    const std::vector<LanePoint> & centerline = segment.centerline;

    if (centerline.size() < 2) {
      continue;
    }

    Polyline centerline_in_ego_frame;
    centerline_in_ego_frame.reserve(centerline.size());
    std::transform(
      centerline.begin(), centerline.end(), std::back_inserter(centerline_in_ego_frame),
      to_ego_frame);
    if (!is_on_ego_level(centerline_in_ego_frame)) {
      continue;
    }

    // Approximate distance using the closest of the first and last points
    // Note: Because the last point (centerline.size() - 1) of the centerline is the same as the
    // first point of the next segment, we use (centerline.size() - 2) to avoid obtaining the same
    // distance for adjacent segments.
    const float distance_squared =
      std::min(calc_distance(centerline.front()), calc_distance(centerline[centerline.size() - 2]));
    distances.push_back({static_cast<int64_t>(i), distance_squared});
  }

  // Step 2: Sort indices by distance
  std::sort(distances.begin(), distances.end(), [](const auto & a, const auto & b) {
    return a.distance_squared < b.distance_squared;
  });

  // Step 3: Select indices that are inside the mask
  std::vector<int64_t> selected_indices;
  for (const ColWithDistance & distance : distances) {
    selected_indices.push_back(distance.index);
    if (selected_indices.size() >= static_cast<size_t>(max_segments)) {
      break;
    }
  }

  return selected_indices;
}

std::tuple<xt::xarray<float>, xt::xarray<float>, xt::xarray<float>>
LaneSegmentContext::create_tensor_data_from_indices(
  const Eigen::Matrix4d & transform_matrix, const std::vector<int64_t> & segment_indices,
  const int64_t max_segments) const
{
  xt::xarray<float> tensor_data = xt::zeros<float>(
    {static_cast<size_t>(max_segments), static_cast<size_t>(POINTS_PER_SEGMENT),
     static_cast<size_t>(SEGMENT_POINT_DIM)});
  xt::xarray<float> speed_limit_tensor =
    xt::zeros<float>({static_cast<size_t>(max_segments), size_t{1}});
  xt::xarray<float> lane_type_tensor =
    xt::zeros<float>({static_cast<size_t>(max_segments), static_cast<size_t>(LANE_TYPE_DIM)});

  auto convert_to_vector4d = [](const LanePoint & point) {
    return Eigen::Vector4d(point.x(), point.y(), point.z(), 1.0);
  };

  auto encode = [](const int64_t line_type) {
    Eigen::Vector<double, LINE_TYPE_NUM> one_hot = Eigen::Vector<double, LINE_TYPE_NUM>::Zero();
    if (line_type >= 0 && line_type < LINE_TYPE_NUM) {
      one_hot[line_type] = 1.0;
    }
    return one_hot;
  };

  int64_t added_segments = 0;
  for (const int64_t segment_idx : segment_indices) {
    if (added_segments >= max_segments) {
      break;
    }

    const LaneSegment & lane_segment = lanelet_map_.lane_segments[segment_idx];

    // Check if segment has valid data
    if (
      lane_segment.centerline.empty() || lane_segment.left_boundary.empty() ||
      lane_segment.right_boundary.empty()) {
      continue;
    }

    const std::vector<LanePoint> & centerline = lane_segment.centerline;
    const std::vector<LanePoint> & left_boundary = lane_segment.left_boundary;
    const std::vector<LanePoint> & right_boundary = lane_segment.right_boundary;

    if (
      centerline.size() != POINTS_PER_SEGMENT || left_boundary.size() != POINTS_PER_SEGMENT ||
      right_boundary.size() != POINTS_PER_SEGMENT) {
      continue;
    }

    const Eigen::Vector<double, LINE_TYPE_NUM> lt_left = encode(lane_segment.left_line_type);
    const Eigen::Vector<double, LINE_TYPE_NUM> lt_right = encode(lane_segment.right_line_type);

    for (int64_t type = 0; type < LINE_TYPE_NUM; ++type) {
      lane_type_tensor(added_segments, LINE_TYPE_LEFT_START + type) =
        static_cast<float>(lt_left[type]);
      lane_type_tensor(added_segments, LINE_TYPE_RIGHT_START + type) =
        static_cast<float>(lt_right[type]);
    }

    // Process each point in the segment
    for (int64_t i = 0; i < POINTS_PER_SEGMENT; ++i) {
      // Center (0, 1)
      const Eigen::Vector4d center = transform_matrix * convert_to_vector4d(centerline[i]);
      tensor_data(added_segments, i, X) = static_cast<float>(center.x());
      tensor_data(added_segments, i, Y) = static_cast<float>(center.y());

      // Left (2, 3)
      const Eigen::Vector4d left = transform_matrix * convert_to_vector4d(left_boundary[i]);
      tensor_data(added_segments, i, LB_X) = static_cast<float>(left.x() - center.x());
      tensor_data(added_segments, i, LB_Y) = static_cast<float>(left.y() - center.y());

      // Right (4, 5)
      const Eigen::Vector4d right = transform_matrix * convert_to_vector4d(right_boundary[i]);
      tensor_data(added_segments, i, RB_X) = static_cast<float>(right.x() - center.x());
      tensor_data(added_segments, i, RB_Y) = static_cast<float>(right.y() - center.y());
    }

    speed_limit_tensor(added_segments, 0) = lane_segment.speed_limit_mps.value_or(0.0f);
    ++added_segments;
  }

  return {std::move(tensor_data), std::move(lane_type_tensor), std::move(speed_limit_tensor)};
}

namespace
{
xt::xarray<float> create_polyline_tensor(
  const std::vector<MapPolyline> & elements, const Eigen::Matrix4d & transform_matrix,
  const double center_x, const double center_y, const int64_t num_elements,
  const int64_t num_points)
{
  using autoware::ml_planner::constants::LANE_MASK_RANGE_M;

  struct PolylineWithDistance
  {
    Polyline points;
    double min_distance;
  };
  std::vector<PolylineWithDistance> selected;
  selected.reserve(elements.size());

  for (const auto & element : elements) {
    const bool is_inside =
      std::any_of(element.points.begin(), element.points.end(), [&](const LanePoint & point) {
        return point.x() > center_x - LANE_MASK_RANGE_M &&
               point.x() < center_x + LANE_MASK_RANGE_M &&
               point.y() > center_y - LANE_MASK_RANGE_M && point.y() < center_y + LANE_MASK_RANGE_M;
      });
    if (!is_inside) {
      continue;
    }

    Polyline transformed;
    transformed.reserve(element.points.size());
    double min_distance = std::numeric_limits<double>::max();
    for (const auto & point : element.points) {
      const Eigen::Vector4d transformed_point =
        transform_matrix * Eigen::Vector4d(point.x(), point.y(), point.z(), 1.0);
      transformed.emplace_back(transformed_point.x(), transformed_point.y(), transformed_point.z());
      min_distance =
        std::min(min_distance, std::hypot(transformed_point.x(), transformed_point.y()));
    }
    if (!is_on_ego_level(transformed)) {
      continue;
    }
    selected.push_back({std::move(transformed), min_distance});
  }

  std::sort(
    selected.begin(), selected.end(),
    [](const PolylineWithDistance & a, const PolylineWithDistance & b) {
      return a.min_distance < b.min_distance;
    });

  xt::xarray<float> tensor = xt::zeros<float>(
    {static_cast<size_t>(num_elements), static_cast<size_t>(num_points), size_t{2}});
  const size_t count = std::min(static_cast<size_t>(num_elements), selected.size());
  for (size_t i = 0; i < count; ++i) {
    const size_t point_count = std::min(static_cast<size_t>(num_points), selected[i].points.size());
    for (size_t j = 0; j < point_count; ++j) {
      tensor(i, j, 0) = static_cast<float>(selected[i].points[j].x());
      tensor(i, j, 1) = static_cast<float>(selected[i].points[j].y());
    }
  }
  return tensor;
}
}  // namespace

// Internal functions implementation
namespace
{

std::map<lanelet::Id, size_t> create_lane_id_to_array_index_map(
  const std::vector<LaneSegment> & lane_segments)
{
  std::map<lanelet::Id, size_t> lane_id_to_index;
  for (size_t i = 0; i < lane_segments.size(); ++i) {
    lane_id_to_index[lane_segments[i].id] = i;
  }
  return lane_id_to_index;
}

bool is_segment_inside(const LaneSegment & segment, const double center_x, const double center_y)
{
  for (const auto & point : segment.centerline) {
    if (
      std::abs(point.x() - center_x) <= autoware::ml_planner::constants::LANE_MASK_RANGE_M &&
      std::abs(point.y() - center_y) <= autoware::ml_planner::constants::LANE_MASK_RANGE_M) {
      return true;
    }
  }

  return false;
}

bool is_on_ego_level(const Polyline & points_in_ego_frame)
{
  const auto closest = std::min_element(
    points_in_ego_frame.begin(), points_in_ego_frame.end(),
    [](const LanePoint & a, const LanePoint & b) {
      return std::hypot(a.x(), a.y()) < std::hypot(b.x(), b.y());
    });
  using autoware::ml_planner::constants::MAX_SAME_LEVEL_HEIGHT_DIFF_M;
  using autoware::ml_planner::constants::SAME_LEVEL_HEIGHT_DIFF_PER_DISTANCE;
  if (closest == points_in_ego_frame.end()) {
    return false;
  }
  const double distance = std::hypot(closest->x(), closest->y());
  return std::abs(closest->z()) <=
         MAX_SAME_LEVEL_HEIGHT_DIFF_M + SAME_LEVEL_HEIGHT_DIFF_PER_DISTANCE * distance;
}

std::optional<size_t> find_closest_segment_index(
  const std::vector<LaneSegment> & lane_segments, const std::vector<int64_t> & segment_indices,
  const lanelet::BasicPoint2d & point, const double yaw, const double yaw_threshold)
{
  const auto to_2d = [](const LanePoint & p) { return lanelet::BasicPoint2d(p.x(), p.y()); };

  std::optional<size_t> closest_index;
  double min_distance = std::numeric_limits<double>::max();
  for (size_t i = 0; i < segment_indices.size(); ++i) {
    const LaneSegment & lane_segment = lane_segments[segment_indices[i]];

    lanelet::BasicPolygon2d polygon;
    std::transform(
      lane_segment.left_boundary.begin(), lane_segment.left_boundary.end(),
      std::back_inserter(polygon), to_2d);
    std::transform(
      lane_segment.right_boundary.rbegin(), lane_segment.right_boundary.rend(),
      std::back_inserter(polygon), to_2d);
    const double distance = lanelet::geometry::distance2d(polygon, point);
    if (distance >= min_distance) {
      continue;
    }

    lanelet::BasicLineString2d centerline;
    std::transform(
      lane_segment.centerline.begin(), lane_segment.centerline.end(),
      std::back_inserter(centerline), to_2d);
    const auto segment = lanelet::geometry::closestSegment(centerline, point);
    const double lanelet_yaw =
      std::atan2(segment.second.y() - segment.first.y(), segment.second.x() - segment.first.x());
    if (std::abs(autoware_utils_math::normalize_radian(lanelet_yaw - yaw)) > yaw_threshold) {
      continue;
    }
    min_distance = distance;
    closest_index = i;
  }
  return closest_index;
}

}  // namespace

TrafficLightStatus identify_current_light_status(
  const int64_t turn_direction, const std::vector<TrafficLightElement> & traffic_light_elements)
{
  auto is_arrow_shape = [](const uint8_t shape) {
    return shape == TrafficLightElement::LEFT_ARROW || shape == TrafficLightElement::RIGHT_ARROW ||
           shape == TrafficLightElement::UP_ARROW || shape == TrafficLightElement::DOWN_ARROW ||
           shape == TrafficLightElement::DOWN_LEFT_ARROW ||
           shape == TrafficLightElement::DOWN_RIGHT_ARROW;
  };
  auto to_status = [&](const TrafficLightElement & element) {
    return TrafficLightStatus{element.color, is_arrow_shape(element.shape)};
  };

  // Filter out ineffective elements (color == 0 which is UNKNOWN)
  std::vector<TrafficLightElement> effective_elements;
  for (const auto & element : traffic_light_elements) {
    if (element.color != TrafficLightElement::UNKNOWN) {
      effective_elements.push_back(element);
    }
  }

  // If no effective elements, return UNKNOWN (0)
  if (effective_elements.empty()) {
    return TrafficLightStatus{};
  }

  // If only one effective element, return its state
  if (effective_elements.size() == 1) {
    return to_status(effective_elements[0]);
  }

  // For multiple elements, find the one that matches the turn direction
  // Map turn direction to corresponding arrow shape
  const std::map<int64_t, uint8_t> direction_to_shape_map = {
    {LaneSegment::TURN_DIRECTION_NONE, TrafficLightElement::UNKNOWN},       // none
    {LaneSegment::TURN_DIRECTION_STRAIGHT, TrafficLightElement::UP_ARROW},  // straight
    {LaneSegment::TURN_DIRECTION_LEFT, TrafficLightElement::LEFT_ARROW},    // left
    {LaneSegment::TURN_DIRECTION_RIGHT, TrafficLightElement::RIGHT_ARROW}   // right
  };

  const auto target_shape_iter = direction_to_shape_map.find(turn_direction);
  const uint8_t target_shape = (target_shape_iter != direction_to_shape_map.end())
                                 ? target_shape_iter->second
                                 : TrafficLightElement::UNKNOWN;

  // If multiple matching elements, take the one with highest confidence
  auto get_max_confidence_status = [&](const std::vector<TrafficLightElement> & elements) {
    return to_status(*std::max_element(
      elements.begin(), elements.end(),
      [](const TrafficLightElement & a, const TrafficLightElement & b) {
        return a.confidence < b.confidence;
      }));
  };

  // First priority: Find elements with exactly matching direction
  std::vector<TrafficLightElement> matching_elements;
  for (const TrafficLightElement & element : effective_elements) {
    if (element.shape == target_shape) {
      matching_elements.push_back(element);
    }
  }
  if (!matching_elements.empty()) {
    return get_max_confidence_status(matching_elements);
  }

  // Second priority: Find circle elements
  std::vector<TrafficLightElement> circle_elements;
  for (const TrafficLightElement & element : effective_elements) {
    if (element.shape == TrafficLightElement::CIRCLE) {
      circle_elements.push_back(element);
    }
  }
  if (!circle_elements.empty()) {
    return get_max_confidence_status(circle_elements);
  }

  // If no matching direction or circle, return the element with highest confidence
  return get_max_confidence_status(effective_elements);
}

}  // namespace autoware::ml_planner::preprocess
