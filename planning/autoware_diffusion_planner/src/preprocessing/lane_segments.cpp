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

#include "autoware/diffusion_planner/preprocessing/lane_segments.hpp"

#include "autoware/diffusion_planner/constants.hpp"
#include "autoware/diffusion_planner/dimensions.hpp"

#include <autoware_lanelet2_extension/regulatory_elements/road_marking.hpp>  // for lanelet::autoware::RoadMarking
#include <autoware_lanelet2_extension/utility/query.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>

#include <Eigen/src/Core/Matrix.h>
#include <lanelet2_core/Forward.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <tuple>
#include <utility>
#include <vector>

namespace autoware::diffusion_planner::preprocess
{

// Internal functions declaration
namespace
{
using namespace autoware_perception_msgs::msg;

Eigen::MatrixXd process_segment_to_matrix(const LaneSegment & segment);
void transform_selected_rows(
  const Eigen::Matrix4d & transform_matrix, Eigen::MatrixXd & output_matrix, int64_t num_segments,
  int64_t row_idx, bool do_translation = true);
uint8_t identify_current_light_status(
  const int64_t turn_direction, const std::vector<TrafficLightElement> & traffic_light_elements);
}  // namespace

// LaneSegmentContext implementation
LaneSegmentContext::LaneSegmentContext(const std::shared_ptr<lanelet::LaneletMap> & lanelet_map_ptr)
: lanelet_map_ptr_(lanelet_map_ptr),
  lane_segments_(convert_to_lane_segments(lanelet_map_ptr, POINTS_PER_SEGMENT))
{
  if (lane_segments_.empty()) {
    throw std::runtime_error("No lane segments found in the map");
  }
}

std::pair<std::vector<float>, std::vector<float>> LaneSegmentContext::get_route_segments(
  const Eigen::Matrix4d & transform_matrix,
  const std::map<lanelet::Id, TrafficSignalStamped> & traffic_light_id_map,
  const LaneletRoute & route, const double center_x, const double center_y) const
{
  // Step 1: Select route segment indices
  const std::vector<int64_t> segment_indices =
    select_route_segment_indices(route, center_x, center_y, ROUTE_LANES_SHAPE[1]);

  // Step 2: Create tensor data from indices
  return create_tensor_data_from_indices(
    transform_matrix, traffic_light_id_map, segment_indices, ROUTE_LANES_SHAPE[1]);
}

std::pair<std::vector<float>, std::vector<float>> LaneSegmentContext::get_lane_segments(
  const Eigen::Matrix4d & transform_matrix,
  const std::map<lanelet::Id, TrafficSignalStamped> & traffic_light_id_map, const double center_x,
  const double center_y, const int64_t m) const
{
  if (m <= 0) {
    throw std::invalid_argument("m must be greater than 0.");
  }

  // Step 1: Select lane segment indices
  const std::vector<int64_t> segment_indices =
    select_lane_segment_indices(transform_matrix, center_x, center_y, LANES_SHAPE[1]);

  // Step 2: Create tensor data from indices
  return create_tensor_data_from_indices(
    transform_matrix, traffic_light_id_map, segment_indices, LANES_SHAPE[1]);
}

void LaneSegmentContext::add_traffic_light_one_hot_encoding_to_segment(
  const std::map<lanelet::Id, TrafficSignalStamped> & traffic_light_id_map,
  Eigen::MatrixXd & segment_matrix, const autoware::diffusion_planner::LaneSegment & lane_segment,
  const int64_t col_counter) const
{
  const Eigen::Vector<double, TRAFFIC_LIGHT_ONE_HOT_DIM> traffic_light_one_hot_encoding = [&]() {
    Eigen::Vector<double, TRAFFIC_LIGHT_ONE_HOT_DIM> encoding =
      Eigen::Vector<double, TRAFFIC_LIGHT_ONE_HOT_DIM>::Zero();
    if (lane_segment.traffic_light_id == LaneSegment::TRAFFIC_LIGHT_ID_NONE) {
      encoding[TRAFFIC_LIGHT_NO_TRAFFIC_LIGHT - TRAFFIC_LIGHT] = 1.0;
      return encoding;
    }

    const auto traffic_light_stamped_info_itr =
      traffic_light_id_map.find(lane_segment.traffic_light_id);
    if (traffic_light_stamped_info_itr == traffic_light_id_map.end()) {
      encoding[TRAFFIC_LIGHT_WHITE - TRAFFIC_LIGHT] = 1.0;
      return encoding;
    }

    const auto & signal = traffic_light_stamped_info_itr->second.signal;
    const uint8_t traffic_color =
      identify_current_light_status(lane_segment.turn_direction, signal.elements);
    return Eigen::Vector<double, TRAFFIC_LIGHT_ONE_HOT_DIM>{
      traffic_color == TrafficLightElement::GREEN,    // 3
      traffic_color == TrafficLightElement::AMBER,    // 2
      traffic_color == TrafficLightElement::RED,      // 1
      traffic_color == TrafficLightElement::UNKNOWN,  // 0
      traffic_color == TrafficLightElement::WHITE     // 4
    };
  }();

  Eigen::MatrixXd one_hot_encoding_matrix =
    traffic_light_one_hot_encoding.replicate(1, POINTS_PER_SEGMENT);
  segment_matrix.block<TRAFFIC_LIGHT_ONE_HOT_DIM, POINTS_PER_SEGMENT>(
    TRAFFIC_LIGHT, col_counter * POINTS_PER_SEGMENT) =
    one_hot_encoding_matrix.block<TRAFFIC_LIGHT_ONE_HOT_DIM, POINTS_PER_SEGMENT>(0, 0);
}

void LaneSegmentContext::apply_transforms(
  const Eigen::Matrix4d & transform_matrix, Eigen::MatrixXd & output_matrix,
  int64_t num_segments) const
{
  // transform the x and y coordinates
  transform_selected_rows(transform_matrix, output_matrix, num_segments, X);
  // the dx and dy coordinates do not require translation
  transform_selected_rows(transform_matrix, output_matrix, num_segments, dX, false);
  transform_selected_rows(transform_matrix, output_matrix, num_segments, LB_X);
  transform_selected_rows(transform_matrix, output_matrix, num_segments, RB_X);

  // subtract center from boundaries
  output_matrix.row(LB_X) = output_matrix.row(LB_X) - output_matrix.row(X);
  output_matrix.row(LB_Y) = output_matrix.row(LB_Y) - output_matrix.row(Y);
  output_matrix.row(RB_X) = output_matrix.row(RB_X) - output_matrix.row(X);
  output_matrix.row(RB_Y) = output_matrix.row(RB_Y) - output_matrix.row(Y);
}

std::vector<ColWithDistance> LaneSegmentContext::compute_distances(
  const Eigen::Matrix4d & transform_matrix, const float center_x, const float center_y) const
{
  auto compute_squared_distance = [](double x, double y, const Eigen::Matrix4d & transform_matrix) {
    Eigen::Vector4d p(x, y, 0.0, 1.0);
    Eigen::Vector4d p_transformed = transform_matrix * p;
    return p_transformed.head<2>().squaredNorm();
  };

  auto is_inside = [&](const double x, const double y) {
    using autoware::diffusion_planner::constants::LANE_MASK_RANGE_M;
    return (
      x > center_x - LANE_MASK_RANGE_M && x < center_x + LANE_MASK_RANGE_M &&
      y > center_y - LANE_MASK_RANGE_M && y < center_y + LANE_MASK_RANGE_M);
  };

  std::vector<ColWithDistance> distances;
  distances.reserve(lane_segments_.size());

  for (size_t i = 0; i < lane_segments_.size(); ++i) {
    const auto & segment = lane_segments_[i];
    const auto & centerlines = segment.polyline.waypoints();

    if (centerlines.size() != POINTS_PER_SEGMENT) {
      continue;
    }

    // Compute mean, first, and last points
    double mean_x = 0.0, mean_y = 0.0;
    for (const auto & point : centerlines) {
      mean_x += point.x();
      mean_y += point.y();
    }
    mean_x /= centerlines.size();
    mean_y /= centerlines.size();

    const double first_x = centerlines[0].x();
    const double first_y = centerlines[0].y();
    const double last_x = centerlines[POINTS_PER_SEGMENT - 1].x();
    const double last_y = centerlines[POINTS_PER_SEGMENT - 1].y();

    const bool inside =
      is_inside(mean_x, mean_y) || is_inside(first_x, first_y) || is_inside(last_x, last_y);

    const double distance_first = compute_squared_distance(first_x, first_y, transform_matrix);
    const double distance_last = compute_squared_distance(last_x, last_y, transform_matrix);
    const double distance_squared = std::min(distance_last, distance_first);

    distances.push_back({static_cast<int64_t>(i), distance_squared, inside});
  }

  return distances;
}

std::vector<int64_t> LaneSegmentContext::select_route_segment_indices(
  const LaneletRoute & route, const double center_x, const double center_y,
  const int64_t max_segments) const
{
  std::vector<int64_t> selected_indices;
  const lanelet::Lanelets route_lanelets = filter_route_lanelets(route, center_x, center_y);

  // Create a map from lanelet ID to segment index for quick lookup
  std::map<lanelet::Id, size_t> id_to_segment_idx;
  for (size_t i = 0; i < lane_segments_.size(); ++i) {
    id_to_segment_idx[lane_segments_[i].id] = i;
  }

  // Select route segment indices
  for (const lanelet::Lanelet & route_segment : route_lanelets) {
    if (selected_indices.size() >= static_cast<size_t>(max_segments)) {
      break;
    }

    const auto segment_idx_itr = id_to_segment_idx.find(route_segment.id());
    if (segment_idx_itr == id_to_segment_idx.end()) {
      continue;
    }

    selected_indices.push_back(static_cast<int64_t>(segment_idx_itr->second));
  }

  return selected_indices;
}

std::vector<int64_t> LaneSegmentContext::select_lane_segment_indices(
  const Eigen::Matrix4d & transform_matrix, const double center_x, const double center_y,
  const int64_t max_segments) const
{
  // Step 1: Compute distances
  std::vector<ColWithDistance> distances = compute_distances(transform_matrix, center_x, center_y);
  // Step 2: Sort indices by distance
  std::sort(distances.begin(), distances.end(), [](const auto & a, const auto & b) {
    return a.distance_squared < b.distance_squared;
  });

  // Step 3: Select indices that are inside the mask
  std::vector<int64_t> selected_indices;
  for (const auto & distance : distances) {
    if (!distance.inside) {
      continue;
    }
    selected_indices.push_back(distance.index);
    if (selected_indices.size() >= static_cast<size_t>(max_segments)) {
      break;
    }
  }

  return selected_indices;
}

std::pair<std::vector<float>, std::vector<float>>
LaneSegmentContext::create_tensor_data_from_indices(
  const Eigen::Matrix4d & transform_matrix,
  const std::map<lanelet::Id, TrafficSignalStamped> & traffic_light_id_map,
  const std::vector<int64_t> & segment_indices, const int64_t max_segments) const
{
  const auto total_points = max_segments * POINTS_PER_SEGMENT;
  Eigen::MatrixXd output_matrix(SEGMENT_POINT_DIM, total_points);
  output_matrix.setZero();

  std::vector<float> speed_limit_vector(max_segments, 0.0f);

  int64_t added_segments = 0;
  for (const int64_t segment_idx : segment_indices) {
    if (added_segments >= max_segments) {
      break;
    }

    const auto & lane_segment = lane_segments_[segment_idx];

    // Process segment to matrix
    Eigen::MatrixXd segment_matrix = process_segment_to_matrix(lane_segment);
    if (segment_matrix.rows() != POINTS_PER_SEGMENT || segment_matrix.cols() != SEGMENT_POINT_DIM) {
      continue;
    }

    // Transpose to match expected format (SEGMENT_POINT_DIM x POINTS_PER_SEGMENT)
    segment_matrix.transposeInPlace();

    output_matrix.block(
      0, added_segments * POINTS_PER_SEGMENT, SEGMENT_POINT_DIM, POINTS_PER_SEGMENT) =
      segment_matrix;

    add_traffic_light_one_hot_encoding_to_segment(
      traffic_light_id_map, output_matrix, lane_segment, added_segments);

    speed_limit_vector[added_segments] = lane_segment.speed_limit_mps.value_or(0.0f);
    ++added_segments;
  }

  // Transform the segments
  apply_transforms(transform_matrix, output_matrix, added_segments);

  // Convert to float vector
  const Eigen::MatrixXf output_matrix_f = output_matrix.cast<float>().eval();
  std::vector<float> tensor_data(
    output_matrix_f.data(), output_matrix_f.data() + output_matrix_f.size());

  return {tensor_data, speed_limit_vector};
}

lanelet::Lanelets LaneSegmentContext::filter_route_lanelets(
  const LaneletRoute & route, const double center_x, const double center_y) const
{
  lanelet::Lanelets route_lanelets;
  for (const LaneletSegment & route_segment : route.segments) {
    route_lanelets.push_back(
      lanelet_map_ptr_->laneletLayer.get(route_segment.preferred_primitive.id));
  }

  double closest_distance = std::numeric_limits<double>::max();
  size_t closest_index = 0;

  auto distance_to_lanelet =
    [](const double x, const double y, const lanelet::ConstLanelet & lanelet) {
      double distance = std::numeric_limits<double>::max();
      for (const lanelet::ConstPoint2d & point : lanelet.centerline()) {
        const double diff_x = point.x() - x;
        const double diff_y = point.y() - y;
        const double curr_distance = std::sqrt(diff_x * diff_x + diff_y * diff_y);
        distance = std::min(distance, curr_distance);
      }
      return distance;
    };

  const size_t num_lanelets = route_lanelets.size();
  for (size_t i = 0; i < num_lanelets; ++i) {
    const double distance = distance_to_lanelet(center_x, center_y, route_lanelets[i]);
    if (distance < closest_distance) {
      closest_distance = distance;
      closest_index = i;
    }
  }

  lanelet::Lanelets filtered_route;
  for (size_t i = closest_index; i < num_lanelets; ++i) {
    const double distance = distance_to_lanelet(center_x, center_y, route_lanelets[i]);
    if (distance > autoware::diffusion_planner::constants::LANE_MASK_RANGE_M) {
      break;
    }
    filtered_route.push_back(route_lanelets[i]);
  }
  return filtered_route;
}

// Internal functions implementation
namespace
{

void transform_selected_rows(
  const Eigen::Matrix4d & transform_matrix, Eigen::MatrixXd & output_matrix, int64_t num_segments,
  int64_t row_idx, bool do_translation)
{
  Eigen::MatrixXd xy_block(4, num_segments * POINTS_PER_SEGMENT);
  xy_block.setZero();
  xy_block.block(0, 0, 2, num_segments * POINTS_PER_SEGMENT) =
    output_matrix.block(row_idx, 0, 2, num_segments * POINTS_PER_SEGMENT);

  xy_block.row(3) = do_translation ? Eigen::MatrixXd::Ones(1, num_segments * POINTS_PER_SEGMENT)
                                   : Eigen::MatrixXd::Zero(1, num_segments * POINTS_PER_SEGMENT);

  Eigen::MatrixXd transformed_block = transform_matrix * xy_block;
  output_matrix.block(row_idx, 0, 2, num_segments * POINTS_PER_SEGMENT) =
    transformed_block.block(0, 0, 2, num_segments * POINTS_PER_SEGMENT);
}

uint8_t identify_current_light_status(
  const int64_t turn_direction, const std::vector<TrafficLightElement> & traffic_light_elements)
{
  // Filter out ineffective elements (color == 0 which is UNKNOWN)
  std::vector<TrafficLightElement> effective_elements;
  for (const auto & element : traffic_light_elements) {
    if (element.color != TrafficLightElement::UNKNOWN) {
      effective_elements.push_back(element);
    }
  }

  // If no effective elements, return UNKNOWN (0)
  if (effective_elements.empty()) {
    return TrafficLightElement::UNKNOWN;
  }

  // If only one effective element, return its color
  if (effective_elements.size() == 1) {
    return effective_elements[0].color;
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
  auto get_max_confidence_color = [](const std::vector<TrafficLightElement> & elements) {
    return std::max_element(
             elements.begin(), elements.end(),
             [](const TrafficLightElement & a, const TrafficLightElement & b) {
               return a.confidence < b.confidence;
             })
      ->color;
  };

  // First priority: Find elements with exactly matching direction
  std::vector<TrafficLightElement> matching_elements;
  for (const TrafficLightElement & element : effective_elements) {
    if (element.shape == target_shape) {
      matching_elements.push_back(element);
    }
  }
  if (!matching_elements.empty()) {
    return get_max_confidence_color(matching_elements);
  }

  // Second priority: Find circle elements
  std::vector<TrafficLightElement> circle_elements;
  for (const TrafficLightElement & element : effective_elements) {
    if (element.shape == TrafficLightElement::CIRCLE) {
      circle_elements.push_back(element);
    }
  }
  if (!circle_elements.empty()) {
    return get_max_confidence_color(circle_elements);
  }

  // If no matching direction or circle, return the element with highest confidence
  return get_max_confidence_color(effective_elements);
}

Eigen::MatrixXd process_segment_to_matrix(const LaneSegment & segment)
{
  if (
    segment.polyline.is_empty() || segment.left_boundaries.empty() ||
    segment.right_boundaries.empty()) {
    return {};
  }
  const auto & centerlines = segment.polyline.waypoints();
  const auto & left_boundaries = segment.left_boundaries.front().waypoints();
  const auto & right_boundaries = segment.right_boundaries.front().waypoints();

  if (
    centerlines.size() != POINTS_PER_SEGMENT || left_boundaries.size() != POINTS_PER_SEGMENT ||
    right_boundaries.size() != POINTS_PER_SEGMENT) {
    throw std::runtime_error(
      "Segment data size mismatch: centerlines, left boundaries, and right boundaries must have "
      "POINTS_PER_SEGMENT points");
  }

  Eigen::MatrixXd segment_data(POINTS_PER_SEGMENT, SEGMENT_POINT_DIM);
  segment_data.setZero();

  // Build each row
  for (int64_t i = 0; i < POINTS_PER_SEGMENT; ++i) {
    segment_data(i, X) = centerlines[i].x();
    segment_data(i, Y) = centerlines[i].y();
    segment_data(i, dX) =
      i < POINTS_PER_SEGMENT - 1 ? centerlines[i + 1].x() - centerlines[i].x() : 0.0f;
    segment_data(i, dY) =
      i < POINTS_PER_SEGMENT - 1 ? centerlines[i + 1].y() - centerlines[i].y() : 0.0f;
    segment_data(i, LB_X) = left_boundaries[i].x();
    segment_data(i, LB_Y) = left_boundaries[i].y();
    segment_data(i, RB_X) = right_boundaries[i].x();
    segment_data(i, RB_Y) = right_boundaries[i].y();
  }

  return segment_data;
}

}  // namespace

}  // namespace autoware::diffusion_planner::preprocess
