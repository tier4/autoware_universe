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
using autoware_perception_msgs::msg::TrafficLightElement;

Eigen::MatrixXd process_segments_to_matrix(
  const std::vector<LaneSegment> & lane_segments, ColLaneIDMaps & col_id_mapping);
Eigen::MatrixXd process_segment_to_matrix(const LaneSegment & segment);
void transform_selected_rows(
  const Eigen::Matrix4d & transform_matrix, Eigen::MatrixXd & output_matrix, int64_t num_segments,
  int64_t row_idx, bool do_translation = true);
uint8_t identify_current_light_status(
  const int64_t turn_direction, const std::vector<TrafficLightElement> & traffic_light_elements);
}  // namespace

// LaneSegmentContext implementation
LaneSegmentContext::LaneSegmentContext(const std::shared_ptr<lanelet::LaneletMap> & lanelet_map_ptr)
: lane_segments_(convert_to_lane_segments(lanelet_map_ptr, POINTS_PER_SEGMENT))
{
  if (lane_segments_.empty()) {
    throw std::runtime_error("No lane segments found in the map");
  }

  map_lane_segments_matrix_ = process_segments_to_matrix(lane_segments_, col_id_mapping_);
}

std::pair<std::vector<float>, std::vector<float>> LaneSegmentContext::get_route_segments(
  const Eigen::Matrix4d & transform_matrix,
  const std::map<lanelet::Id, TrafficSignalStamped> & traffic_light_id_map,
  const lanelet::ConstLanelets & current_lanes) const
{
  const auto total_route_points = ROUTE_LANES_SHAPE[1] * POINTS_PER_SEGMENT;
  Eigen::MatrixXd full_route_segment_matrix(SEGMENT_POINT_DIM, total_route_points);
  full_route_segment_matrix.setZero();
  int64_t added_route_segments = 0;

  std::vector<float> speed_limit_vector(ROUTE_LANES_SHAPE[1]);

  // Add traffic light one-hot encoding to the route segments
  for (const auto & route_segment : current_lanes) {
    if (added_route_segments >= ROUTE_LANES_SHAPE[1]) {
      break;
    }
    auto route_segment_row_itr = col_id_mapping_.lane_id_to_matrix_col.find(route_segment.id());
    if (route_segment_row_itr == col_id_mapping_.lane_id_to_matrix_col.end()) {
      continue;
    }

    const auto row_idx = route_segment_row_itr->second;
    full_route_segment_matrix.block(
      0, added_route_segments * POINTS_PER_SEGMENT, SEGMENT_POINT_DIM, POINTS_PER_SEGMENT) =
      map_lane_segments_matrix_.block(0, row_idx, SEGMENT_POINT_DIM, POINTS_PER_SEGMENT);

    const int64_t seg_idx = row_idx / POINTS_PER_SEGMENT;
    const int64_t turn_direction = lane_segments_[seg_idx].turn_direction;
    add_traffic_light_one_hot_encoding_to_segment(
      traffic_light_id_map, full_route_segment_matrix, row_idx, added_route_segments,
      turn_direction);

    speed_limit_vector[added_route_segments] =
      lane_segments_[seg_idx].speed_limit_mps.value_or(0.0f);
    ++added_route_segments;
  }
  // Transform the route segments.
  apply_transforms(transform_matrix, full_route_segment_matrix, added_route_segments);
  return {
    {full_route_segment_matrix.data(),
     full_route_segment_matrix.data() + full_route_segment_matrix.size()},
    speed_limit_vector};
}

std::pair<std::vector<float>, std::vector<float>> LaneSegmentContext::get_lane_segments(
  const Eigen::Matrix4d & transform_matrix,
  const std::map<lanelet::Id, TrafficSignalStamped> & traffic_light_id_map, const float center_x,
  const float center_y, const int64_t m) const
{
  if (map_lane_segments_matrix_.rows() != SEGMENT_POINT_DIM || m <= 0) {
    throw std::invalid_argument(
      "Input matrix must have at least SEGMENT_POINT_DIM rows and m must be greater than 0.");
  }
  // Step 1: Compute distances
  std::vector<ColWithDistance> distances = compute_distances(transform_matrix, center_x, center_y);
  // Step 2: Sort indices by distance
  std::sort(distances.begin(), distances.end(), [](const auto & a, const auto & b) {
    return a.distance_squared < b.distance_squared;
  });
  // Step 3: Apply transformation to selected rows
  const Eigen::MatrixXd ego_centric_lane_segments =
    transform_points_and_add_traffic_info(transform_matrix, traffic_light_id_map, distances, m);

  // Extract lane tensor data
  const auto total_lane_points = LANES_SHAPE[1] * POINTS_PER_SEGMENT;
  Eigen::MatrixXd lane_matrix(SEGMENT_POINT_DIM, total_lane_points);
  lane_matrix.block(0, 0, SEGMENT_POINT_DIM, total_lane_points) =
    ego_centric_lane_segments.block(0, 0, SEGMENT_POINT_DIM, total_lane_points);
  const Eigen::MatrixXf lane_matrix_f = lane_matrix.cast<float>().eval();
  std::vector<float> lane_tensor_data(
    lane_matrix_f.data(), lane_matrix_f.data() + lane_matrix_f.size());

  // Extract lane speed tensor data
  const auto total_speed_points = LANES_SPEED_LIMIT_SHAPE[1];
  std::vector<float> lane_speed_vector(total_speed_points);
  for (int64_t i = 0; i < total_speed_points; ++i) {
    const int64_t row_idx = distances[i].index / POINTS_PER_SEGMENT;
    lane_speed_vector[i] = lane_segments_[row_idx].speed_limit_mps.value_or(0.0f);
  }

  return {lane_tensor_data, lane_speed_vector};
}

void LaneSegmentContext::add_traffic_light_one_hot_encoding_to_segment(
  const std::map<lanelet::Id, TrafficSignalStamped> & traffic_light_id_map,
  Eigen::MatrixXd & segment_matrix, const int64_t row_idx, const int64_t col_counter,
  const int64_t turn_direction) const
{
  const autoware::diffusion_planner::LaneSegment & lane_segment =
    lane_segments_[row_idx / POINTS_PER_SEGMENT];

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
    const uint8_t traffic_color = identify_current_light_status(turn_direction, signal.elements);
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
  const auto cols = map_lane_segments_matrix_.cols();
  if (cols % POINTS_PER_SEGMENT != 0) {
    throw std::runtime_error("input matrix cols are not divisible by POINTS_PER_SEGMENT");
  }

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
  distances.reserve(cols / POINTS_PER_SEGMENT);
  for (int64_t i = 0; i < cols; i += POINTS_PER_SEGMENT) {
    // Directly access input matrix as raw memory
    const double mean_x = map_lane_segments_matrix_.block(X, i, 1, POINTS_PER_SEGMENT).mean();
    const double mean_y = map_lane_segments_matrix_.block(Y, i, 1, POINTS_PER_SEGMENT).mean();
    const double first_x = map_lane_segments_matrix_(X, i);
    const double first_y = map_lane_segments_matrix_(Y, i);
    const double last_x = map_lane_segments_matrix_(X, i + POINTS_PER_SEGMENT - 1);
    const double last_y = map_lane_segments_matrix_(Y, i + POINTS_PER_SEGMENT - 1);
    const bool inside =
      is_inside(mean_x, mean_y) || is_inside(first_x, first_y) || is_inside(last_x, last_y);

    const double distance_first = compute_squared_distance(first_x, first_y, transform_matrix);
    const double distance_last = compute_squared_distance(last_x, last_y, transform_matrix);
    const double distance_squared = std::min(distance_last, distance_first);

    distances.push_back({static_cast<int64_t>(i), distance_squared, inside});
  }

  return distances;
}

Eigen::MatrixXd LaneSegmentContext::transform_points_and_add_traffic_info(
  const Eigen::Matrix4d & transform_matrix,
  const std::map<lanelet::Id, TrafficSignalStamped> & traffic_light_id_map,
  const std::vector<ColWithDistance> & distances, int64_t m) const
{
  if (
    map_lane_segments_matrix_.rows() != SEGMENT_POINT_DIM ||
    map_lane_segments_matrix_.cols() % POINTS_PER_SEGMENT != 0) {
    throw std::invalid_argument("input_matrix size mismatch");
  }

  const int64_t n_total_segments =
    static_cast<int64_t>(map_lane_segments_matrix_.cols() / POINTS_PER_SEGMENT);
  const int64_t num_segments = std::min(m, n_total_segments);

  Eigen::MatrixXd output_matrix(SEGMENT_POINT_DIM, m * POINTS_PER_SEGMENT);
  output_matrix.setZero();

  int64_t added_segments = 0;
  for (auto distance : distances) {
    if (!distance.inside) {
      continue;
    }
    const auto col_idx_in_original_map = distance.index;

    // get POINTS_PER_SEGMENT rows corresponding to a single segment
    output_matrix.block<SEGMENT_POINT_DIM, POINTS_PER_SEGMENT>(
      0, added_segments * POINTS_PER_SEGMENT) =
      map_lane_segments_matrix_.block<SEGMENT_POINT_DIM, POINTS_PER_SEGMENT>(
        0, col_idx_in_original_map);

    const int64_t turn_direction =
      lane_segments_[col_idx_in_original_map / POINTS_PER_SEGMENT].turn_direction;
    add_traffic_light_one_hot_encoding_to_segment(
      traffic_light_id_map, output_matrix, col_idx_in_original_map, added_segments, turn_direction);

    ++added_segments;
    if (added_segments >= num_segments) {
      break;
    }
  }

  apply_transforms(transform_matrix, output_matrix, added_segments);
  return output_matrix;
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

Eigen::MatrixXd process_segments_to_matrix(
  const std::vector<LaneSegment> & lane_segments, ColLaneIDMaps & col_id_mapping)
{
  if (lane_segments.empty()) {
    throw std::runtime_error("Empty lane segment data");
  }
  std::vector<Eigen::MatrixXd> all_segment_matrices;
  for (const auto & segment : lane_segments) {
    Eigen::MatrixXd segment_matrix = process_segment_to_matrix(segment);

    if (segment_matrix.rows() != POINTS_PER_SEGMENT) {
      throw std::runtime_error("Segment matrix rows not equal to POINTS_PER_SEGMENT");
    }
    all_segment_matrices.push_back(segment_matrix);
  }

  // Now allocate the full matrix
  const int64_t rows =
    static_cast<int64_t>(POINTS_PER_SEGMENT) * static_cast<int64_t>(lane_segments.size());
  const int64_t cols = all_segment_matrices[0].cols();
  Eigen::MatrixXd stacked_matrix(rows, cols);

  int64_t current_row = 0;
  for (int64_t i = 0; i < static_cast<int64_t>(lane_segments.size()); ++i) {
    const auto & mat = all_segment_matrices[i];
    stacked_matrix.middleRows(current_row, mat.rows()) = mat;
    const auto id = lane_segments[i].id;
    col_id_mapping.lane_id_to_matrix_col.emplace(id, current_row);
    col_id_mapping.matrix_col_to_lane_id.emplace(current_row, id);
    current_row += POINTS_PER_SEGMENT;
  }
  return stacked_matrix.transpose();
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

  auto encode = [](const int64_t line_type) {
    Eigen::Vector<double, LINE_TYPE_NUM> one_hot = Eigen::Vector<double, LINE_TYPE_NUM>::Zero();
    if (line_type >= 0 && line_type < LINE_TYPE_NUM) {
      one_hot[line_type] = 1.0;
    }
    return one_hot;
  };

  const Eigen::Vector<double, LINE_TYPE_NUM> left = encode(segment.left_line_type);
  const Eigen::Vector<double, LINE_TYPE_NUM> right = encode(segment.right_line_type);

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
    for (int64_t j = 0; j < LINE_TYPE_NUM; ++j) {
      segment_data(i, LINE_TYPE_LEFT_START + j) = left(j);
      segment_data(i, LINE_TYPE_RIGHT_START + j) = right(j);
    }
  }

  return segment_data;
}

}  // namespace

}  // namespace autoware::diffusion_planner::preprocess
