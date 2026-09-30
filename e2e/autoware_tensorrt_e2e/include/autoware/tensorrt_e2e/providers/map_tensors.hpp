// Copyright 2026 TIER IV, Inc.
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

#ifndef AUTOWARE__TENSORRT_E2E__PROVIDERS__MAP_TENSORS_HPP_
#define AUTOWARE__TENSORRT_E2E__PROVIDERS__MAP_TENSORS_HPP_

#include "autoware/tensorrt_e2e/types.hpp"

#include <autoware/diffusion_planner/preprocessing/lane_segments.hpp>

#include <autoware_planning_msgs/msg/lanelet_route.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <map>
#include <tuple>
#include <utility>
#include <vector>

/**
 * The map tensors as training builds them: the crop, the slot order and the route's first
 * lanelet are decided at the cloud pose (`EgoFrame::sensor_to_map`), and only the points
 * written into the tensors are expressed in the planning frame (`EgoFrame::map_to_ego`).
 * Under `cloud_stamp` the two frames are one, and every function here reduces to the plain
 * diffusion-planner call.
 *
 * Only the shared package's existing API is used. Lanes split selection from writing, so
 * they are written straight into the planning frame. Polygons and line strings sort and
 * write in one call, so they are built at the cloud pose and their present points are then
 * moved into the planning frame by the planar transform between the two poses -- the same
 * operation training applies (OnePlanner projects/resworld/latency.py, reexpress_map).
 */
namespace autoware::tensorrt_e2e
{

namespace map_tensors
{

using autoware::diffusion_planner::preprocess::LaneSegmentContext;
using autoware::diffusion_planner::preprocess::TrafficSignalStamped;

//! Segment table indices behind the `lanes` slots, and the tensor data itself.
struct Lanes
{
  std::vector<int64_t> indices;
  std::vector<float> data;
  std::vector<float> speed_limit;
};

inline double center_x(const EgoFrame & ego) { return ego.sensor_to_map(0, 3); }
inline double center_y(const EgoFrame & ego) { return ego.sensor_to_map(1, 3); }

inline Lanes build_lanes(
  const LaneSegmentContext & context, const EgoFrame & ego,
  const std::map<lanelet::Id, TrafficSignalStamped> & traffic_lights, const int64_t num_segments)
{
  Lanes lanes;
  lanes.indices = context.select_lane_segment_indices(
    ego.map_to_sensor, static_cast<float>(center_x(ego)), static_cast<float>(center_y(ego)),
    num_segments);
  std::tie(lanes.data, lanes.speed_limit) = context.create_tensor_data_from_indices(
    ego.map_to_ego, traffic_lights, lanes.indices, num_segments);
  return lanes;
}

//! The route's lane slots; the same selection at the cloud pose (`center_z` included).
inline Lanes build_route_lanes(
  const LaneSegmentContext & context, const EgoFrame & ego,
  const autoware_planning_msgs::msg::LaneletRoute & route,
  const std::map<lanelet::Id, TrafficSignalStamped> & traffic_lights, const int64_t num_segments)
{
  Lanes lanes;
  lanes.indices = context.select_route_segment_indices(
    route, center_x(ego), center_y(ego), ego.sensor_to_map(2, 3), num_segments);
  std::tie(lanes.data, lanes.speed_limit) = context.create_tensor_data_from_indices(
    ego.map_to_ego, traffic_lights, lanes.indices, num_segments);
  return lanes;
}

//! The cloud frame's planar pose in the planning frame: cos, sin of its yaw and its origin.
struct PlanarTransform
{
  double cos_yaw;
  double sin_yaw;
  double x;
  double y;
};

inline PlanarTransform cloud_to_planning(const EgoFrame & ego)
{
  const Eigen::Matrix4d transform = ego.map_to_ego * ego.sensor_to_map;
  const double yaw = std::atan2(transform(1, 0), transform(0, 0));
  return {std::cos(yaw), std::sin(yaw), transform(0, 3), transform(1, 3)};
}

//! Move each present point of a `[elements, points, point_dim]` tensor whose first two columns
//! are x, y. A point is present when any of its values is non-zero, as in training; padding
//! stays exactly zero and the type columns are untouched.
inline void reexpress_points(
  std::vector<float> & data, const int64_t point_dim, const PlanarTransform & transform)
{
  const auto dim = static_cast<size_t>(point_dim);
  for (size_t base = 0; base + dim <= data.size(); base += dim) {
    const auto first = data.begin() + static_cast<std::ptrdiff_t>(base);
    if (std::all_of(first, first + static_cast<std::ptrdiff_t>(dim), [](const float v) {
          return v == 0.0f;
        })) {
      continue;
    }
    const double x = data[base];
    const double y = data[base + 1];
    data[base] = static_cast<float>(transform.cos_yaw * x - transform.sin_yaw * y + transform.x);
    data[base + 1] = static_cast<float>(transform.sin_yaw * x + transform.cos_yaw * y + transform.y);
  }
}

//! A cloud-frame point tensor, moved into the planning frame when the two poses differ.
inline std::vector<float> in_planning_frame(
  std::vector<float> data, const EgoFrame & ego, const int64_t point_dim)
{
  if (ego.sensor_to_map != ego.ego_to_map) {
    reexpress_points(data, point_dim, cloud_to_planning(ego));
  }
  return data;
}

inline std::vector<float> build_polygons(
  const LaneSegmentContext & context, const EgoFrame & ego, const int64_t num_elements,
  const int64_t num_types)
{
  return in_planning_frame(
    context.create_polygon_tensor(
      ego.map_to_sensor, static_cast<float>(center_x(ego)), static_cast<float>(center_y(ego)),
      num_elements, num_types),
    ego, 2 + num_types);
}

inline std::vector<float> build_line_strings(
  const LaneSegmentContext & context, const EgoFrame & ego, const int64_t num_elements)
{
  return in_planning_frame(
    context.create_line_string_tensor(
      ego.map_to_sensor, static_cast<float>(center_x(ego)), static_cast<float>(center_y(ego)),
      num_elements),
    ego, 2 + autoware::diffusion_planner::LINE_STRING_TYPE_NUM);
}

}  // namespace map_tensors
}  // namespace autoware::tensorrt_e2e

#endif  // AUTOWARE__TENSORRT_E2E__PROVIDERS__MAP_TENSORS_HPP_
