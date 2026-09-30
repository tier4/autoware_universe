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

inline std::vector<float> build_polygons(
  const LaneSegmentContext & context, const EgoFrame & ego, const int64_t num_elements,
  const int64_t num_types)
{
  return context.create_polygon_tensor(
    ego.map_to_sensor, static_cast<float>(center_x(ego)), static_cast<float>(center_y(ego)),
    num_elements, num_types, &ego.map_to_ego);
}

inline std::vector<float> build_line_strings(
  const LaneSegmentContext & context, const EgoFrame & ego, const int64_t num_elements)
{
  return context.create_line_string_tensor(
    ego.map_to_sensor, static_cast<float>(center_x(ego)), static_cast<float>(center_y(ego)),
    num_elements, &ego.map_to_ego);
}

}  // namespace map_tensors
}  // namespace autoware::tensorrt_e2e

#endif  // AUTOWARE__TENSORRT_E2E__PROVIDERS__MAP_TENSORS_HPP_
