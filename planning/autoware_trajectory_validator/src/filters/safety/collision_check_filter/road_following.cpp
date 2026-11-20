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

#include "road_following.hpp"

#include <autoware/lanelet2_utils/geometry.hpp>
#include <autoware/lanelet2_utils/intersection.hpp>
#include <autoware/lanelet2_utils/topology.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <rclcpp/duration.hpp>

#include <algorithm>
#include <string>
#include <utility>
#include <vector>

namespace autoware::trajectory_validator::plugin::safety::road_following
{
namespace
{
[[nodiscard]] bool contains_lanelet(
  const lanelet::ConstLanelets & lanelets, const lanelet::ConstLanelet & lanelet)
{
  return std::any_of(lanelets.begin(), lanelets.end(), [&](const auto & candidate) {
    return candidate.id() == lanelet.id();
  });
}

[[nodiscard]] bool is_turn_lanelet(const lanelet::ConstLanelet & lanelet)
{
  return autoware::experimental::lanelet2_utils::is_left_direction(lanelet) ||
         autoware::experimental::lanelet2_utils::is_right_direction(lanelet);
}
}  // namespace

tl::expected<std::vector<lanelet::ConstLanelets>, std::string> build_lanelet_lists(
  const LaneletRoute & route, const lanelet::LaneletMap & lanelet_map,
  const lanelet::routing::RoutingGraphConstPtr & routing_graph)
{
  std::vector<lanelet::ConstLanelets> lists;
  for (const auto & segment : route.segments) {
    for (const auto & primitive : segment.primitives) {
      const auto lanelet_it = lanelet_map.laneletLayer.find(primitive.id);
      if (lanelet_it == lanelet_map.laneletLayer.end()) {
        return tl::make_unexpected(
          "route lanelet " + std::to_string(primitive.id) + " is not in the map");
      }
      const auto & lanelet = *lanelet_it;
      // A turn lanelet is in no list. The lists that reach it stop before it.
      if (is_turn_lanelet(lanelet)) {
        continue;
      }

      bool has_target_list = false;
      std::vector<lanelet::ConstLanelets> branch_copies;
      for (auto & list : lists) {
        const auto next_lanelets =
          autoware::experimental::lanelet2_utils::following_lanelets(list.back(), routing_graph);
        if (!contains_lanelet(next_lanelets, lanelet)) {
          continue;
        }
        has_target_list = true;
        if (next_lanelets.size() >= 2) {
          // A branch point: copy the list for this branch. The original list stops at the branch
          // point.
          auto branch_copy = list;
          branch_copy.push_back(lanelet);
          branch_copies.push_back(std::move(branch_copy));
        } else {
          list.push_back(lanelet);
        }
      }
      lists.insert(lists.end(), branch_copies.begin(), branch_copies.end());
      if (!has_target_list) {
        lists.push_back({lanelet});
      }
    }
  }
  return lists;
}

tl::expected<LaneChain, std::string> build_lane_chain(const lanelet::ConstLanelets & lanelets)
{
  const auto shape = autoware::experimental::lanelet2_utils::combine_lanelets_shape(lanelets);
  if (!shape) {
    return tl::make_unexpected("cannot combine an empty lanelet list");
  }
  return LaneChain{lanelets, *shape};
}

size_t find_window_end(
  const std::vector<TrajectoryPoint> & points, const RoadFollowingParams & params)
{
  if (points.empty()) {
    return 0U;
  }
  const double start_time = rclcpp::Duration(points.front().time_from_start).seconds();
  for (size_t i = 0; i < points.size(); ++i) {
    const double arc_length = autoware::motion_utils::calcSignedArcLength(points, 0, i);
    const double time = rclcpp::Duration(points.at(i).time_from_start).seconds() - start_time;
    if (arc_length > params.window_distance && time > params.window_time) {
      return i;
    }
  }
  return points.size();
}
}  // namespace autoware::trajectory_validator::plugin::safety::road_following
