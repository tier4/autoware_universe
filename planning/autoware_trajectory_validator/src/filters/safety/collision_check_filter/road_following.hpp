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

#ifndef FILTERS__SAFETY__COLLISION_CHECK_FILTER__ROAD_FOLLOWING_HPP_
#define FILTERS__SAFETY__COLLISION_CHECK_FILTER__ROAD_FOLLOWING_HPP_

#include "parameter.hpp"
#include "trajectory_utils.hpp"

#include <tl_expected/expected.hpp>

#include <autoware_planning_msgs/msg/lanelet_route.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_routing/Forward.h>

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

namespace autoware::trajectory_validator::plugin::safety::road_following
{
using autoware_planning_msgs::msg::LaneletRoute;

/**
 * @brief One lanelet list with its combined shape.
 */
struct LaneChain
{
  lanelet::ConstLanelets lanelets;  // joined by next only, in driving order
  lanelet::ConstLanelet shape;      // combine_lanelets_shape(lanelets)
};

/**
 * @brief Split the route lanelets into lists that "next" joins, in driving order.
 * @details A turn lanelet ends a list and is in no list. A branch point is the end of a lanelet
 * with two or more next lanelets. A branch is one route lanelet that starts at a branch point. At
 * a branch point, the list is copied for each branch. A lanelet with no previous list starts a
 * new list.
 * @return The lists, or the reason when a route lanelet id is not in the map.
 */
[[nodiscard]] tl::expected<std::vector<lanelet::ConstLanelets>, std::string> build_lanelet_lists(
  const LaneletRoute & route, const lanelet::LaneletMap & lanelet_map,
  const lanelet::routing::RoutingGraphConstPtr & routing_graph);

/**
 * @brief Combine one lanelet list into one chain with its combined shape.
 * @return The chain, or the reason when the list is empty.
 */
[[nodiscard]] tl::expected<LaneChain, std::string> build_lane_chain(
  const lanelet::ConstLanelets & lanelets);

/**
 * @brief Find the end of the window that the road following tests check.
 * @details The arc length and the time of each sample count from the first sample. The window
 * ends at the first sample with an arc length over `window_distance` and a time over
 * `window_time`. That sample is not in the window.
 * @return The index of that sample, or `trajectory.size()` when no sample is over both limits.
 */
[[nodiscard]] size_t find_window_end(
  const TrajectoryData & trajectory, const RoadFollowingParams & params);

/**
 * @brief Test if the window samples follow one chain.
 * @details The window is the samples `[0, window_end)` of `trajectory`. Each window sample must
 * pass three tests. Its footprint crosses no left or right bound of the chain lanelets from
 * `start_lanelet` to the end of the chain. Its lateral distance from the chain centerline is
 * `max_lateral_deviation` or less. Its heading differs from the centerline angle by
 * `max_heading_deviation` or less.
 * @return False when `start_lanelet` is not in the chain. Otherwise, true when every window sample
 * passes the three tests. An empty window gives true.
 */
[[nodiscard]] bool follows_chain(
  const LaneChain & chain, const lanelet::ConstLanelet & start_lanelet,
  const TrajectoryData & trajectory, size_t window_end, const RoadFollowingParams & params);

/**
 * @brief The road following test of one route. It builds the lane chains once for one route and
 * one map.
 */
class RoadFollowingChecker
{
public:
  /**
   * @brief Build the chains and the route lanelets once for one route and one map.
   * @return The checker, or the reason when an input is null, a step of the chain build fails, or
   * the route gives no chain.
   */
  [[nodiscard]] static tl::expected<RoadFollowingChecker, std::string> create(
    const LaneletRoute::ConstSharedPtr & route,
    const std::shared_ptr<const lanelet::LaneletMap> & lanelet_map,
    const lanelet::routing::RoutingGraphConstPtr & routing_graph);

  [[nodiscard]] const std::vector<LaneChain> & chains() const;  // for tests and debug markers

private:
  RoadFollowingChecker(std::vector<LaneChain> chains, lanelet::ConstLanelets route_lanelets);

  std::vector<LaneChain> chains_;
  lanelet::ConstLanelets route_lanelets_;  // the lanelets of all chains, each lanelet once
};
}  // namespace autoware::trajectory_validator::plugin::safety::road_following

#endif  // FILTERS__SAFETY__COLLISION_CHECK_FILTER__ROAD_FOLLOWING_HPP_
