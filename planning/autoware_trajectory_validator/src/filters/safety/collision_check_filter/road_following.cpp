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

#include "trajectory_utils.hpp"

#include <autoware/lanelet2_utils/geometry.hpp>
#include <autoware/lanelet2_utils/intersection.hpp>
#include <autoware/lanelet2_utils/topology.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_utils_math/normalization.hpp>

#include <boost/geometry/algorithms/within.hpp>

#include <lanelet2_core/geometry/Polygon.h>
#include <tf2/utils.h>

#include <algorithm>
#include <cmath>
#include <memory>
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

size_t find_window_end(const TrajectoryData & trajectory, const RoadFollowingParams & params)
{
  // A TrajectoryData is never empty, so the first sample exists.
  const auto & times = trajectory.getTimes();
  const auto & poses = trajectory.getPoses();
  for (size_t i = 0; i < trajectory.size(); ++i) {
    const double arc_length = autoware::motion_utils::calcSignedArcLength(poses, 0, i);
    const double time = times.at(i) - times.front();
    if (arc_length > params.window_distance && time > params.window_time) {
      return i;
    }
  }
  return trajectory.size();
}

bool follows_chain(
  const LaneChain & chain, const TrajectoryData & trajectory, const size_t window_end,
  const RoadFollowingParams & params)
{
  if (window_end == 0U) {
    return true;  // The window has no sample to test.
  }

  // The window is the samples [0, window_end). IndexRange of TrajectoryData is inclusive.
  const size_t last = std::min(window_end, trajectory.size()) - 1U;
  // The left bounds in driving order, then the right bounds in reverse: a clockwise open ring, as
  // Lanelet2 registers BasicPolygon2d for Boost.Geometry.
  const auto chain_polygon = chain.shape.polygon2d().basicPolygon();
  for (size_t i = 0; i <= last; ++i) {
    // The footprint must be inside the chain polygon, as in the rear collision checker of the
    // planning validator, which tests a line against a combined lanelet polygon.
    if (!boost::geometry::within(
          trajectory.get_or_compute_convex(IndexRange{i, i}), chain_polygon)) {
      return false;
    }

    const auto & pose = trajectory.getPoses().at(i);
    const double lateral_distance =
      autoware::experimental::lanelet2_utils::get_lateral_distance_to_centerline(chain.shape, pose);
    if (std::abs(lateral_distance) > params.max_lateral_deviation) {
      return false;
    }

    const lanelet::BasicPoint3d position(pose.position.x, pose.position.y, pose.position.z);
    const double centerline_angle =
      autoware::experimental::lanelet2_utils::get_lanelet_angle(chain.shape, position);
    const double heading_deviation =
      autoware_utils_math::normalize_radian(tf2::getYaw(pose.orientation) - centerline_angle);
    if (std::abs(heading_deviation) > params.max_heading_deviation) {
      return false;
    }
  }
  return true;
}

RoadFollowingChecker::RoadFollowingChecker(std::vector<LaneChain> chains)
: chains_(std::move(chains))
{
}

tl::expected<RoadFollowingChecker, std::string> RoadFollowingChecker::create(
  const LaneletRoute::ConstSharedPtr & route,
  const std::shared_ptr<const lanelet::LaneletMap> & lanelet_map,
  const lanelet::routing::RoutingGraphConstPtr & routing_graph)
{
  if (!route) {
    return tl::make_unexpected("the route is missing");
  }
  if (!lanelet_map) {
    return tl::make_unexpected("the lanelet map is missing");
  }
  if (!routing_graph) {
    return tl::make_unexpected("the routing graph is missing");
  }

  const auto lists = build_lanelet_lists(*route, *lanelet_map, routing_graph);
  if (!lists) {
    return tl::make_unexpected(lists.error());
  }

  std::vector<LaneChain> chains;
  chains.reserve(lists->size());
  for (const auto & list : *lists) {
    auto chain = build_lane_chain(list);
    if (!chain) {
      return tl::make_unexpected(chain.error());
    }
    chains.push_back(std::move(*chain));
  }
  if (chains.empty()) {
    return tl::make_unexpected("the route gives no chain");
  }
  return RoadFollowingChecker(std::move(chains));
}

bool RoadFollowingChecker::is_road_following(
  const TrajectoryData & trajectory, const RoadFollowingParams & params) const
{
  const size_t window_end = find_window_end(trajectory, params);
  // A footprint outside every chain fails at the first sample of each chain.
  return std::any_of(chains_.begin(), chains_.end(), [&](const LaneChain & chain) {
    return follows_chain(chain, trajectory, window_end, params);
  });
}

const std::vector<LaneChain> & RoadFollowingChecker::chains() const
{
  return chains_;
}
}  // namespace autoware::trajectory_validator::plugin::safety::road_following
