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

#include <boost/geometry/algorithms/envelope.hpp>
#include <boost/geometry/algorithms/intersects.hpp>

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
using autoware_utils_geometry::Box2d;
using autoware_utils_geometry::Point2d;
using autoware_utils_geometry::Segment2d;

// One segment of a chain bound, as a ring of its 2 points, with its bounding box.
struct BoundSegment
{
  Box2d box;
  Polygon2d shape;
};

// The segments of the left and right bounds of the chain lanelets from `start_lanelet` to the end
// of the chain. The line that joins the ends of the two bounds is not a bound. A segment whose box
// does not overlap `footprints_box` cannot cross a footprint, so it is not built.
[[nodiscard]] std::vector<BoundSegment> collect_bound_segments(
  const LaneChain & chain, const lanelet::ConstLanelet & start_lanelet,
  const Box2d & footprints_box)
{
  std::vector<BoundSegment> segments;
  const auto start_it = std::find_if(
    chain.lanelets.begin(), chain.lanelets.end(),
    [&](const auto & lanelet) { return lanelet.id() == start_lanelet.id(); });
  for (auto lanelet_it = start_it; lanelet_it != chain.lanelets.end(); ++lanelet_it) {
    for (const auto & bound : {lanelet_it->leftBound2d(), lanelet_it->rightBound2d()}) {
      for (size_t k = 1; k < bound.size(); ++k) {
        const Point2d start{bound[k - 1].x(), bound[k - 1].y()};
        const Point2d end{bound[k].x(), bound[k].y()};
        const auto box = boost::geometry::return_envelope<Box2d>(Segment2d{start, end});
        if (!boost::geometry::intersects(footprints_box, box)) {
          continue;
        }
        Polygon2d shape;
        shape.outer().push_back(start);
        shape.outer().push_back(end);
        segments.push_back(BoundSegment{box, shape});
      }
    }
  }
  return segments;
}

// A bounding box test first, then the separating axis test of CCF, as in find_collision_timing.
[[nodiscard]] bool crosses_any_segment(
  const Box2d & footprint_box, const Polygon2d & footprint,
  const std::vector<BoundSegment> & segments)
{
  return std::any_of(segments.begin(), segments.end(), [&](const auto & segment) {
    return boost::geometry::intersects(footprint_box, segment.box) &&
           geometry::intersects_sat(footprint, segment.shape);
  });
}

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
  const LaneChain & chain, const lanelet::ConstLanelet & start_lanelet,
  const TrajectoryData & trajectory, const size_t window_end, const RoadFollowingParams & params)
{
  // Without this check, the walk finds no bound, and the bound test passes.
  if (!contains_lanelet(chain.lanelets, start_lanelet)) {
    return false;
  }
  if (window_end == 0U) {
    return true;  // The window has no sample to test.
  }

  // The window is the samples [0, window_end). IndexRange of TrajectoryData is inclusive.
  const size_t last = std::min(window_end, trajectory.size()) - 1U;
  const auto segments = collect_bound_segments(
    chain, start_lanelet, trajectory.get_or_compute_envelope(IndexRange{0U, last}));
  for (size_t i = 0; i <= last; ++i) {
    const IndexRange sample{i, i};
    if (crosses_any_segment(
          trajectory.get_or_compute_envelope(sample), trajectory.get_or_compute_convex(sample),
          segments)) {
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

RoadFollowingChecker::RoadFollowingChecker(
  std::vector<LaneChain> chains, lanelet::ConstLanelets route_lanelets)
: chains_(std::move(chains)), route_lanelets_(std::move(route_lanelets))
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
  lanelet::ConstLanelets route_lanelets;
  for (const auto & list : *lists) {
    auto chain = build_lane_chain(list);
    if (!chain) {
      return tl::make_unexpected(chain.error());
    }
    // The lists repeat a lanelet at a branch point, so each lanelet is kept once.
    for (const auto & lanelet : list) {
      if (!contains_lanelet(route_lanelets, lanelet)) {
        route_lanelets.push_back(lanelet);
      }
    }
    chains.push_back(std::move(*chain));
  }
  if (chains.empty()) {
    return tl::make_unexpected("the route gives no chain");
  }
  return RoadFollowingChecker(std::move(chains), std::move(route_lanelets));
}

const std::vector<LaneChain> & RoadFollowingChecker::chains() const
{
  return chains_;
}
}  // namespace autoware::trajectory_validator::plugin::safety::road_following
