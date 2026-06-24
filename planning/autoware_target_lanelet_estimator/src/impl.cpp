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

#include "autoware/target_lanelet_estimator/impl.hpp"

#include <autoware_utils_geometry/boost_geometry.hpp>
#include <tf2/utils.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <boost/geometry/algorithms/area.hpp>
#include <boost/geometry/algorithms/correct.hpp>
#include <boost/geometry/algorithms/covered_by.hpp>
#include <boost/geometry/algorithms/disjoint.hpp>
#include <boost/geometry/algorithms/intersection.hpp>

#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/Polygon.h>
#include <lanelet2_core/primitives/BoundingBox.h>
#include <lanelet2_routing/RoutingGraph.h>

#include <algorithm>
#include <cmath>
#include <optional>
#include <utility>
#include <vector>

namespace autoware::target_lanelet_estimator
{
namespace
{
constexpr double preferred_lanelet_initial_probability = 0.8;
constexpr double other_lanelet_initial_probability = 0.2;
constexpr double same_segment_lane_change_probability = 0.05;
constexpr double following_transition_weight = 0.8;
constexpr double non_following_transition_weight = 0.2;
constexpr double epsilon = 1.0e-9;

struct RouteLanelet
{
  lanelet::ConstLanelet lanelet;
  bool preferred{false};
};

struct RouteSegmentLanelets
{
  size_t index{0};
  std::vector<RouteLanelet> lanelets;
};

struct UpdateScope
{
  std::optional<size_t> current_segment_index;
  std::optional<size_t> next_segment_index;
};

std::vector<RouteSegmentLanelets> extract_route_segments(
  const LaneletRoute & route, const lanelet::LaneletMapConstPtr & lanelet_map)
{
  std::vector<RouteSegmentLanelets> route_segments;
  route_segments.reserve(route.segments.size());
  for (size_t segment_index = 0; segment_index < route.segments.size(); ++segment_index) {
    const auto & segment = route.segments.at(segment_index);
    RouteSegmentLanelets route_segment;
    route_segment.index = segment_index;
    route_segment.lanelets.reserve(segment.primitives.size());
    for (const auto & primitive : segment.primitives) {
      route_segment.lanelets.push_back(
        {lanelet_map->laneletLayer.get(primitive.id),
         primitive.id == segment.preferred_primitive.id});
    }
    route_segments.push_back(std::move(route_segment));
  }
  return route_segments;
}

double initial_probability(const RouteLanelet & lanelet)
{
  return lanelet.preferred ? preferred_lanelet_initial_probability
                           : other_lanelet_initial_probability;
}

double previous_probability(
  const RouteLanelet & lanelet, const LaneletProbabilityMap & previous_posteriors)
{
  const auto it = previous_posteriors.find(lanelet.lanelet.id());
  if (it == previous_posteriors.end() || !std::isfinite(it->second)) {
    return initial_probability(lanelet);
  }
  return std::clamp(it->second, 0.0, 1.0);
}

double previous_probability_sum(
  const RouteSegmentLanelets & segment, const LaneletProbabilityMap & previous_posteriors)
{
  double sum = 0.0;
  for (const auto & lanelet : segment.lanelets) {
    sum += previous_probability(lanelet, previous_posteriors);
  }
  return sum;
}

double initial_probability_sum(const RouteSegmentLanelets & segment)
{
  double sum = 0.0;
  for (const auto & lanelet : segment.lanelets) {
    sum += initial_probability(lanelet);
  }
  return sum;
}

double normalized_previous_probability(
  const RouteLanelet & lanelet, const RouteSegmentLanelets & segment,
  const LaneletProbabilityMap & previous_posteriors)
{
  const double sum = previous_probability_sum(segment, previous_posteriors);
  if (sum > epsilon) {
    return previous_probability(lanelet, previous_posteriors) / sum;
  }

  const double initial_sum = initial_probability_sum(segment);
  if (initial_sum > epsilon) {
    return initial_probability(lanelet) / initial_sum;
  }
  return segment.lanelets.empty() ? 0.0 : 1.0 / static_cast<double>(segment.lanelets.size());
}

std::vector<lanelet::BasicPolygon2d> compute_trajectory_footprints(
  const Trajectory & trajectory, const VehicleInfo & vehicle_info)
{
  const auto base_footprint = vehicle_info.createFootprint();

  std::vector<lanelet::BasicPolygon2d> footprints;
  footprints.reserve(trajectory.points.size());
  for (const auto & point : trajectory.points) {
    const auto & pose = point.pose;
    const double yaw = tf2::getYaw(pose.orientation);
    const double cos_yaw = std::cos(yaw);
    const double sin_yaw = std::sin(yaw);

    lanelet::BasicPolygon2d footprint;
    footprint.reserve(base_footprint.size());
    for (const auto & p : base_footprint) {
      footprint.emplace_back(
        pose.position.x + cos_yaw * p.x() - sin_yaw * p.y(),
        pose.position.y + sin_yaw * p.x() + cos_yaw * p.y());
    }
    boost::geometry::correct(footprint);  // close/orient so area and intersection are valid
    footprints.push_back(std::move(footprint));
  }
  return footprints;
}

// Likelihood that the ego intends to drive on a lanelet: the maximum over trajectory points of
// (overlap area between the footprint and the lanelet) / (footprint area), in [0, 1].
double lanelet_likelihood(
  const lanelet::ConstLanelet & lanelet, const std::vector<lanelet::BasicPolygon2d> & footprints,
  double footprint_area)
{
  if (footprint_area <= 0.0) {
    return 0.0;
  }
  lanelet::BasicPolygon2d lanelet_polygon = lanelet.polygon2d().basicPolygon();
  boost::geometry::correct(lanelet_polygon);

  double likelihood = 0.0;
  for (const auto & footprint : footprints) {
    std::vector<lanelet::BasicPolygon2d> overlap;
    boost::geometry::intersection(footprint, lanelet_polygon, overlap);
    double overlap_area = 0.0;
    for (const auto & polygon : overlap) {
      overlap_area += std::abs(boost::geometry::area(polygon));
    }
    likelihood = std::max(likelihood, std::clamp(overlap_area / footprint_area, 0.0, 1.0));
    if (likelihood >= 1.0) {
      return 1.0;
    }
  }
  return likelihood;
}

bool is_on_any_lanelet(
  const lanelet::BasicPolygon2d & footprint, const lanelet::LaneletMapConstPtr & lanelet_map)
{
  lanelet::BoundingBox2d bbox;
  boost::geometry::envelope(footprint, bbox);
  // Pad the search box so axis-aligned lanelets are not missed by the R-tree query. This only
  // widens the candidate set; the disjoint test below still uses the original footprint.
  constexpr double search_margin_m = 2.0;
  const lanelet::BasicPoint2d margin(search_margin_m, search_margin_m);
  const lanelet::BoundingBox2d search_box(bbox.min() - margin, bbox.max() + margin);

  for (const auto & lanelet : lanelet_map->laneletLayer.search(search_box)) {
    auto lanelet_polygon = lanelet.polygon2d().basicPolygon();
    boost::geometry::correct(lanelet_polygon);
    if (!boost::geometry::disjoint(footprint, lanelet_polygon)) {
      return true;
    }
  }
  return false;
}

bool point_on_lanelet(
  const geometry_msgs::msg::Point & point, const lanelet::ConstLanelet & lanelet)
{
  lanelet::BasicPolygon2d lanelet_polygon = lanelet.polygon2d().basicPolygon();
  boost::geometry::correct(lanelet_polygon);
  return boost::geometry::covered_by(lanelet::BasicPoint2d{point.x, point.y}, lanelet_polygon);
}

std::optional<size_t> find_segment_containing_point(
  const std::vector<RouteSegmentLanelets> & route_segments, const geometry_msgs::msg::Point & point)
{
  for (const auto & segment : route_segments) {
    for (const auto & lanelet : segment.lanelets) {
      if (point_on_lanelet(point, lanelet.lanelet)) {
        return segment.index;
      }
    }
  }
  return std::nullopt;
}

bool trajectory_reaches_segment(const Trajectory & trajectory, const RouteSegmentLanelets & segment)
{
  for (const auto & point : trajectory.points) {
    for (const auto & lanelet : segment.lanelets) {
      if (point_on_lanelet(point.pose.position, lanelet.lanelet)) {
        return true;
      }
    }
  }
  return false;
}

UpdateScope determine_update_scope(
  const LaneletRoute & route, const Trajectory & trajectory,
  const std::vector<RouteSegmentLanelets> & route_segments)
{
  UpdateScope scope;
  for (const auto & point : trajectory.points) {
    scope.current_segment_index =
      find_segment_containing_point(route_segments, point.pose.position);
    if (scope.current_segment_index) {
      break;
    }
  }

  if (!scope.current_segment_index) {
    return scope;
  }

  const size_t next_segment_index = *scope.current_segment_index + 1;
  if (
    next_segment_index < route.segments.size() &&
    trajectory_reaches_segment(trajectory, route_segments.at(next_segment_index))) {
    scope.next_segment_index = next_segment_index;
  }
  return scope;
}

bool follows(
  const RouteLanelet & from, const RouteLanelet & to,
  const lanelet::routing::RoutingGraphConstPtr & routing_graph)
{
  if (!routing_graph) {
    return false;
  }

  const auto relation = routing_graph->routingRelation(from.lanelet, to.lanelet, false);
  return relation && *relation == lanelet::routing::RelationType::Successor;
}

size_t count_following_lanelets(
  const RouteLanelet & from, const RouteSegmentLanelets & to_segment,
  const lanelet::routing::RoutingGraphConstPtr & routing_graph)
{
  return static_cast<size_t>(std::count_if(
    to_segment.lanelets.begin(), to_segment.lanelets.end(),
    [&](const auto & to) { return follows(from, to, routing_graph); }));
}

double same_segment_transition_probability(
  const RouteLanelet & from, const RouteLanelet & to, const RouteSegmentLanelets & segment)
{
  if (from.lanelet.id() != to.lanelet.id()) {
    return same_segment_lane_change_probability;
  }

  const size_t non_self_lanelets = segment.lanelets.empty() ? 0 : segment.lanelets.size() - 1;
  return std::clamp(
    1.0 - same_segment_lane_change_probability * static_cast<double>(non_self_lanelets), 0.0, 1.0);
}

double previous_segment_transition_probability(
  const RouteLanelet & from, const RouteLanelet & to, const RouteSegmentLanelets & to_segment,
  const lanelet::routing::RoutingGraphConstPtr & routing_graph)
{
  const size_t num_following = count_following_lanelets(from, to_segment, routing_graph);
  const size_t num_non_following = to_segment.lanelets.size() - num_following;
  const double denominator =
    following_transition_weight * static_cast<double>(num_following) +
    non_following_transition_weight * static_cast<double>(num_non_following);
  if (denominator <= epsilon) {
    return 0.0;
  }

  return (follows(from, to, routing_graph) ? following_transition_weight
                                           : non_following_transition_weight) /
         denominator;
}

double same_segment_prior(
  const RouteLanelet & target, const RouteSegmentLanelets & segment,
  const LaneletProbabilityMap & previous_posteriors)
{
  double prior = 0.0;
  for (const auto & from : segment.lanelets) {
    prior += same_segment_transition_probability(from, target, segment) *
             normalized_previous_probability(from, segment, previous_posteriors);
  }
  return std::clamp(prior, 0.0, 1.0);
}

double previous_segment_prior(
  const RouteLanelet & target, const RouteSegmentLanelets & previous_segment,
  const RouteSegmentLanelets & current_segment, const LaneletProbabilityMap & previous_posteriors,
  const lanelet::routing::RoutingGraphConstPtr & routing_graph)
{
  double prior = 0.0;
  for (const auto & from : previous_segment.lanelets) {
    prior += previous_segment_transition_probability(from, target, current_segment, routing_graph) *
             normalized_previous_probability(from, previous_segment, previous_posteriors);
  }
  return std::clamp(prior, 0.0, 1.0);
}

double posterior_probability(double prior, double likelihood)
{
  const double clamped_prior = std::clamp(prior, 0.0, 1.0);
  const double clamped_likelihood = std::clamp(likelihood, 0.0, 1.0);
  const double numerator = clamped_prior * clamped_likelihood;
  const double denominator = numerator + (1.0 - clamped_prior) * (1.0 - clamped_likelihood);
  if (denominator <= epsilon) {
    return 0.0;
  }
  return std::clamp(numerator / denominator, 0.0, 1.0);
}
}  // namespace

LaneletProbabilityMap initialize_lanelet_probabilities(const LaneletRoute & route)
{
  LaneletProbabilityMap probabilities;
  for (const auto & segment : route.segments) {
    for (const auto & primitive : segment.primitives) {
      const double probability = primitive.id == segment.preferred_primitive.id
                                   ? preferred_lanelet_initial_probability
                                   : other_lanelet_initial_probability;
      const auto [it, inserted] = probabilities.emplace(primitive.id, probability);
      if (!inserted) {
        it->second = std::max(it->second, probability);
      }
    }
  }
  return probabilities;
}

TargetLaneletsResult get_target_lanelets(
  const LaneletRoute & route, const Trajectory & trajectory,
  const lanelet::LaneletMapConstPtr & lanelet_map, const VehicleInfo & vehicle_info,
  const LaneletProbabilityMap & previous_posteriors,
  const lanelet::routing::RoutingGraphConstPtr & routing_graph)
{
  const auto route_segments = extract_route_segments(route, lanelet_map);
  const auto update_scope = determine_update_scope(route, trajectory, route_segments);
  const auto footprints = compute_trajectory_footprints(trajectory, vehicle_info);
  const double footprint_area =
    footprints.empty() ? 0.0 : std::abs(boost::geometry::area(footprints.front()));

  TargetLaneletsResult result;
  for (const auto & segment : route_segments) {
    for (const auto & route_lanelet : segment.lanelets) {
      const double likelihood =
        lanelet_likelihood(route_lanelet.lanelet, footprints, footprint_area);

      double prior = previous_probability(route_lanelet, previous_posteriors);
      double posterior = prior;
      bool updated = false;
      if (update_scope.current_segment_index == segment.index) {
        prior = same_segment_prior(route_lanelet, segment, previous_posteriors);
        posterior = posterior_probability(prior, likelihood);
        updated = true;
      } else if (
        update_scope.next_segment_index == segment.index && update_scope.current_segment_index) {
        const auto & previous_segment = route_segments.at(*update_scope.current_segment_index);
        prior = previous_segment_prior(
          route_lanelet, previous_segment, segment, previous_posteriors, routing_graph);
        posterior = posterior_probability(prior, likelihood);
        updated = true;
      }

      result.lanelets.push_back(
        {route_lanelet.lanelet.id(), posterior, prior, likelihood, updated});
    }
  }

  // out_of_lanelet: a footprint overlaps no lanelet at all
  for (const auto & footprint : footprints) {
    if (!is_on_any_lanelet(footprint, lanelet_map)) {
      result.out_of_lanelet = true;
      break;
    }
  }
  return result;
}

TargetLaneletsResult get_target_lanelets(
  const LaneletRoute & route, const Trajectory & trajectory,
  const lanelet::LaneletMapConstPtr & lanelet_map, const VehicleInfo & vehicle_info)
{
  return get_target_lanelets(
    route, trajectory, lanelet_map, vehicle_info, initialize_lanelet_probabilities(route), nullptr);
}

}  // namespace autoware::target_lanelet_estimator
