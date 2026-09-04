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

#ifndef AUTOWARE__TARGET_LANELET_ESTIMATOR__TRACKER_HPP_
#define AUTOWARE__TARGET_LANELET_ESTIMATOR__TRACKER_HPP_

#include "autoware/target_lanelet_estimator/impl.hpp"

#include <autoware_utils_geometry/boost_geometry.hpp>

#include <geometry_msgs/msg/pose.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_routing/Forward.h>

#include <vector>

namespace autoware::target_lanelet_estimator
{
// Recursive per-vehicle lanelet-probability estimate. One tracker holds the running state of one
// vehicle (the ego or a surrounding object); the node keeps a collection of them. It is agnostic to
// how the poses and footprint are collected, so the same class serves the ego and other vehicles.
class LaneletProbabilityTracker
{
public:
  // Advance the recursive Bayesian filter by one step from the vehicle's poses (trajectory or
  // predicted path) and its local-frame footprint. The previous posterior is carried internally.
  void update(
    const LaneletRoute & route, const std::vector<geometry_msgs::msg::Pose> & poses,
    const autoware_utils_geometry::LinearRing2d & base_footprint,
    const lanelet::LaneletMapConstPtr & lanelet_map,
    const lanelet::routing::RoutingGraphConstPtr & routing_graph, const Parameters & params)
  {
    // fully qualified: the member get_target_lanelets() below would otherwise shadow the free one
    result_ = autoware::target_lanelet_estimator::get_target_lanelets(
      route, poses, base_footprint, lanelet_map, get_posteriors(), routing_graph, params);
  }

  const TargetLaneletsResult & get_target_lanelets() const { return result_; }

  // Posterior probability of every route lanelet, in the form the next update needs as its prior.
  // Empty on a fresh tracker, which makes the first update fall back to the initial probabilities.
  LaneletProbabilityMap get_posteriors() const
  {
    LaneletProbabilityMap posteriors;
    posteriors.reserve(result_.lanelet_probabilities.size());
    for (const auto & lanelet : result_.lanelet_probabilities) {
      posteriors[lanelet.id] = lanelet.posterior;
    }
    return posteriors;
  }

private:
  TargetLaneletsResult result_;
};

}  // namespace autoware::target_lanelet_estimator

#endif  // AUTOWARE__TARGET_LANELET_ESTIMATOR__TRACKER_HPP_
