// Copyright 2026 Autoware Foundation
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

#ifndef AUTOWARE__AVOIDANCE_TARGET_DETECTOR__TRAFFIC_RULES_HPP_
#define AUTOWARE__AVOIDANCE_TARGET_DETECTOR__TRAFFIC_RULES_HPP_

#include <lanelet2_core/Forward.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/GenericTrafficRules.h>

#include <memory>
#include <optional>
#include <string>

namespace autoware::avoidance_target_detector::traffic_rules
{

/** Location name registered for custom GoalPurposeRules traffic rules. */
inline constexpr const char * k_goal_purpose_location = "goal_purpose";

class GoalPurposeRules : public lanelet::traffic_rules::GenericTrafficRules
{
public:
  using lanelet::traffic_rules::GenericTrafficRules::GenericTrafficRules;

protected:
  lanelet::Optional<bool> canPass(
    const lanelet::RegulatoryElementConstPtrs & reg_elems) const override;

  lanelet::Optional<bool> canPass(
    const std::string & type, const std::string & location) const override;

  const lanelet::traffic_rules::CountrySpeedLimits & countrySpeedLimits() const override;

  lanelet::Optional<lanelet::traffic_rules::SpeedLimitInformation> speedLimit(
    const lanelet::RegulatoryElementConstPtrs & reg_elems) const override;

private:
  lanelet::traffic_rules::CountrySpeedLimits speed_limits_;
};

/** Build a routing graph using goal_purpose traffic rules. */
lanelet::routing::RoutingGraphConstPtr create_goal_purpose_routing_graph(
  const lanelet::LaneletMap & lanelet_map);

/** Get the left neighbor lanelet using goal_purpose routing rules. */
std::optional<lanelet::ConstLanelet> get_left_lanelet(
  const lanelet::routing::RoutingGraph & routing_graph, const lanelet::ConstLanelet & lanelet);

/** Get the right neighbor lanelet using goal_purpose routing rules. */
std::optional<lanelet::ConstLanelet> get_right_lanelet(
  const lanelet::routing::RoutingGraph & routing_graph, const lanelet::ConstLanelet & lanelet);

}  // namespace autoware::avoidance_target_detector::traffic_rules

#endif  // AUTOWARE__AVOIDANCE_TARGET_DETECTOR__TRAFFIC_RULES_HPP_
