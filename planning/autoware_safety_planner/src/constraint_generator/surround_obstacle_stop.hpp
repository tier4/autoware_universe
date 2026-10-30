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

#ifndef CONSTRAINT_GENERATOR__SURROUND_OBSTACLE_STOP_HPP_
#define CONSTRAINT_GENERATOR__SURROUND_OBSTACLE_STOP_HPP_

#include "constraint_generator_interface.hpp"

#include <autoware/obstacle_proximity_checker/obstacle_proximity_checker.hpp>
#include <rclcpp/time.hpp>

#include <memory>
#include <optional>
#include <string>

namespace autoware::safety_planner::experimental
{

class SurroundObstacleStopConstraintGenerator : public ConstraintGeneratorInterface
{
public:
  std::string get_name() const override { return "surround_obstacle_stop"; }
  ConstraintGeneratorOutput generate_constraints(const PlannerContext & context) override;

private:
  std::unique_ptr<obstacle_proximity_checker::ProximityChecker> proximity_checker_;

  bool is_stop_active_{false};
  std::optional<rclcpp::Time> last_obstacle_found_time_;

  void reset();
};

}  // namespace autoware::safety_planner::experimental

#endif  // CONSTRAINT_GENERATOR__SURROUND_OBSTACLE_STOP_HPP_
