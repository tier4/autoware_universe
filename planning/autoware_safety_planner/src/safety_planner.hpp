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

#ifndef SAFETY_PLANNER_HPP_
#define SAFETY_PLANNER_HPP_

#include "constraint_generator/constraint_generator_interface.hpp"
#include "context.hpp"
#include "trajectory_planner/trajectory_planner_interface.hpp"
#include "type_alias.hpp"

#include <tl/expected.hpp>

#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace autoware::safety_planner
{

struct SafetyPlannerResult
{
  std::optional<PlannedTrajectory> normal_trajectory;
  std::optional<PlannedTrajectory> cautious_trajectory;
  struct Debug
  {
    std::map<std::string, ConstraintGeneratorOutput> constraint_generator_outputs;
    PathPointTrajectory reference_path;
    TrajectoryPlannerDebug normal;
    TrajectoryPlannerDebug cautious;
  } debug;
};

class SafetyPlanner
{
public:
  SafetyPlanner(const Params & params, std::shared_ptr<TimeKeeper> time_keeper);

  tl::expected<SafetyPlannerResult, std::string> plan(const SafetyPlannerInput & input);

  std::vector<std::string> get_constraint_generator_plugin_names() const;

  std::string get_trajectory_planner_plugin_name() const;

private:
  Params params_;
  std::shared_ptr<TimeKeeper> time_keeper_;

  tl::expected<PathPointTrajectory, std::string> build_reference_path(
    const SafetyPlannerInput & input) const;

  /**
   ***********************************************************
   * @defgroup Constraint plugins
   * @{
   */

  void load_constraint_generator_plugins();

  std::map<std::string, ConstraintGeneratorOutput> calculate_constraints(
    const PlannerContext & context);

  using ConstraintGeneratorLoader = pluginlib::ClassLoader<ConstraintGeneratorInterface>;
  //! Must outlive the loaded instances: destroying the loader unloads them
  std::unique_ptr<ConstraintGeneratorLoader> constraint_generator_loader_;
  std::vector<std::shared_ptr<ConstraintGeneratorInterface>> constraint_generator_plugins_;

  /** @* */

private:
  /**
   ***********************************************************
   * @defgroup Trajectory planning
   * @{
   */

  void load_trajectory_planner_plugin();

  using TrajectoryPlannerLoader = pluginlib::ClassLoader<TrajectoryPlannerInterface>;
  std::unique_ptr<TrajectoryPlannerLoader> trajectory_planner_loader_;
  std::shared_ptr<TrajectoryPlannerInterface> trajectory_planner_;

  /** @* */
};

}  // namespace autoware::safety_planner

#endif  // SAFETY_PLANNER_HPP_
