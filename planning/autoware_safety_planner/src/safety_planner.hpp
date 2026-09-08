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

#ifndef AUTOWARE__SAFETY_PLANNER__SAFETY_PLANNER_HPP_
#define AUTOWARE__SAFETY_PLANNER__SAFETY_PLANNER_HPP_

// The pipeline itself. The ROS interface (subscriptions, publications, timer) belongs to
// SafetyPlannerNode; this class only takes a SafetyPlannerInput and returns the result, and holds
// neither a publisher nor a clock (message types, the TimeKeeper and pluginlib are fine). It
// generates the constraints and splits them by certainty; compiling them and driving the rough
// planner and the optimizer is the trajectory_planner plugin's job.

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
  std::optional<Trajectory> normal_trajectory;
  std::optional<Trajectory> cautious_trajectory;

  struct Debug
  {
    //! plugin name (get_name()) -> output; publishing debug_markers is the node's job
    std::map<std::string, ConstraintGeneratorOutput> constraint_generator_outputs;
    PathPointTrajectory reference_path;
    CompiledConstraints compiled_constraints;  //!< of the normal side; cautious is not visualized
    RoughPlanResult rough_plan_result;
    TrajectoryOptimizerResult trajectory_optimizer_result;
  } debug;
};

class SafetyPlanner
{
public:
  SafetyPlanner(const Params & params, std::shared_ptr<TimeKeeper> time_keeper);

  //! Returns the reason as a string in a cycle where the reference_path could not be built (no
  //! path along the route, the goal behind the ego)
  tl::expected<SafetyPlannerResult, std::string> plan(const SafetyPlannerInput & input);

  //! Names (get_name()) of the loaded plugins, which the node turns into debug marker publishers
  std::vector<std::string> get_constraint_generator_plugin_names() const;

  //! Name (get_name()) of the loaded trajectory planner plugin, empty when none is loaded
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

#endif  // AUTOWARE__SAFETY_PLANNER__SAFETY_PLANNER_HPP_
