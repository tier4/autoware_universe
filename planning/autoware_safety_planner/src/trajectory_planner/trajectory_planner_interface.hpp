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

#ifndef TRAJECTORY_PLANNER__TRAJECTORY_PLANNER_INTERFACE_HPP_
#define TRAJECTORY_PLANNER__TRAJECTORY_PLANNER_INTERFACE_HPP_

// Interface of the trajectory planner plugins. SafetyPlanner generates the constraints and splits
// them by certainty; compiling them and planning on them is left to the plugin. Input: the two
// constraint sets (normal / cautious); output: the two trajectories.

#include "../constraint.hpp"
#include "../context.hpp"
#include "../type_alias.hpp"

#include <autoware_utils_debug/time_keeper.hpp>

#include <map>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace autoware::safety_planner
{

struct TrajectoryPlannerInput
{
  const PlannerContext & context;
  const std::vector<Constraint> & normal_constraints;    //!< certainty = DEFINITE only
  const std::vector<Constraint> & cautious_constraints;  //!< DEFINITE + POSSIBLE
};

struct TrajectoryPlannerResult
{
  std::optional<Trajectory> normal_trajectory;
  std::optional<Trajectory> cautious_trajectory;

  //! Debug output of the plugin as ROS messages keyed by a name; the node publishes each under
  //! ~/debug/<name>. What is in it depends on the plugin
  struct Debug
  {
    std::map<std::string, Trajectory> trajectories;
    std::map<std::string, MarkerArray> markers;
  } debug;
};

class TrajectoryPlannerInterface
{
public:
  TrajectoryPlannerInterface() = default;
  virtual ~TrajectoryPlannerInterface() = default;

  //! Called once right after the plugin is loaded. A derived class loads its own inner plugins
  //! (the optimizer, ...) by overriding this.
  virtual void on_initialize(
    const std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper, const Params & params)
  {
    time_keeper_ = time_keeper;
    params_ = params;
  }

  virtual std::string get_name() const = 0;
  virtual TrajectoryPlannerResult plan(const TrajectoryPlannerInput & input) = 0;

protected:
  mutable std::shared_ptr<TimeKeeper> time_keeper_{nullptr};
  Params params_;
};

}  // namespace autoware::safety_planner

#endif  // TRAJECTORY_PLANNER__TRAJECTORY_PLANNER_INTERFACE_HPP_
