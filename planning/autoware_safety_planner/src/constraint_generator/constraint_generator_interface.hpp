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

#ifndef CONSTRAINT_GENERATOR__CONSTRAINT_GENERATOR_INTERFACE_HPP_
#define CONSTRAINT_GENERATOR__CONSTRAINT_GENERATOR_INTERFACE_HPP_

#include "../constraint.hpp"
#include "../context.hpp"
#include "../type_alias.hpp"

#include <memory>
#include <string>
#include <vector>

namespace autoware::safety_planner
{

struct ConstraintGeneratorOutput
{
  std::vector<Constraint> constraints;
  std::optional<MarkerArray> debug_markers;
};

class ConstraintGeneratorInterface
{
public:
  ConstraintGeneratorInterface() = default;
  virtual ~ConstraintGeneratorInterface() = default;

  void on_initialize(
    const std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper, const Params & params)
  {
    time_keeper_ = time_keeper;
    params_ = params;
  }

  virtual std::string get_name() const = 0;
  virtual ConstraintGeneratorOutput generate_constraints(const PlannerContext & context) = 0;

protected:
  mutable std::shared_ptr<TimeKeeper> time_keeper_{nullptr};
  Params params_;
};

}  // namespace autoware::safety_planner

#endif  // CONSTRAINT_GENERATOR__CONSTRAINT_GENERATOR_INTERFACE_HPP_
