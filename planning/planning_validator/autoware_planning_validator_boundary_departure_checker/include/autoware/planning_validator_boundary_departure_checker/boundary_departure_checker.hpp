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

#ifndef AUTOWARE__PLANNING_VALIDATOR_BOUNDARY_DEPARTURE_CHECKER__BOUNDARY_DEPARTURE_CHECKER_HPP_
#define AUTOWARE__PLANNING_VALIDATOR_BOUNDARY_DEPARTURE_CHECKER__BOUNDARY_DEPARTURE_CHECKER_HPP_

#include <autoware/boundary_departure_checker/parameters.hpp>
#include <autoware/boundary_departure_checker/uncrossable_boundary_checker.hpp>
#include <autoware/planning_validator/plugin_interface.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>

namespace autoware::planning_validator
{

class BoundaryDepartureChecker : public PluginInterface
{
public:
  void init(
    rclcpp::Node & node, const std::string & name,
    const std::shared_ptr<PlanningValidatorContext> & context) override;
  void validate() override;
  void setup_diag() override;
  std::string get_module_name() const override { return module_name_; }

private:
  void set_diag_status(
    DiagnosticStatusWrapper & stat, const bool & is_ok, const std::string & msg) const;

  bool enable_{true};
  InvalidTrajectoryHandlingType handling_type_{
    InvalidTrajectoryHandlingType::USE_PREVIOUS_RESULT_WITH_SOFT_STOP};
  autoware::boundary_departure_checker::UncrossableBoundaryDepartureParam params_;
  std::unique_ptr<autoware::boundary_departure_checker::UncrossableBoundaryChecker> checker_;
  bool is_valid_{true};
};

}  // namespace autoware::planning_validator

#endif  // AUTOWARE__PLANNING_VALIDATOR_BOUNDARY_DEPARTURE_CHECKER__BOUNDARY_DEPARTURE_CHECKER_HPP_
