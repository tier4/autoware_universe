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

#include "autoware/trajectory_validator/validator_interface.hpp"

#include <pluginlib/class_list_macros.hpp>

#include <string>
#include <vector>

namespace autoware::trajectory_validator::plugin
{
class DummyFilter : public ValidatorInterface
{
  struct DummyFilterParam
  {
    double dummy_param{0.0};
  };

public:
  DummyFilter() : ValidatorInterface("DummyFilter") {}

  result_t is_feasible(
    const TrajectoryPoints & traj_points, const FilterContext & /*context*/) final
  {
    if (traj_points.empty()) {
      std::vector<MetricReport> metrics{autoware_trajectory_validator::build<MetricReport>()
                                          .validator_name(get_name())
                                          .validator_category(category())
                                          .metric_name("empty_trajectory")
                                          .metric_value(0.0)
                                          .level(MetricReport::ERROR)};
      return ValidationResult{false, std::move(metrics)};
    }

    // Magic trigger: If we set velocity to -999.0 in our test, simulate a plugin rejection
    const auto is_feasible = traj_points.front().longitudinal_velocity_mps != -999.0;

    std::vector<MetricReport> metrics{
      autoware_trajectory_validator::build<MetricReport>()
        .validator_name(get_name())
        .validator_category(category())
        .metric_name("explicit_rejection")
        .metric_value(0.0)
        .level(is_feasible ? MetricReport::OK : MetricReport::ERROR)};
    return ValidationResult{is_feasible, std::move(metrics)};
  }

  void update_parameters([[maybe_unused]] const validator::Params & params) final {}

  [[nodiscard]] bool is_shadow_mode() const final { return false; }

private:
  DummyFilterParam params_;
};
}  // namespace autoware::trajectory_validator::plugin

PLUGINLIB_EXPORT_CLASS(
  autoware::trajectory_validator::plugin::DummyFilter,
  autoware::trajectory_validator::plugin::ValidatorInterface)
