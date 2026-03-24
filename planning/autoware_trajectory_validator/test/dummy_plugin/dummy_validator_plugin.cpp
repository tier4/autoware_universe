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
#include <utility>
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
      return tl::make_unexpected("Empty trajectory");
    }

    // Magic trigger: If we set velocity to -999.0 in our test, simulate a plugin rejection
    const bool is_ok = traj_points.front().longitudinal_velocity_mps == -999.0;

    std::vector<TrajectoryMetricStatus> metrics{
      autoware_internal_planning_msgs::build<TrajectoryMetricStatus>()
        .name("check_dummy_metric")
        .level(is_ok ? TrajectoryMetricStatus::ERROR : TrajectoryMetricStatus::OK)
        .score(is_ok ? 0.0 : 1.0)};

    return autoware_internal_planning_msgs::build<TrajectoryValidationStatus>()
      .name(get_name())
      .level(is_ok ? TrajectoryValidationStatus::ERROR : TrajectoryValidationStatus::OK)
      .metrics(std::move(metrics));
  }

  void update_parameters([[maybe_unused]] const validator::Params & params) final {}

  [[nodiscard]] bool is_debug_mode() const override { return false; }

private:
  DummyFilterParam params_;
};
}  // namespace autoware::trajectory_validator::plugin

PLUGINLIB_EXPORT_CLASS(
  autoware::trajectory_validator::plugin::DummyFilter,
  autoware::trajectory_validator::plugin::ValidatorInterface)
