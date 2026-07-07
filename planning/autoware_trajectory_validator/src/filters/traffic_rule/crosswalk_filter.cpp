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

#include "autoware/trajectory_validator/filters/traffic_rule/crosswalk_filter.hpp"

#include <utility>
#include <vector>

namespace autoware::trajectory_validator::plugin::traffic_rule
{

CrosswalkFilter::CrosswalkFilter() : ValidatorInterface("crosswalk_filter")
{
}

void CrosswalkFilter::update_parameters(const validator::Params & params)
{
  params_ = params.crosswalk;
}

void CrosswalkFilter::set_vehicle_info(const VehicleInfo & vehicle_info)
{
  ValidatorInterface::set_vehicle_info(vehicle_info);
}

CrosswalkFilter::result_t CrosswalkFilter::is_feasible(
  const CandidateTrajectory & /*candidate_trajectory*/, const FilterContext & /*context*/)
{
  std::vector<MetricReport> metrics;
  return ValidationResult{true, std::move(metrics)};
}

}  // namespace autoware::trajectory_validator::plugin::traffic_rule

#include <pluginlib/class_list_macros.hpp>
namespace traffic_rule = autoware::trajectory_validator::plugin::traffic_rule;
PLUGINLIB_EXPORT_CLASS(
  traffic_rule::CrosswalkFilter, autoware::trajectory_validator::plugin::ValidatorInterface)
