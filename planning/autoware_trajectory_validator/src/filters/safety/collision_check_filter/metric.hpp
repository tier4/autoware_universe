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

#ifndef AUTOWARE__TRAJECTORY_VALIDATOR__FILTERS__SAFETY__COLLISION_CHECK_FILTER__METRIC_HPP_
#define AUTOWARE__TRAJECTORY_VALIDATOR__FILTERS__SAFETY__COLLISION_CHECK_FILTER__METRIC_HPP_

#include "parameter.hpp"
#include "types.hpp"

#include <autoware_trajectory_validator/msg/metric_report.hpp>

#include <string>
#include <vector>

namespace autoware::trajectory_validator::plugin::safety::metric
{
using autoware_trajectory_validator::msg::MetricReport;

std::vector<MetricReport> build_metric_reports(
  const std::string & validator_name, const std::string & validator_category,
  const DracArtifact & drac_artifact, const PetArtifact & pet_artifact,
  const RssArtifact & rss_artifact, const DracParamMap & drac_param_map,
  const PetParamMap & pet_param_map);

}  // namespace autoware::trajectory_validator::plugin::safety::metric

#endif  // AUTOWARE__TRAJECTORY_VALIDATOR__FILTERS__SAFETY__COLLISION_CHECK_FILTER__METRIC_HPP_
