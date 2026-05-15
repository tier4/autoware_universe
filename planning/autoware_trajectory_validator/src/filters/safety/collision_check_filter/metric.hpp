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

#include <array>
#include <cstdint>
#include <limits>
#include <map>
#include <optional>
#include <string>

namespace autoware::trajectory_validator::plugin::safety::metric
{
using autoware_trajectory_validator::msg::MetricReport;

// Fixed trajectory_type buckets that the metric layer emits per cycle. The order is
// preserved across calls so the Evaluator/Grafana sees a stable time series.
inline constexpr std::array<const char *, 3> kAggregatedTrajectoryTypes = {
  "map_based_predicted_path",
  "constant_curvature_path",
  "diffusion_based_trajectory",
};

// Returns the canonical bucket name for a raw trajectory_type label (which may contain
// extra suffixes such as object id or acceleration). Empty string if no bucket matches.
std::string canonical_trajectory_type(const std::string & raw);

// Masks every assessment_trajectory flag except the target type, for all per-class
// DracParams entries. Needed because assess_drac stops as soon as findings disappear
// across all enabled types, so callers must run assess_drac once per isolated type to
// obtain per-trajectory-type DRAC values for metric emission.
DracParamMap isolate_drac_param_map(const DracParamMap & m, const std::string & target_type);

// Combines per-trajectory-type DRAC into a single overall value. Returns nullopt if
// any sub-type could not avoid collision; otherwise returns max (least negative).
std::optional<double> combine_per_type_drac(
  const std::map<std::string, std::optional<double>> & per_type);

struct PetWorst
{
  double pet{std::numeric_limits<double>::infinity()};
  bool has_finding{false};

  double metric_value() const;
  uint8_t metric_level(const PetParams & params) const;
};

struct DracWorst
{
  std::optional<double> drac{0.0};

  double metric_value() const;
  uint8_t metric_level(const DracParams & params) const;
};

struct RssWorst
{
  double required_deceleration{0.0};
  bool has_violation{false};

  double metric_value() const;
  uint8_t metric_level() const;
};

// Smallest |pet| per trajectory_type bucket. Missing buckets get the default-constructed
// PetWorst (no finding, +inf, OK).
std::map<std::string, PetWorst> compute_pet_worst(const PetArtifact & artifact);

// Wraps per-trajectory-type DRAC values computed by collision_timing_assessment::assess.
// Missing buckets get drac=0.0 (treated as OK).
std::map<std::string, DracWorst> compute_drac_worst(
  const std::map<std::string, std::optional<double>> & required_acceleration_by_type);

// Largest required_deceleration across RSS evaluations. has_violation reflects whether
// the upstream RssArtifact flagged any non-SAFE evaluation.
RssWorst compute_rss_worst(const RssArtifact & artifact);

MetricReport make_metric_report(
  const std::string & validator_name, const std::string & validator_category,
  const std::string & metric_name, double value, uint8_t level);

}  // namespace autoware::trajectory_validator::plugin::safety::metric

#endif  // AUTOWARE__TRAJECTORY_VALIDATOR__FILTERS__SAFETY__COLLISION_CHECK_FILTER__METRIC_HPP_
