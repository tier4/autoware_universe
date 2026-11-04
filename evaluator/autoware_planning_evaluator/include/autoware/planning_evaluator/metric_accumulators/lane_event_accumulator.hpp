// Copyright 2025 Tier IV, Inc.
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

#ifndef AUTOWARE__PLANNING_EVALUATOR__METRIC_ACCUMULATORS__LANE_EVENT_ACCUMULATOR_HPP_
#define AUTOWARE__PLANNING_EVALUATOR__METRIC_ACCUMULATORS__LANE_EVENT_ACCUMULATOR_HPP_

#include "autoware/planning_evaluator/metrics/metric.hpp"
#include "autoware/planning_evaluator/metrics/output_metric.hpp"

#include <autoware_internal_planning_msgs/msg/planning_factor.hpp>
#include <autoware_internal_planning_msgs/msg/planning_factor_array.hpp>
#include <autoware_utils/math/accumulator.hpp>
#include <lane_event_classifier_msgs/msg/driving_factor.hpp>
#include <lane_event_classifier_msgs/msg/driving_state.hpp>
#include <nlohmann/json.hpp>

#include <tier4_metric_msgs/msg/metric.hpp>
#include <tier4_metric_msgs/msg/metric_array.hpp>

#include <optional>
#include <string>
#include <unordered_map>

namespace planning_diagnostics
{
using autoware_internal_planning_msgs::msg::PlanningFactor;
using autoware_internal_planning_msgs::msg::PlanningFactorArray;
using autoware_utils::Accumulator;
using MetricMsg = tier4_metric_msgs::msg::Metric;
using MetricArrayMsg = tier4_metric_msgs::msg::MetricArray;
using json = nlohmann::json;
using lane_event_classifier_msgs::msg::DrivingFactor;
using lane_event_classifier_msgs::msg::DrivingState;

/**
 * @class LaneEventAccumulator
 * @brief Accumulates lane event statistics.
 *
 * Two scopes (published under metric_to_str(Metric::trajectory_validation) + "/" + <scope> + "/…"):
 * 1) Whole trajectory (per candidate / generator), from ValidationReport.risk.level —
 *    <scope> = <generator_name>
 * 2) Each MetricReport row —
 *    <scope> = <generator_name>/<validator_name>/<metric_name>
 *    Per generator, metric scopes already present in stats (keys `gen/validator/metric` in
 *    stats_by_scope_) are updated every report; missing rows are treated as OK for that step.
 *
 * `count_other_than_safe_as_error`: if false, HIGH_CAUTION and above count as error; if true,
 * LOW_CAUTION also counts as error (anything other than SAFE).
 *
 * Metric rows whose `metric_name` matches `check_*_<32-hex UUID>` (optional `_<suffix>`) are
 * skipped (object-id-specific checks); see `shouldCollectMetricRow` in
 * trajectory_validation_accumulator.cpp.
 */
class LaneEventAccumulator
{
public:
  struct Parameters
  {
    bool count_other_than_safe_as_error = false;
    double initial_span_duration_s =
      0.1;  // [s] default duration on the first frame of a span
  } parameters;

  LaneEventAccumulator() = default;
  ~LaneEventAccumulator() = default;

  /**
   * @brief Consume a new validation report array (one message may contain multiple trajectories).
   */
  void update(const DrivingFactor & msg);

  /**
   * @brief Append live metrics for publishing (names follow the scheme in the class comment).
   */
  bool addMetricMsg(const Metric & metric, MetricArrayMsg & metrics_msg);

  // /**
  //  * @brief Append instantaneous MetricReport.metric_value for publishing (/{scope}/value).
  //  */
  // void addInstantMetricMsgs(const DrivingState & msg, MetricArrayMsg & metrics_msg);

  /**
   * @brief Final statistics for output.json (min/max/mean/total duration, count).
   */
  json getOutputJson(const OutputMetric & output_metric);

  static std::string toDrivingStateName(uint8_t state)
  {
    switch (state) {
      case DrivingState::LANE_FOLLOWING:
        return "lane_following";
      case DrivingState::LANE_CHANGING:
        return "lane_change";
      case DrivingState::ABORTING_LANE_CHANGE:
        return "aborting_lane_change";
      case DrivingState::INTENTIONAL_LANE_CROSSING:
        return "intentional_lane_crossing";
      case DrivingState::ABORTING_INTENTIONAL_LANE_CROSSING:
        return "aborting_intentional_lane_crossing";
      case DrivingState::UNKNOWN:
        return "unknown";
      default:
        return "";
    }
  }

private:
  void flushOpenForJson();

  struct SpanStats
  {
    double state_durration_s{0.0};
    double completed_duration_total_s_{0.0};
    uint64_t count{0};
    Accumulator<double> duration_accumulator;
  };
  
  uint8_t prev_state_{0};
  std::optional<double> last_update_time_s_;

  std::unordered_map<uint8_t, SpanStats> stats_by_state_;
  std::vector<DrivingFactor> factors_;
  std::unordered_map<std::string, Accumulator<double>> value_stats_by_scope_;
};

}  // namespace planning_diagnostics

#endif  // AUTOWARE__PLANNING_EVALUATOR__METRIC_ACCUMULATORS__LANE_EVENT_ACCUMULATOR_HPP_
