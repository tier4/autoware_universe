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

#include "autoware/planning_evaluator/metric_accumulators/lane_event_accumulator.hpp"

#include <cmath>
#include <regex>

#include <rclcpp/rclcpp.hpp>

namespace planning_diagnostics
{
void LaneEventAccumulator::flushOpenForJson()
{
  if (!last_update_time_s_.has_value()) {
    return;
  }

  auto & stats = stats_by_state_[prev_state_];
  if (stats.state_durration_s > 0.0) {
    stats.duration_accumulator.add(stats.state_durration_s);
    stats.completed_duration_total_s_ += stats.state_durration_s;
  }
}

void LaneEventAccumulator::update(const DrivingFactor & msg)
{
  if (msg.header.stamp.sec == 0) {
    return;
  }

  const auto & stamp = msg.header.stamp;
  const double report_time_s =
    static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
  const auto state = msg.driving_state.state;

  if (!last_update_time_s_.has_value()) {
    auto & stats = stats_by_state_[state];
    stats.count += 1;
    last_update_time_s_ = report_time_s;
    prev_state_ = state;
    return;
  }

  const double dt_s = report_time_s - last_update_time_s_.value();

if (state == prev_state_) {
    stats_by_state_[state].state_durration_s += dt_s;
  } else {
    auto & prev_stats = stats_by_state_[prev_state_];
    prev_stats.state_durration_s += dt_s;
    prev_stats.duration_accumulator.add(prev_stats.state_durration_s);
    prev_stats.completed_duration_total_s_ += prev_stats.state_durration_s;

    auto & new_stats = stats_by_state_[state];
    new_stats.count += 1;
    new_stats.state_durration_s = 0.0;
  }

  prev_state_ = state;
  last_update_time_s_ = report_time_s;
}

bool LaneEventAccumulator::addMetricMsg(
  const Metric & metric, MetricArrayMsg & metrics_msg)
{
  if (metric != Metric::lane_event) {
    return false;
  }

  const std::string base = metric_to_str.at(Metric::lane_event) + "/";
  bool any = false;

  for (const auto & [state, stats] : stats_by_state_) {
    if (state == DrivingState::UNKNOWN) {
      continue;
    }
    const std::string scope = base + toDrivingStateName(state);

    MetricMsg m_count;
    m_count.name = scope + "/count";
    m_count.value = std::to_string(stats.count);
    metrics_msg.metric_array.push_back(m_count);

    MetricMsg m_duration;
    m_duration.name = scope + "/duration";
    m_duration.value = std::to_string(stats.state_durration_s);
    metrics_msg.metric_array.push_back(m_duration);

    any = true;
  }
  return any;
}

json LaneEventAccumulator::getOutputJson(const OutputMetric & output_metric)
{
  json j;
  if (output_metric != OutputMetric::lane_event) {
    return j;
  }

  flushOpenForJson();
  const std::string base = metric_to_str.at(Metric::lane_event) + "/";

  for (const auto & [state, stats] : stats_by_state_) {
    if (state == DrivingState::UNKNOWN) {
      continue;
    }
    const std::string scope = base + toDrivingStateName(state);

    if (stats.duration_accumulator.count() > 0) {
      j[scope + "/duration/min"] = stats.duration_accumulator.min();
      j[scope + "/duration/max"] = stats.duration_accumulator.max();
      j[scope + "/duration/mean"] = stats.duration_accumulator.mean();
      j[scope + "/duration/total"] = stats.completed_duration_total_s_;
    }
    if (stats.count > 0) {
      j[scope + "/count"] = stats.count;
    }
  }
  return j;
}

}  // namespace planning_diagnostics
