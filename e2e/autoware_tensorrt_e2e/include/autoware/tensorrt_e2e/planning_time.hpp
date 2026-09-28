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

#ifndef AUTOWARE__TENSORRT_E2E__PLANNING_TIME_HPP_
#define AUTOWARE__TENSORRT_E2E__PLANNING_TIME_HPP_

#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>

namespace autoware::tensorrt_e2e
{

/**
 * @brief When the plan starts: the other convention a model and this node must share.
 *
 * `kCloudStamp`: the ego state, ego history, map, route and goal are all taken at the
 * pacing cloud's stamp T, and the trajectory is stamped T -- the convention every model
 * before OnePlanner's latency-aware line was trained on.
 *
 * `kPlanningTime`: the plan starts at the newest odometry sample, T + L. The ego state,
 * history, map, route and goal are taken there; the BEV maps (current one included) are
 * warped into that pose; traffic lights stay selected at T, as in training; the model
 * reads `sensor_latency` = L; and the trajectory is stamped T + L. Training must draw L
 * from the distribution fed HERE, which is not the cloud's age: L runs to the newest
 * odometry sample the node holds when the cloud arrives, so it is the cloud age minus up to
 * one odometry period (20 ms at 50 Hz) minus the callback that has not run yet, and it comes
 * in odometry-period steps. Measured on a replay with the car's LiDAR timing (2025-09-03
 * split 0, packets retimed): concatenated cloud 143 ms old at publish, L = 110.5 ms on 425
 * of 569 plans (90.5 on the rest of the low side).
 *
 * The package states its convention (`planning_time` in the ml_package file) and the
 * graph states it too, by whether it reads `sensor_latency`: the node refuses any
 * disagreement between the two, because either one run the other way plans from inputs
 * that were never seen together in training.
 */
enum class PlanningTime { kCloudStamp, kPlanningTime };

//! The graph input a planning-time model reads: seconds from the cloud stamp to planning.
inline constexpr const char * SENSOR_LATENCY_TENSOR = "sensor_latency";

inline PlanningTime parse_planning_time(const std::string & value)
{
  if (value == "cloud_stamp") {
    return PlanningTime::kCloudStamp;
  }
  if (value == "planning_time") {
    return PlanningTime::kPlanningTime;
  }
  throw std::runtime_error(
    "planning_time must be 'cloud_stamp' or 'planning_time', got '" + value + "'");
}

inline const char * planning_time_name(const PlanningTime planning_time)
{
  return planning_time == PlanningTime::kPlanningTime ? "planning_time" : "cloud_stamp";
}

/**
 * @brief Refuse a package whose stated convention the graph contradicts.
 * @throws std::runtime_error when the graph reads `sensor_latency` under `cloud_stamp`, or
 * does not read it under `planning_time`.
 */
inline void check_planning_time_inputs(
  const PlanningTime planning_time, const bool graph_reads_sensor_latency)
{
  if (planning_time == PlanningTime::kPlanningTime && !graph_reads_sensor_latency) {
    throw std::runtime_error(
      "planning_time is 'planning_time' but the graph has no '" +
      std::string(SENSOR_LATENCY_TENSOR) +
      "' input: the model was trained to plan at the cloud stamp; regenerate its ml_package "
      "file, or deploy a model trained with a sensor latency");
  }
  if (planning_time == PlanningTime::kCloudStamp && graph_reads_sensor_latency) {
    throw std::runtime_error(
      "The graph reads '" + std::string(SENSOR_LATENCY_TENSOR) +
      "', so it was trained to plan at the newest odometry, but planning_time is "
      "'cloud_stamp': set planning_time: \"planning_time\" (the exporter's ml_package file "
      "does)");
  }
}

/**
 * @brief The stamp the plan starts at, or std::nullopt while it is not available yet.
 * @param sensor_stamp_ns the pacing cloud's stamp T.
 * @param newest_odometry_ns the newest odometry sample's stamp.
 *
 * Under `kPlanningTime` this is the newest odometry sample itself, never an extrapolation
 * to the wall clock: the ego state, the trajectory stamp and `sensor_latency` then all
 * describe one measured instant. An odometry that has not yet reached T is waited for,
 * because training never saw a negative latency.
 */
inline std::optional<int64_t> planning_stamp_ns(
  const PlanningTime planning_time, const int64_t sensor_stamp_ns,
  const int64_t newest_odometry_ns)
{
  if (planning_time == PlanningTime::kCloudStamp) {
    return sensor_stamp_ns;
  }
  if (newest_odometry_ns < sensor_stamp_ns) {
    return std::nullopt;
  }
  return newest_odometry_ns;
}

}  // namespace autoware::tensorrt_e2e

#endif  // AUTOWARE__TENSORRT_E2E__PLANNING_TIME_HPP_
