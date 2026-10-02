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

#include "autoware/tensorrt_e2e/curvature_bias.hpp"
#include "autoware/tensorrt_e2e/engine_identity.hpp"
#include "autoware/tensorrt_e2e/planning_time.hpp"
#include "autoware/tensorrt_e2e/pose_discontinuity.hpp"
#include "autoware/tensorrt_e2e/rolling_latency.hpp"
#include "autoware/tensorrt_e2e/training_ego_shape.hpp"
#include <iostream>
#include <limits>
using namespace autoware::tensorrt_e2e;
void check(bool ok) {
  if (!ok)
    throw std::runtime_error("Regression failed");
}
int main(int argc, char **argv) {
  if (argc != 2)
    throw std::runtime_error("Pass an empty scratch directory");
  const std::filesystem::path root(argv[1]);
  // The ego shape the model learned, chosen by the vehicle it runs on.
  check(std::string(training_ego_shape_for(2.75).vehicle) == "jpntaxi");
  check(std::string(training_ego_shape_for(4.76012).vehicle) == "j6");
  check(training_ego_shape_for(4.76012).length == 7.2369);
  for (const double unknown : {3.8, 1.0, std::numeric_limits<double>::quiet_NaN()}) {
    bool refused = false;
    try {
      training_ego_shape_for(unknown);
    } catch (const std::runtime_error &) {
      refused = true;
    }
    check(refused);
  }
  const auto file = root / "graph.onnx";
  std::ofstream(file) << "original graph";
  const auto digest = file_sha256(file);
  const auto engine = root / "graph.engine";
  std::ofstream(engine) << "engine one";
  check(!engine_identity_matches(file.string(), engine.string()));
  record_engine_identity(file.string(), engine.string());
  check(engine_identity_matches(file.string(), engine.string()));
  check(!engine_identity_matches(file.string(), engine.string(), "fp16"));
  std::ofstream(engine) << "engine two";
  check(!engine_identity_matches(file.string(), engine.string()));
  record_engine_identity(file.string(), engine.string());
  std::ofstream(file) << "wrong graph";
  check(file_sha256(file) != digest);
  check(!engine_identity_matches(file.string(), engine.string()));
  PoseContinuityLimits limits;
  check(!pose_discontinuous({100000, 0, 1, 0}, {100003, 0, 1, 0}, 0.1, limits));
  check(pose_discontinuous({100000, 0, 1, 0}, {100020, 0, 1, 0}, 0.1, limits));
  check(pose_discontinuous({0, 0, 1, 0}, {0, 0, 0, 1}, 0.1, limits));
  check(
      !pose_discontinuous({0, 0, -1, 0.001}, {0, 0, -1, -0.001}, 0.1, limits));
  RollingLatency latency;
  for (int i = 1; i <= 100; ++i)
    latency.add(i);
  check(latency.percentile(.95) == 95 && latency.percentile(.99) == 99);
  latency.add(-1);
  latency.add(std::numeric_limits<double>::infinity());
  check(latency.percentile(.99) == 99);
  for (int i = 0; i < 512; ++i)
    latency.add(7);
  check(latency.percentile(.99) == 7);
  latency.clear();
  check(latency.percentile(.95) == 0);
  // When the plan starts: a package and its graph must agree, and a planning-time plan
  // starts at the newest odometry, never before the cloud.
  const auto refuses = [](const auto &call) {
    try {
      call();
    } catch (const std::runtime_error &) {
      return true;
    }
    return false;
  };
  check(parse_planning_time("cloud_stamp") == PlanningTime::kCloudStamp);
  check(parse_planning_time("planning_time") == PlanningTime::kPlanningTime);
  check(refuses([] { parse_planning_time("now"); }));
  check(refuses([] { parse_planning_time(""); }));
  check(std::string(planning_time_name(PlanningTime::kPlanningTime)) == "planning_time");
  check(!refuses([] { check_planning_time_inputs(PlanningTime::kCloudStamp, false); }));
  check(!refuses([] { check_planning_time_inputs(PlanningTime::kPlanningTime, true); }));
  check(refuses([] { check_planning_time_inputs(PlanningTime::kCloudStamp, true); }));
  check(refuses([] { check_planning_time_inputs(PlanningTime::kPlanningTime, false); }));
  const int64_t cloud = 1'000'000'000;
  const int64_t odometry = cloud + 146'000'000;
  check(planning_stamp_ns(PlanningTime::kCloudStamp, cloud, odometry) == cloud);
  check(planning_stamp_ns(PlanningTime::kPlanningTime, cloud, odometry) == odometry);
  check(planning_stamp_ns(PlanningTime::kPlanningTime, cloud, cloud) == cloud);
  check(!planning_stamp_ns(PlanningTime::kPlanningTime, cloud, cloud - 1));
  // The curvature bias, tick for tick as OnePlanner's projects/resworld/curvature_bias.py
  // runs it over these frames (values printed by that function, L = 2.75 m): held at
  // rest and below 1 m/s, started on the first moving tick, signed through a reverse.
  {
    const double v[] = {0.0, 0.0, 0.5, 3.0, 6.0, 8.0, 8.0, 0.4, 0.0, 7.0, 9.0, -2.0, 10.0};
    const double r[] = {0.0, 0.01, 0.02, 0.05, 0.06, 0.08, -0.04, 0.0, 0.0, 0.07, 0.09, 0.01, 0.1};
    const double s[] = {0.01, 0.01, 0.02, 0.03, 0.02, 0.03, -0.01, 0.0, 0.0, 0.025, 0.03, 0.0,
                        0.028};
    const double b[] = {0.0, 0.0, 0.0, 5.754301851692e-03, 5.606624598994e-03,
                        5.288689733601e-03, 4.964257873251e-03, 4.964257873251e-03,
                        4.964257873251e-03, 4.766392655051e-03, 4.489436385117e-03,
                        4.026631111451e-03, 3.821252805637e-03};
    CurvatureBiasFilter filter(2.0, 1.0);
    for (int i = 0; i < 13; ++i) {
      filter.update(v[i], r[i], s[i], 2.75, 0.1);
      check(std::abs(filter.value() - b[i]) <= 1e-7 * std::abs(b[i]) + 1e-12);
    }
  }
  check(refuses([] { CurvatureBiasFilter(0.0, 1.0); }));
  check(!refuses([] { check_curvature_bias_inputs(false, false); }));
  check(!refuses([] { check_curvature_bias_inputs(true, true); }));
  check(refuses([] { check_curvature_bias_inputs(false, true); }));
  check(refuses([] { check_curvature_bias_inputs(true, false); }));
  std::cout
      << "Engine identity, pose reset, rolling latency, planning time, curvature bias: PASS\n";
}
