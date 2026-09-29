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

#include "autoware/tensorrt_e2e/engine_identity.hpp"
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
  std::cout
      << "Engine identity, pose reset, rolling latency: PASS\n";
}
