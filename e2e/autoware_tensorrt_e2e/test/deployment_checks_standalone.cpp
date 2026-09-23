// Copyright 2026 TIER IV, Inc.
// Licensed under the Apache License, Version 2.0.
#include "autoware/tensorrt_e2e/deployment_manifest.hpp"
#include "autoware/tensorrt_e2e/pose_discontinuity.hpp"
#include "autoware/tensorrt_e2e/rolling_latency.hpp"
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
  nlohmann::json manifest = {{"schema_version", 1},
                             {"files", {{"graph.onnx", digest}}},
                             {"planner_file", "graph.onnx"},
                             {"extractor_file", "graph.onnx"},
                             {"parameters", nlohmann::json::object()}};
  const auto path = root / "deployment_manifest.json";
  std::ofstream(path) << manifest;
  check(validate_deployment_manifest(path) == manifest);
  std::ofstream(file) << "wrong graph";
  bool rejected = false;
  try {
    validate_deployment_manifest(path);
  } catch (const std::runtime_error &) {
    rejected = true;
  }
  check(rejected);
  check(file_sha256(file) != digest);
  check(!engine_identity_matches(file.string(), engine.string()));
  manifest["files"] = {{"../graph.onnx", digest}};
  std::ofstream(path) << manifest;
  rejected = false;
  try {
    validate_deployment_manifest(path);
  } catch (const std::runtime_error &) {
    rejected = true;
  }
  check(rejected);
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
      << "Manifest tampering/path checks, pose reset, rolling latency: PASS\n";
}
