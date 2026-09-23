// Copyright 2026 Tier IV, Inc.
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

#ifndef AUTOWARE__TENSORRT_E2E__ENGINE_CACHE_HPP_
#define AUTOWARE__TENSORRT_E2E__ENGINE_CACHE_HPP_

#include "autoware/tensorrt_e2e/deployment_manifest.hpp"
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>

#include <filesystem>
#include <string>

namespace autoware::tensorrt_e2e
{

// Tie a serialized engine to BOTH graph and engine bytes, not mtimes. Copying
// an old engine next to a new graph must never silently retain old weights.
inline bool drop_stale_engine(const std::string &onnx_path,
                              const std::string &engine_path,
                              const rclcpp::Logger &logger,
                              const std::string &precision = "default") {
  namespace fs = std::filesystem;
  if (!fs::exists(engine_path))
    return false;
  if (engine_identity_matches(onnx_path, engine_path, precision))
    return false;
  // Missing identity (first upgrade) rebuilds once; subsequent starts reuse a
  // matching engine. Failure to remove is fatal, never silent stale reuse.
  fs::remove(engine_path);
  RCLCPP_INFO(logger, "%s has no matching graph/engine identity; rebuilding",
              engine_path.c_str());
  return true;
}

}  // namespace autoware::tensorrt_e2e

#endif  // AUTOWARE__TENSORRT_E2E__ENGINE_CACHE_HPP_
