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

#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>

#include <filesystem>
#include <string>

namespace autoware::tensorrt_e2e
{

/**
 * @brief Delete a cached TensorRT engine that is older than the ONNX beside it.
 *
 * TrtCommon reuses `<model>.engine` whenever it deserializes and the plan's TensorRT
 * version matches. Its two freshness checks -- validateNetworkIO() and
 * validateProfileDims() -- both return success when the caller passes nothing, and log
 * only "Network IO is empty, skipping validation. It might lead to undefined behavior".
 * This node cannot declare a static IO list the way autoware_bevfusion does, because it
 * is model-agnostic and reads its bindings out of whatever engine it is given. So a
 * re-exported ONNX dropped next to a stale engine would silently keep running the old
 * weights, and obey_graph_precision() would be silently inert as well, since the layer
 * precisions it pins are applied to a network that a cached plan never builds.
 *
 * Comparing mtimes is enough for that: the exporter writes a new ONNX, the engine beside
 * it is then older, and it is rebuilt. It intentionally does not hash the graph -- an
 * engine that is merely *newer* than its ONNX is left alone, which is the normal state
 * after a build.
 *
 * @param onnx_path Path to the ONNX the engine was built from.
 * @param engine_path Path to the cached engine (TrtCommonConfig derives it from the ONNX
 *                    by replacing the extension when it is not given explicitly).
 * @param logger Logger for the one-line notice when a stale engine is removed.
 * @return true when a stale engine was deleted.
 */
inline bool drop_stale_engine(
  const std::string & onnx_path, const std::string & engine_path, const rclcpp::Logger & logger)
{
  namespace fs = std::filesystem;
  std::error_code ec;
  if (!fs::exists(engine_path, ec) || !fs::exists(onnx_path, ec)) {
    return false;
  }
  const auto engine_time = fs::last_write_time(engine_path, ec);
  if (ec) {
    return false;
  }
  const auto onnx_time = fs::last_write_time(onnx_path, ec);
  if (ec || engine_time >= onnx_time) {
    return false;
  }
  fs::remove(engine_path, ec);
  if (ec) {
    RCLCPP_WARN(
      logger, "%s is older than %s but could not be removed (%s); it will be reused as is.",
      engine_path.c_str(), onnx_path.c_str(), ec.message().c_str());
    return false;
  }
  RCLCPP_INFO(
    logger, "%s is older than %s: removed, the engine will be rebuilt.", engine_path.c_str(),
    onnx_path.c_str());
  return true;
}

}  // namespace autoware::tensorrt_e2e

#endif  // AUTOWARE__TENSORRT_E2E__ENGINE_CACHE_HPP_
