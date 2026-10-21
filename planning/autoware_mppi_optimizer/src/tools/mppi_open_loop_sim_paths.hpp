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

#pragma once

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <cstdlib>
#include <filesystem>
#include <stdexcept>
#include <string>

namespace autoware::mppi_optimizer::open_loop_sim
{

inline std::string packageShareFilePath(const char * package_name, const std::filesystem::path & rel)
{
  try {
    const auto share = ament_index_cpp::get_package_share_directory(package_name);
    const auto path = std::filesystem::path(share) / rel;
    if (std::filesystem::is_regular_file(path)) {
      return path.string();
    }
  } catch (const std::exception &) {
  }
  return {};
}

/** Same yaml as diffusion / neural_net planning sim when autoware_launch is installed. */
inline std::string defaultPlanningMppiParamsYaml()
{
  if (const auto launch = packageShareFilePath(
        "autoware_launch",
        "config/planning/neural_net_planner/mppi_optimizer.param.yaml");
      !launch.empty()) {
    return launch;
  }
  return packageShareFilePath(
    "autoware_mppi_optimizer", std::filesystem::path("config") / "mppi_optimizer.param.yaml");
}

inline std::string defaultVehicleDescriptionYaml(const char * filename)
{
  return packageShareFilePath(
    "j6_gen2_description", std::filesystem::path("config") / filename);
}

inline std::string defaultOutDir(const char * subdir)
{
  std::filesystem::path root;
  if (const char * cache = std::getenv("XDG_CACHE_HOME"); cache != nullptr && cache[0] != '\0') {
    root = cache;
  } else if (const char * home = std::getenv("HOME"); home != nullptr && home[0] != '\0') {
    root = std::filesystem::path(home) / ".cache";
  } else {
    throw std::runtime_error(
      "Cannot resolve default open-loop sim output directory: set HOME or XDG_CACHE_HOME, or "
      "pass --out-dir");
  }
  return (root / "autoware" / subdir).string();
}

}  // namespace autoware::mppi_optimizer::open_loop_sim
