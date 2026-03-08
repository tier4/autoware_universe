// Copyright 2025 TIER IV, Inc.
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

#include "autoware/agnocast_wrapper/node.hpp"

#include <autoware_utils_logging/logger_level_configure.hpp>

#include <memory>

namespace autoware::agnocast_wrapper
{

/// @brief Create a LoggerLevelConfigure that uses the appropriate backend
///        (rclcpp or agnocast) based on the wrapper Node's mode.
/// @param node Pointer to the wrapper Node
/// @return Unique pointer to LoggerLevelConfigureInterface
inline std::unique_ptr<autoware_utils_logging::LoggerLevelConfigureInterface>
make_logger_level_configure(Node * node)
{
  if (node->is_using_agnocast()) {
    return std::make_unique<
      autoware_utils_logging::BasicLoggerLevelConfigure<agnocast::Node>>(
      node->get_agnocast_node().get());
  } else {
    return std::make_unique<
      autoware_utils_logging::BasicLoggerLevelConfigure<rclcpp::Node>>(
      node->get_rclcpp_node().get());
  }
}

}  // namespace autoware::agnocast_wrapper
