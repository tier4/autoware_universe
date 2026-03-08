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

#include <autoware_utils_diagnostics/diagnostics_interface.hpp>

#include <memory>
#include <string>

namespace autoware::agnocast_wrapper
{

/// @brief Create a DiagnosticsInterface that uses the appropriate backend
///        (rclcpp or agnocast) based on the wrapper Node's mode.
/// @param node Pointer to the wrapper Node
/// @param diagnostic_name Name of the diagnostic
/// @return Unique pointer to DiagnosticsInterfaceBase
inline std::unique_ptr<autoware_utils_diagnostics::DiagnosticsInterfaceBase>
make_diagnostics_interface(Node * node, const std::string & diagnostic_name)
{
  if (node->is_using_agnocast()) {
    return std::make_unique<
      autoware_utils_diagnostics::BasicDiagnosticsInterface<agnocast::Node>>(
      node->get_agnocast_node().get(), diagnostic_name);
  } else {
    return std::make_unique<
      autoware_utils_diagnostics::BasicDiagnosticsInterface<rclcpp::Node>>(
      node->get_rclcpp_node().get(), diagnostic_name);
  }
}

}  // namespace autoware::agnocast_wrapper
