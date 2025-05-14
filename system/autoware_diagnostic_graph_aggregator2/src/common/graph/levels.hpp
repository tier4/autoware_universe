// Copyright 2023 The Autoware Contributors
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

#ifndef COMMON__GRAPH__LEVELS_HPP_
#define COMMON__GRAPH__LEVELS_HPP_

#include "types/diags.hpp"
#include "types/forward.hpp"

#include <rclcpp/time.hpp>

#include <optional>

namespace autoware::diagnostic_graph_aggregator
{

class LatchLevel
{
public:
  explicit LatchLevel(ConfigYaml yaml);
  void update(const rclcpp::Time & stamp, DiagnosticLevel level);
  DiagnosticLevel level() const;
  DiagnosticLevel input_level() const;
  DiagnosticLevel latch_level() const;

private:
  void update_latch_status(const rclcpp::Time & stamp, DiagnosticLevel level);

  double latch_duration_;
  bool latch_enabled_;

  bool warn_latched_;
  bool error_latched_;
  std::optional<rclcpp::Time> warn_stamp_;
  std::optional<rclcpp::Time> error_stamp_;
  DiagnosticLevel input_level_;
};

}  // namespace autoware::diagnostic_graph_aggregator

#endif  // COMMON__GRAPH__LEVELS_HPP_
