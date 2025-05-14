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

#include "graph/levels.hpp"

#include "config/yaml.hpp"

#include <algorithm>

namespace autoware::diagnostic_graph_aggregator
{

LatchLevel::LatchLevel(ConfigYaml yaml)
{
  const auto latch = yaml.optional("latch");
  latch_enabled_ = latch.exists();
  if (latch_enabled_) {
    latch_duration_ = latch.float64();
  }

  warn_latched_ = false;
  error_latched_ = false;
  warn_stamp_ = std::nullopt;
  error_stamp_ = std::nullopt;
  input_level_ = DiagnosticStatus::STALE;
}

void LatchLevel::update(const rclcpp::Time & stamp, DiagnosticLevel level)
{
  input_level_ = level;

  if (latch_enabled_) {
    update_latch_status(stamp, level);
  }
}

void LatchLevel::update_latch_status(const rclcpp::Time & stamp, DiagnosticLevel level)
{
  if (level != DiagnosticStatus::STALE) {
    if (level < DiagnosticStatus::WARN) {
      warn_stamp_ = std::nullopt;
    } else {
      warn_stamp_ = warn_stamp_.value_or(stamp);
    }
    if (level < DiagnosticStatus::ERROR) {
      error_stamp_ = std::nullopt;
    } else {
      error_stamp_ = error_stamp_.value_or(stamp);
    }
  }

  if (warn_stamp_ && !warn_latched_) {
    const auto duration = (stamp - *warn_stamp_).seconds();
    warn_latched_ = latch_duration_ < duration;
  }
  if (error_stamp_ && !error_latched_) {
    const auto duration = (stamp - *error_stamp_).seconds();
    error_latched_ = latch_duration_ < duration;
  }
}

DiagnosticLevel LatchLevel::level() const
{
  return std::max(input_level(), latch_level());
}

DiagnosticLevel LatchLevel::input_level() const
{
  return input_level_;
}

DiagnosticLevel LatchLevel::latch_level() const
{
  if (error_latched_) {
    return DiagnosticStatus::ERROR;
  }
  if (warn_latched_) {
    return DiagnosticStatus::WARN;
  }
  return DiagnosticStatus::OK;
}

}  // namespace autoware::diagnostic_graph_aggregator
