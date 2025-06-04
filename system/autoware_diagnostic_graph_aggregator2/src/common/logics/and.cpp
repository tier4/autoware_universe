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

#include "logics/and.hpp"

#include "config/entity.hpp"
#include "config/yaml.hpp"
#include "graph/links.hpp"

#include <algorithm>
#include <string>

namespace autoware::diagnostic_graph_aggregator
{

AndLogic::AndLogic(const LogicConfig & config)
{
  ConfigYaml yaml = config.yaml();
  for (ConfigYaml node : yaml.required("list").list()) {
    links_.push_back(config.parse_unit(node));
  }
}

DiagnosticLevel AndLogic::level() const
{
  if (links_.empty()) {
    return DiagnosticStatus::OK;
  }

  DiagnosticLevel level = DiagnosticStatus::OK;
  for (const auto * const link : links_) {
    level = std::max(level, link->level());
  }
  return level;
}

OrLogic::OrLogic(const LogicConfig & config)
{
  ConfigYaml yaml = config.yaml();
  for (ConfigYaml node : yaml.required("list").list()) {
    links_.push_back(config.parse_unit(node));
  }
}

DiagnosticLevel OrLogic::level() const
{
  if (links_.empty()) {
    return DiagnosticStatus::OK;
  }

  DiagnosticLevel level = DiagnosticStatus::ERROR;
  for (const auto * const link : links_) {
    level = std::min(level, link->level());
  }
  return level;
}

DiagLogic::DiagLogic(const LogicConfig & config)
{
  link_ = config.parse_diag(config.yaml());
}

DiagnosticLevel DiagLogic::level() const
{
  // STALE to ERROR
  return std::min(link_->level(), DiagnosticStatus::ERROR);
}

ConstLogic::ConstLogic(const LogicConfig &)
{
}

DiagnosticLevel OkLogic::level() const
{
  return DiagnosticStatus::OK;
}

DiagnosticLevel StaleLogic::level() const
{
  return DiagnosticStatus::STALE;
}

RegisterLogic<DiagLogic> registration4("diag");
RegisterLogic<AndLogic> registration("and");
RegisterLogic<OrLogic> registration3("or");
RegisterLogic<OkLogic> registration2("ok");
RegisterLogic<StaleLogic> registration6("stale");
RegisterLogic<AndLogic> registration5("short-circuit-and");

}  // namespace autoware::diagnostic_graph_aggregator
