// Copyright 2024 The Autoware Contributors
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

#include "config/entity.hpp"

#include "graph/links.hpp"

#include <string>
#include <utility>

namespace autoware::diagnostic_graph_aggregator
{

LogicConfig::LogicConfig(UnitConfig unit)
{
  unit_ = unit;
}

ConfigYaml LogicConfig::yaml() const
{
  return ConfigYaml(unit_->yaml);
}

UnitLink * LogicConfig::parse_unit(ConfigYaml yaml) const
{
  unit_->links_temp.push_back(std::make_pair(std::make_unique<UnitLink>(), yaml));
  return unit_->links_temp.back().first.get();
}

UnitLink * LogicConfig::parse_diag(ConfigYaml yaml) const
{
  unit_->diag_temp = std::make_pair(std::make_unique<UnitLink>(), yaml);
  return unit_->diag_temp->first.get();
}

}  // namespace autoware::diagnostic_graph_aggregator
