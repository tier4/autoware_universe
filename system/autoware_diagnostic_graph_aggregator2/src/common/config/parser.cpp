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

#include "config/parser.hpp"

#include "config/entity.hpp"
#include "graph/port.hpp"

#include <string>
#include <utility>

namespace autoware::diagnostic_graph_aggregator
{

LogicConfig::LogicConfig(UnitConfig unit, LogicEntity * data)
{
  unit_ = unit;
  data_ = data;
}

ConfigYaml LogicConfig::yaml() const
{
  return ConfigYaml(unit_->yaml);
}

ChildPort * LogicConfig::parse(ConfigYaml yaml) const
{
  data_->units.push_back(std::make_pair(std::make_unique<ChildPort>(), yaml));
  return data_->units.back().first.get();
}

ChildPort * LogicConfig::parse(std::string name) const
{
  unit_->diag = std::make_pair(std::make_unique<ChildPort>(), name);
  return unit_->diag->first.get();
}

}  // namespace autoware::diagnostic_graph_aggregator
