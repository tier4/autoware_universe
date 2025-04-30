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

#include "graph/units.hpp"

#include "graph/logic.hpp"

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::diagnostic_graph_aggregator
{

NodeUnit::NodeUnit(
  const std::vector<UnitLink *> parents, const std::vector<UnitLink *> children,
  std::unique_ptr<Logic> && logic)
{
  parents_ = parents;
  children_ = children;
  logic_ = std::move(logic);
}

NodeUnit::~NodeUnit()
{
}

std::string NodeUnit::path() const
{
  return path_;
}

std::string NodeUnit::type() const
{
  return logic_->type();
}

DiagUnit::DiagUnit(const std::vector<UnitLink *> parents, const std::string & name)
{
  parents_ = parents;
  name_ = name;
}

DiagUnit::~DiagUnit()
{
}

std::string DiagUnit::name() const
{
  return name_;
}

}  // namespace autoware::diagnostic_graph_aggregator
