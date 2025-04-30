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

#ifndef COMMON__GRAPH__UNITS_HPP_
#define COMMON__GRAPH__UNITS_HPP_

#include "types/forward.hpp"

#include <memory>
#include <string>
#include <vector>

namespace autoware::diagnostic_graph_aggregator
{

class BaseUnit
{
public:
  virtual ~BaseUnit() = default;
};

class NodeUnit : public BaseUnit
{
public:
  NodeUnit(
    const std::vector<UnitLink *> parents, const std::vector<UnitLink *> children,
    std::unique_ptr<Logic> && logic);
  ~NodeUnit();
  std::string path() const;
  std::string type() const;

private:
  std::vector<UnitLink *> parents_;
  std::vector<UnitLink *> children_;
  std::string path_;
  std::unique_ptr<Logic> logic_;
};

class LeafUnit : public BaseUnit
{
};

class DiagUnit : public LeafUnit
{
public:
  DiagUnit(const std::vector<UnitLink *> parents, const std::string & name);
  ~DiagUnit();
  std::string name() const;

private:
  std::vector<UnitLink *> parents_;
  std::string name_;
};

}  // namespace autoware::diagnostic_graph_aggregator

#endif  // COMMON__GRAPH__UNITS_HPP_
