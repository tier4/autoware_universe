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

#ifndef COMMON__LOGICS__AND_HPP_
#define COMMON__LOGICS__AND_HPP_

#include "graph/logic.hpp"

#include <string>
#include <vector>

namespace autoware::diagnostic_graph_aggregator
{

class AndLogic : public Logic
{
public:
  explicit AndLogic(const LogicConfig & config);
  std::string type() const override { return "and"; }
  DiagnosticLevel level() const override;

private:
  std::vector<UnitLink *> links_;
};

class OrLogic : public Logic
{
public:
  explicit OrLogic(const LogicConfig & config);
  std::string type() const override { return "or"; }
  DiagnosticLevel level() const override;

private:
  std::vector<UnitLink *> links_;
};

class DiagLogic : public Logic
{
public:
  explicit DiagLogic(const LogicConfig & config);
  std::string type() const override { return "diag"; }
  DiagnosticLevel level() const override;

private:
  UnitLink * link_;
};

class ConstLogic : public Logic
{
public:
  explicit ConstLogic(const LogicConfig & config);
};

class OkLogic : public ConstLogic
{
public:
  using ConstLogic::ConstLogic;
  std::string type() const override { return "ok"; }
  DiagnosticLevel level() const override;
};

}  // namespace autoware::diagnostic_graph_aggregator

#endif  // COMMON__LOGICS__AND_HPP_
