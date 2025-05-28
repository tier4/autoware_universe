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

#include "types/diags.hpp"
#include "types/forward.hpp"

#include <rclcpp/time.hpp>

#include <memory>
#include <string>
#include <vector>

namespace autoware::diagnostic_graph_aggregator
{

class BaseUnit
{
public:
  BaseUnit(const std::vector<UnitLink *> parents, int index);
  virtual ~BaseUnit() = default;
  virtual DiagnosticLevel level() const = 0;
  virtual bool is_diag() const = 0;

  int index() const;

private:
  std::vector<UnitLink *> parents_;
  int index_;
};

class NodeUnit : public BaseUnit
{
public:
  NodeUnit(
    const std::vector<UnitLink *> parents, const std::vector<UnitLink *> children, int index,
    const UnitConfig & config);
  ~NodeUnit();
  void reset();
  void dump() const;
  bool is_diag() const override { return false; }
  DiagnosticLevel level() const override;
  DiagNodeStruct create_struct() const;

  std::string path() const;
  std::string type() const;

  void update(const rclcpp::Time & stamp);

private:
  std::string path_;
  std::vector<UnitLink *> children_;
  std::unique_ptr<Logic> logic_;
  std::unique_ptr<LatchLevel> latch_;
};

class DiagUnit : public BaseUnit
{
public:
  DiagUnit(const std::vector<UnitLink *> parents, const UnitConfig & config);
  ~DiagUnit();
  void dump() const;
  bool is_diag() const override { return true; }
  DiagnosticLevel level() const override;
  DiagLeafStruct create_struct() const;

  std::string name() const;
  void update(const rclcpp::Time & stamp);
  void update(const rclcpp::Time & stamp, const DiagnosticStatus & status);

private:
  std::string name_;
  std::unique_ptr<TimeoutLevel> timeout_;
  std::unique_ptr<HysteresisLevel> histeresis_;
  DiagnosticStatus status_;
};

}  // namespace autoware::diagnostic_graph_aggregator

#endif  // COMMON__GRAPH__UNITS_HPP_
