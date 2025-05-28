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

#include "config/entity.hpp"
#include "graph/levels.hpp"
#include "graph/logic.hpp"

#include <memory>
#include <string>
#include <utility>
#include <vector>

//
#include <iostream>

namespace autoware::diagnostic_graph_aggregator
{

BaseUnit::BaseUnit(const std::vector<UnitLink *> parents, int index)
{
  parents_ = parents;
  index_ = index;
}

int BaseUnit::index() const
{
  return index_;
}

NodeUnit::NodeUnit(
  const std::vector<UnitLink *> parents, const std::vector<UnitLink *> children, int index,
  const UnitConfig & config)
: BaseUnit(parents, index)
{
  children_ = children;
  path_ = config->path;
  logic_ = std::move(config->logic);
  latch_ = std::make_unique<LatchLevel>(config->yaml);
}

NodeUnit::~NodeUnit()
{
}

void NodeUnit::reset()
{
  latch_->reset();
}

DiagnosticLevel NodeUnit::level() const
{
  return latch_->level();
}

std::string NodeUnit::path() const
{
  return path_;
}

std::string NodeUnit::type() const
{
  return logic_->type();
}

void NodeUnit::update(const rclcpp::Time & stamp)
{
  latch_->update(stamp, logic_->level());
}

DiagNodeStruct NodeUnit::create_struct() const
{
  DiagNodeStruct msg;
  msg.path = path();
  msg.type = type();
  return msg;
}

DiagUnit::DiagUnit(const std::vector<UnitLink *> parents, const UnitConfig & config)
: BaseUnit(parents, -1)
{
  name_ = config->data;
  timeout_ = std::make_unique<TimeoutLevel>(config->yaml);
  histeresis_ = std::make_unique<HysteresisLevel>(config->yaml);
  status_.level = DiagnosticStatus::STALE;
}

DiagUnit::~DiagUnit()
{
}

DiagnosticLevel DiagUnit::level() const
{
  return histeresis_->level();
}

std::string DiagUnit::name() const
{
  return name_;
}

void DiagUnit::update(const rclcpp::Time & stamp)
{
  timeout_->update(stamp);
  histeresis_->update(stamp, timeout_->level());
}

void DiagUnit::update(const rclcpp::Time & stamp, const DiagnosticStatus & status)
{
  status_ = status;
  timeout_->update(stamp, status.level);
  histeresis_->update(stamp, timeout_->level());
}

DiagLeafStruct DiagUnit::create_struct() const
{
  DiagLeafStruct msg;
  msg.name = name();
  return msg;
}

// TODO(Takagi, Isamu): DO NOT use status_.level in create_status.

// dump functions

std::string str_level(DiagnosticLevel level)
{
  // clang-format off
  switch (level) {
    case DiagnosticStatus::OK:    return "OK";
    case DiagnosticStatus::WARN:  return "WARN";
    case DiagnosticStatus::ERROR: return "ERROR";
    case DiagnosticStatus::STALE: return "STALE";
    default: return "-----";
  }
  // clang-format on
}

template <typename T>
void dump_data(const T & data, int width = 0)
{
  std::cout << "| ";
  if (width) {
    std::cout << std::setw(width) << std::left;
  }
  std::cout << data;
  std::cout << " ";
}

void NodeUnit::dump() const
{
  dump_data("Node");
  dump_data(this);
  dump_data(str_level(latch_->level()), 5);
  dump_data(str_level(latch_->input_level()), 5);
  dump_data(str_level(latch_->latch_level()), 5);
  dump_data(type(), 5);
  dump_data(path(), 50);
  std::cout << "|" << std::endl;
}

void DiagUnit::dump() const
{
  dump_data("Diag");
  dump_data(this);
  dump_data(str_level(level()), 5);
  dump_data(str_level(histeresis_->input_level()), 5);
  dump_data(" --- ");
  dump_data(" --- ");
  dump_data(name(), 50);
  std::cout << "|" << std::endl;
}

}  // namespace autoware::diagnostic_graph_aggregator
