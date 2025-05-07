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
  const std::vector<UnitLink *> parents, const std::vector<UnitLink *> children,
  std::unique_ptr<Logic> && logic, int index, const std::string & path)
: BaseUnit(parents, index)
{
  children_ = children;
  path_ = path;
  logic_ = std::move(logic);
}

NodeUnit::~NodeUnit()
{
}

DiagnosticLevel NodeUnit::level() const
{
  return logic_->level();
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
  (void)stamp;
}

DiagNodeStruct NodeUnit::create_struct() const
{
  DiagNodeStruct msg;
  msg.path = path();
  msg.type = type();
  return msg;
}

DiagUnit::DiagUnit(const std::vector<UnitLink *> parents, const std::string & name)
: BaseUnit(parents, -1)
{
  name_ = name;
  status_.level = DiagnosticStatus::STALE;
}

DiagUnit::~DiagUnit()
{
}

DiagnosticLevel DiagUnit::level() const
{
  return status_.level;
}

std::string DiagUnit::name() const
{
  return name_;
}

void DiagUnit::update(const rclcpp::Time & stamp)
{
  (void)stamp;
}

void DiagUnit::update(
  const rclcpp::Time & now_stamp, const rclcpp::Time & msg_stamp, const DiagnosticStatus & status)
{
  (void)now_stamp;
  (void)msg_stamp;
  status_ = status;
}

DiagLeafStruct DiagUnit::create_struct() const
{
  DiagLeafStruct msg;
  msg.name = name();
  return msg;
}

// dump functions

std::string str_level(DiagnosticLevel level)
{
  switch (level) {
    case DiagnosticStatus::OK:
      return "OK";
    case DiagnosticStatus::WARN:
      return "WARN";
    case DiagnosticStatus::ERROR:
      return "ERROR";
    case DiagnosticStatus::STALE:
      return "STALE";
    default:
      return "-----";
  }
}

template <typename T>
void dump_data(const T & data, int width = 0)
{
  std::cout << "| ";
  if (width) {
    std::cout << std::setw(width);
  }
  std::cout << data;
  std::cout << " ";
}

void NodeUnit::dump() const
{
  dump_data("Node");
  dump_data(this);
  dump_data(str_level(level()), 5);
  dump_data(type(), 5);
  dump_data(path());
  std::cout << "|" << std::endl;
}

void DiagUnit::dump() const
{
  dump_data("Diag");
  dump_data(this);
  dump_data(str_level(level()), 5);
  dump_data(" --- ");
  dump_data(name());
  std::cout << "|" << std::endl;
}

}  // namespace autoware::diagnostic_graph_aggregator
