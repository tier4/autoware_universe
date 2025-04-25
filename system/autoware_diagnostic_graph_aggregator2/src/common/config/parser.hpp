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

#ifndef COMMON__CONFIG__PARSER_HPP_
#define COMMON__CONFIG__PARSER_HPP_

#include "config/yaml.hpp"
#include "graph/logic.hpp"

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::diagnostic_graph_aggregator
{

struct FileConfigData;
struct UnitConfigData;

using FileConfig = std::shared_ptr<FileConfigData>;
using UnitConfig = std::shared_ptr<UnitConfigData>;

struct UnitConfigData
{
  std::string type;
  std::string path;
  std::vector<std::pair<ChildPort::UniquePtr, UnitConfig>> units;
  std::unique_ptr<Logic> logic;
  YAML::Node yaml;
};

struct FileConfigData
{
  std::string original_path;
  std::string resolved_path;
  std::vector<FileConfig> files;
  std::vector<UnitConfig> units;
  YAML::Node yaml;
};

struct LogicConfigData
{
  std::string type;
  std::vector<ChildPort::UniquePtr> ports;
};

class LogicConfig
{
public:
  std::string type() const { return unit_->type; }
  ConfigYaml yaml() const { return ConfigYaml(unit_->yaml); }

  ChildPort * parse(ConfigYaml yaml)
  {
    units_.push_back(std::make_pair(std::make_unique<ChildPort>(), yaml));
    return units_.back().first.get();
  }

protected:
  UnitConfig unit_;
  std::vector<std::pair<ChildPort::UniquePtr, ConfigYaml>> units_;
};

class LogicConfig2 : public LogicConfig
{
public:
  explicit LogicConfig2(UnitConfig unit) { unit_ = unit; }
  auto & ports() { return units_; }
};

struct ParseContext
{
  ParseContext(const std::string & parent, const std::shared_ptr<std::set<std::string>> & visited)
  {
    parent_ = parent;
    visited_ = visited;
  }

  ParseContext child(const std::string & path) { return ParseContext(path, visited_); }

  bool visit(const std::string & path) { return visited_->insert(path).second; }

  std::string file() const { return parent_; }

private:
  std::string parent_;
  std::shared_ptr<std::set<std::string>> visited_;
};

}  // namespace autoware::diagnostic_graph_aggregator

#endif  // COMMON__CONFIG__PARSER_HPP_
