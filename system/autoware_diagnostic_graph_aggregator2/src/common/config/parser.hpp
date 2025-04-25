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

#include "graph/logic.hpp"

#include <yaml-cpp/yaml.h>

#include <memory>
#include <string>
#include <vector>

namespace autoware::diagnostic_graph_aggregator
{

struct FileConfigData;
struct UnitConfigData;

using FileConfig = std::shared_ptr<FileConfigData>;
using UnitConfig = std::shared_ptr<UnitConfigData>;

struct ChildPort
{
  using UniquePtr = std::unique_ptr<ChildPort>;
};

struct UnitConfigData
{
  std::string type;
  std::string path;
  std::vector<std::pair<ChildPort::UniquePtr, UnitConfig>> units;
  std::unique_ptr<Logic> logic;
};

struct FileConfigData
{
  std::string original_path;
  std::string resolved_path;
  std::vector<FileConfig> files;
  std::vector<UnitConfig> units;
  YAML::Node yaml;
};

class LogicConfig
{
public:
  explicit LogicConfig(UnitConfig unit) { unit_ = unit; }
  std::string type() const { return unit_->type; }

private:
  UnitConfig unit_;
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
