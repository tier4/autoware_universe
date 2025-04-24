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

#include <yaml-cpp/yaml.h>

#include <memory>
#include <string>
#include <vector>

namespace autoware::diagnostic_graph_aggregator
{

struct ConfigUnitData
{
  std::string path;
  std::vector<std::shared_ptr<ConfigUnitData>> units;
  // Unit
  // Logic
};

struct ConfigFileData
{
  std::string original_path;
  std::string resolved_path;
  std::vector<std::shared_ptr<ConfigFileData>> files;
  std::vector<std::shared_ptr<ConfigUnitData>> units;
  YAML::Node yaml;
};

using ConfigUnit = std::shared_ptr<ConfigUnitData>;
using ConfigFile = std::shared_ptr<ConfigFileData>;

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

/*
struct ConfigData
{
  ConfigData(const std::string & file, const std::string & tree, const YAML::Node & yaml);
  ConfigData required(const std::string & name);
  ConfigData optional(const std::string & name);

  std::string file;
  std::string tree;
  YAML::Node yaml;
};

struct FileConfig
{
  std::string path;
  YAML::Node yaml;
};
*/

}  // namespace autoware::diagnostic_graph_aggregator

#endif  // COMMON__CONFIG__PARSER_HPP_
