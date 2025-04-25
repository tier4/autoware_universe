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

#include "config/loader.hpp"

#include "config/errors.hpp"
#include "config/parser.hpp"
#include "config/substitutions.hpp"

#include <filesystem>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::diagnostic_graph_aggregator
{

FileConfig load_file(ParseContext context, const std::string & path, Logger & logger);
UnitConfig load_unit(FileConfig file);
std::vector<FileConfig> load_files(ParseContext context, YAML::Node yaml, Logger & logger);
std::vector<UnitConfig> load_units(YAML::Node yaml);

YAML::Node take_optional(YAML::Node yaml, const std::string & name)
{
  // TODO(Takagi, Isamu): check map type.
  if (!yaml[name]) {
    return YAML::Node(YAML::NodeType::Undefined);
  }
  const auto node = yaml[name];
  yaml.remove(name);
  return node;
}

YAML::Node take_required(YAML::Node yaml, const std::string & name)
{
  // TODO(Takagi, Isamu): check map type.
  if (!yaml[name]) {
    throw std::runtime_error("Required field not found: " + name);
  }
  const auto node = yaml[name];
  yaml.remove(name);
  return node;
}

FileConfig load_file(ParseContext context, const std::string & path, Logger & logger)
{
  const auto resolved_path = substitutions::evaluate(path, context);
  if (!context.visit(resolved_path)) {
    throw SameFileFound(resolved_path);
  }
  if (!std::filesystem::exists(resolved_path)) {
    throw FileNotFound(resolved_path);
  }

  FileConfig result = std::make_shared<FileConfigData>();
  result->original_path = path;
  result->resolved_path = resolved_path;
  result->yaml = YAML::LoadFile(result->resolved_path);
  result->files = load_files(context.child(resolved_path), result->yaml, logger);
  return result;
}

UnitConfig load_unit(YAML::Node yaml)
{
  UnitConfig result = std::make_shared<UnitConfigData>();
  result->type = take_required(yaml, "type").as<std::string>("");
  result->path = take_optional(yaml, "path").as<std::string>("");
  result->yaml = yaml;

  if (result->type != "link") {
    LogicConfig2 config(result);
    result->logic = LogicFactory::Create(config);
    for (auto & [port, node] : config.ports()) {
      result->units.push_back(std::make_pair(std::move(port), load_unit(node.raw())));
    }
  }
  return result;
}

std::vector<FileConfig> load_files(ParseContext context, YAML::Node yaml, Logger & logger)
{
  const auto files = take_optional(yaml, "files");
  if (files.IsDefined() && !files.IsSequence()) {
    throw std::runtime_error("Invalid type: files");
  }
  std::vector<FileConfig> result;
  for (YAML::Node file : files) {
    const auto file_path = take_required(file, "path").as<std::string>();
    result.push_back(load_file(context, file_path, logger));
  }
  return result;
}

std::vector<UnitConfig> load_units(YAML::Node yaml)
{
  const auto units = take_optional(yaml, "units");
  if (units.IsDefined() && !units.IsSequence()) {
    throw std::runtime_error("Invalid type: units");
  }
  std::vector<UnitConfig> result;
  for (YAML::Node unit : units) {
    result.push_back(load_unit(unit));
  }
  return result;
}

FileConfig load_root_file(const std::string & path, Logger & logger)
{
  const auto visited = std::make_shared<std::set<std::string>>();
  return load_file(ParseContext("root", visited), path, logger);
}

std::vector<FileConfig> make_file_list(FileConfig root)
{
  std::vector<FileConfig> result;
  result.push_back(root);
  for (size_t i = 0; i < result.size(); ++i) {
    const auto & files = result[i]->files;
    result.insert(result.end(), files.begin(), files.end());
  }
  return result;
}

std::vector<FileConfig> load_unit_tree(const std::vector<FileConfig> & files)
{
  for (const auto & file : files) {
    file->units = load_units(file->yaml);
  }
  return files;
}

}  // namespace autoware::diagnostic_graph_aggregator
