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

#include "config/context.hpp"
#include "config/entity.hpp"
#include "config/errors.hpp"
#include "config/parser.hpp"
#include "config/substitutions.hpp"
#include "config/types/forward.hpp"
#include "config/yaml.hpp"
#include "graph/logic.hpp"
#include "graph/port.hpp"

#include <deque>
#include <filesystem>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

//
#include <iostream>

namespace autoware::diagnostic_graph_aggregator
{

FileConfig load_file(ParseContext context, const std::string & path, Logger & logger);
UnitConfig load_unit(FileConfig file);
std::vector<FileConfig> load_files(ParseContext context, ConfigYaml yaml, Logger & logger);
std::vector<UnitConfig> load_units(ConfigYaml yaml);

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
  result->yaml = ConfigYaml::LoadFile(result->resolved_path);
  result->files = load_files(context.child(resolved_path), result->yaml, logger);
  return result;
}

UnitConfig load_unit(ConfigYaml yaml)
{
  UnitConfig result = std::make_shared<UnitConfigData>();
  result->type = yaml.required("type").text("");
  result->path = yaml.optional("path").text("");
  result->yaml = yaml;

  if (result->type == "link") {
    result->link = yaml.required("link").text("");
  } else {
    LogicEntity entity;
    result->logic = LogicFactory::Create(result->type, LogicConfig(result, &entity));
    for (auto & [port, node] : entity.units) {
      result->units.push_back(std::make_pair(std::move(port), load_unit(node)));
    }
  }
  return result;
}

std::vector<FileConfig> load_files(ParseContext context, ConfigYaml yaml, Logger & logger)
{
  std::vector<FileConfig> result;
  for (ConfigYaml file : yaml.optional("files").list()) {
    result.push_back(load_file(context, file.required("path").text(), logger));
  }
  return result;
}

std::vector<UnitConfig> load_units(ConfigYaml yaml)
{
  std::vector<UnitConfig> result;
  for (ConfigYaml unit : yaml.optional("units").list()) {
    result.push_back(load_unit(unit));
  }
  return result;
}

void load_root_file(GraphConfig & graph, const std::string & path, Logger & logger)
{
  const auto visited = std::make_shared<std::unordered_set<std::string>>();
  graph.root = load_file(ParseContext("root", visited), path, logger);
}

void make_file_list(GraphConfig & graph)
{
  std::vector<FileConfig> result;
  result.push_back(graph.root);
  for (size_t i = 0; i < result.size(); ++i) {
    const auto & files = result[i]->files;
    result.insert(result.end(), files.begin(), files.end());
  }
  graph.files = result;
}

void load_unit_tree(GraphConfig & graph)
{
  for (auto & file : graph.files) {
    file->units = load_units(file->yaml);
  }
}

void make_unit_list(GraphConfig & graph)
{
  std::vector<UnitConfig> result;
  for (const auto & file : graph.files) {
    const auto & units = file->units;
    result.insert(result.end(), units.begin(), units.end());
  }
  for (size_t i = 0; i < result.size(); ++i) {
    for (const auto & [port, unit] : result[i]->units) {
      result.push_back(unit);
    }
  }
  graph.units = result;
}

UnitConfig resolve_links(
  UnitConfig unit, const std::unordered_map<std::string, UnitConfig> & links,
  std::unordered_set<UnitConfig> & visited)
{
  if (!visited.insert(unit).second) {
    throw LinkLoopFound(unit->path);
  }
  if (unit->type != "link") {
    return unit;
  }
  if (!links.count(unit->link)) {
    throw LinkNotFound(unit->link);
  }
  return resolve_links(links.at(unit->link), links, visited);
}

void resolve_links(GraphConfig & graph)
{
  std::unordered_map<std::string, UnitConfig> links;
  for (auto & unit : graph.units) {
    if (!unit->path.empty()) {
      if (!links.insert(std::make_pair(unit->path, unit)).second) {
        throw PathConflict(unit->path);
      }
    }
  }
  for (auto & unit : graph.units) {
    for (auto & [port, child] : unit->units) {
      std::unordered_set<UnitConfig> visited;
      visited.insert(unit);
      child = resolve_links(child, links, visited);
    }
  }
}

void cleanup_files(GraphConfig & graph)
{
  std::vector<UnitConfig> units;
  for (auto & unit : graph.units) {
    if (unit->type != "link") {
      units.push_back(unit);
    }
  }
  graph.root.reset();
  graph.files.clear();
  graph.units = units;
}

void topological_sort(GraphConfig & graph)
{
  std::unordered_map<UnitConfig, int> degrees;
  std::deque<UnitConfig> result;
  std::deque<UnitConfig> buffer;

  // Count degrees of each unit.
  for (const auto & unit : graph.units) {
    for (const auto & [port, child] : unit->units) {
      ++degrees[child];
    }
  }

  // Find initial units that are zero degrees.
  for (const auto & unit : graph.units) {
    // Do not use "at" function because the zero degree units do not exist.
    if (degrees[unit] == 0) {
      buffer.push_back(unit);
    }
  }

  // Sort by topological order.
  while (!buffer.empty()) {
    const auto unit = buffer.front();
    buffer.pop_front();
    for (const auto & [port, child] : unit->units) {
      if (--degrees[child] == 0) {
        buffer.push_back(child);
      }
    }
    result.push_back(unit);
  }

  // Detect circulation because the result does not include the units on the loop.
  if (result.size() != graph.units.size()) {
    throw UnitLoopFound("detect unit loop");
  }
}

GraphConfig load_config(const std::string & path, Logger & logger)
{
  GraphConfig graph;
  load_root_file(graph, path, logger);
  make_file_list(graph);
  load_unit_tree(graph);
  make_unit_list(graph);
  resolve_links(graph);
  cleanup_files(graph);
  topological_sort(graph);
  return graph;
}

}  // namespace autoware::diagnostic_graph_aggregator
