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

#include "check_graph.hpp"

#include "config/entity.hpp"
#include "config/loader.hpp"
#include "graph/graph.hpp"
#include "utils/logger.hpp"

#include <iostream>
#include <string>

namespace autoware::diagnostic_graph_aggregator
{

void dump_file_tree(const FileConfig & file, const std::string & indent = "")
{
  std::cout << indent << file->resolved_path << std::endl;
  for (const auto & child : file->files) {
    dump_file_tree(child, indent + "  ");
  }
}

void dump_file_tree(const std::string & path)
{
  GraphConfig graph;
  Logger logger;
  load_root_file(graph, path, logger);
  dump_file_tree(graph.root);
}

void dump_file_list(const std::string & path)
{
  GraphConfig graph;
  Logger logger;
  load_root_file(graph, path, logger);
  make_file_list(graph);
  for (const auto & file : graph.files) {
    std::cout << file->resolved_path << std::endl;
  }
}

void dump_unit_tree(const UnitConfig & unit, const std::string & indent = "")
{
  std::cout << indent << unit->type << "(" << unit->path << ")" << std::endl;
  for (const auto & [link, child] : unit->links) {
    dump_unit_tree(child, indent + "  ");
  }
}

void dump_unit_tree(const std::string & path)
{
  GraphConfig graph;
  Logger logger;
  load_root_file(graph, path, logger);
  make_file_list(graph);
  load_unit_tree(graph);
  for (const auto & file : graph.files) {
    for (const auto & unit : file->units) {
      dump_unit_tree(unit);
    }
  }
}

void dump_unit_list(const std::string & path)
{
  GraphConfig graph;
  Logger logger;
  load_root_file(graph, path, logger);
  make_file_list(graph);
  load_unit_tree(graph);
  make_unit_list(graph);

  for (const auto & unit : graph.units) {
    std::cout << unit.use_count() << " " << unit->type << "(" << unit->path << ")" << std::endl;
    for (const auto & [link, child] : unit->links) {
      std::cout << " - " << child->path << " ### " << child->link << std::endl;
    }
  }
}

void dump_config(const std::string & path)
{
  Logger logger;
  const auto graph = load_config(path, logger);

  for (const auto & unit : graph.units) {
    std::cout << unit.use_count() << " " << unit->type << "(" << unit->path << ")" << std::endl;
    for (const auto & [link, child] : unit->links) {
      std::cout << " - " << child->path << " ### " << child->link << std::endl;
    }
  }
}

void dump_graph(const std::string & path)
{
  Logger logger;
  const auto graph = Graph(path, "check", logger);
  graph.dump();
}

void check_graph(const std::string & path)
{
  dump_graph(path);
}

}  // namespace autoware::diagnostic_graph_aggregator

int main(int argc, char ** argv)
{
  (void)argc;
  (void)argv;

  std::cout << "================================================" << std::endl;
  autoware::diagnostic_graph_aggregator::check_graph(
    "$(find-pkg-share autoware_diagnostic_graph_aggregator)/example/graph/main.yaml");
  std::cout << "================================================" << std::endl;
  autoware::diagnostic_graph_aggregator::check_graph(
    "$(find-pkg-share autoware_launch)/config/system/diagnostics/autoware-main.yaml");
  std::cout << "================================================" << std::endl;
  autoware::diagnostic_graph_aggregator::check_graph(
    "$(find-pkg-share autoware_diagnostic_graph_aggregator2)/test/units/sample.yaml");
  std::cout << "================================================" << std::endl;
}
