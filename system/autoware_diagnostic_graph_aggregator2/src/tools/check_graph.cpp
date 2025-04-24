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

#include "config/loader.hpp"
#include "logger/logger.hpp"

#include <iostream>
#include <string>

namespace autoware::diagnostic_graph_aggregator
{

void dump_file_tree(const ConfigFile & file, const std::string & indent = "")
{
  std::cout << indent << file->resolved_path << std::endl;
  for (const auto & child : file->files) {
    dump_file_tree(child, indent + "  ");
  }
}

void dump_file_tree(const std::string & path)
{
  Logger logger;
  const auto root = load_root_file(path, logger);
  dump_file_tree(root);
}

void dump_file_list(const std::string & path)
{
  Logger logger;
  const auto root = load_root_file(path, logger);
  const auto list = make_file_list(root);
  for (const auto & file : list) {
    std::cout << file->resolved_path << std::endl;
  }
}

void dump_unit_tree(const std::string & path)
{
  Logger logger;
  const auto root = load_root_file(path, logger);
  const auto list = make_file_list(root);
  const auto list2 = load_unit_tree(list);
  for (const auto & file : list2) {
    std::cout << file->resolved_path << std::endl;
  }
}

void check_graph(const std::string & path)
{
  dump_unit_tree(path);
}

}  // namespace autoware::diagnostic_graph_aggregator

int main(int argc, char ** argv)
{
  (void)argc;
  (void)argv;
  autoware::diagnostic_graph_aggregator::check_graph(
    "$(find-pkg-share autoware_diagnostic_graph_aggregator)/example/graph/main.yaml");
  /*
  std::cout << "================================================" << std::endl;
  autoware::diagnostic_graph_aggregator::check_graph("$(find-pkg-share
  autoware_launch)/config/system/diagnostics/autoware-main.yaml"); std::cout <<
  "================================================" << std::endl;
  autoware::diagnostic_graph_aggregator::check_graph("$(find-pkg-share
  autoware_diagnostic_graph_aggregator2)/test/files/tree1.yaml");
  */
}
