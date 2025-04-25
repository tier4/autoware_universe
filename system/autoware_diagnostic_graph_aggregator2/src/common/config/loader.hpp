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

#ifndef COMMON__CONFIG__LOADER_HPP_
#define COMMON__CONFIG__LOADER_HPP_

#include "config/parser.hpp"
#include "utils/logger.hpp"

#include <string>
#include <vector>

namespace autoware::diagnostic_graph_aggregator
{

GraphConfig load_config(const std::string & path, Logger & logger);

void load_root_file(GraphConfig & graph, const std::string & path, Logger & logger);
void make_file_list(GraphConfig & graph);
void load_unit_tree(GraphConfig & graph);
void make_unit_list(GraphConfig & graph);

}  // namespace autoware::diagnostic_graph_aggregator

#endif  // COMMON__CONFIG__LOADER_HPP_
