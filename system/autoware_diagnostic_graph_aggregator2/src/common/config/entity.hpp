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

#ifndef COMMON__CONFIG__ENTITY_HPP_
#define COMMON__CONFIG__ENTITY_HPP_

#include "config/types/forward.hpp"
#include "config/yaml.hpp"

#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace autoware::diagnostic_graph_aggregator
{

struct GraphConfig
{
  FileConfig root;
  std::vector<FileConfig> files;
  std::vector<UnitConfig> units;
};

struct UnitConfigData
{
  std::string type;
  std::string path;
  std::string link;
  std::unique_ptr<Logic> logic;
  std::vector<std::pair<std::unique_ptr<ChildPort>, UnitConfig>> units;
  std::optional<std::pair<std::unique_ptr<ChildPort>, std::string>> diag;
  ConfigYaml yaml;
};

struct FileConfigData
{
  std::string original_path;
  std::string resolved_path;
  std::vector<FileConfig> files;
  std::vector<UnitConfig> units;
  ConfigYaml yaml;
};

}  // namespace autoware::diagnostic_graph_aggregator

#endif  // COMMON__CONFIG__ENTITY_HPP_
