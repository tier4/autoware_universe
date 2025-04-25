// Copyright 2024 The Autoware Contributors
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

#include "config/yaml.hpp"

#include "config/errors.hpp"

#include <iostream>
#include <string>
#include <vector>

namespace autoware::diagnostic_graph_aggregator
{

ConfigYaml::ConfigYaml(const YAML::Node yaml)
{
  yaml_ = yaml;
}

ConfigYaml ConfigYaml::required(const std::string & name)
{
  // TODO(Takagi, Isamu): check map type.
  if (!yaml_[name]) {
    throw std::runtime_error("Required field not found: " + name);
  }
  const auto node = yaml_[name];
  yaml_.remove(name);
  return ConfigYaml(node);
}

ConfigYaml ConfigYaml::optional(const std::string & name)
{
  // TODO(Takagi, Isamu): check map type.
  if (!yaml_[name]) {
    return ConfigYaml(YAML::Node(YAML::NodeType::Undefined));
  }
  const auto node = yaml_[name];
  yaml_.remove(name);
  return ConfigYaml(node);
}

void ConfigYaml::dump() const
{
  std::cout << YAML::Dump(yaml_) << std::endl;
}

std::vector<ConfigYaml> ConfigYaml::list() const
{
  if (yaml_.IsDefined() && !yaml_.IsSequence()) {
    throw std::runtime_error("Invalid type: files");
  }
  std::vector<ConfigYaml> result;
  for (const auto & node : yaml_) {
    result.push_back(ConfigYaml(node));
  }
  return result;
}

std::string ConfigYaml::text() const
{
  return yaml_.as<std::string>();
}

}  // namespace autoware::diagnostic_graph_aggregator
