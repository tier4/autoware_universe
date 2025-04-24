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

#include "config/parser.hpp"

#include "config/errors.hpp"

namespace autoware::diagnostic_graph_aggregator
{

/*
ConfigData::ConfigData(const std::string & file, const std::string & tree, const YAML::Node & yaml)
{
  file_ = file;
  tree_ = tree;
  yaml_ = yaml;
}


ConfigData ConfigData::required(const std::string & name)
{
  // TODO(Takagi, Isamu): check map type.
  const auto tree_path = path_.field(name);
  if (!yaml_[name]) {
    throw FieldNotFound(tree_path);
  }
  const auto data = yaml_[name];
  yaml_.remove(name);
  return TreeData(data, tree_path);
}

ConfigData ConfigData::optional(const std::string & name)
{
  // TODO(Takagi, Isamu): check map type.
  const auto tree_path = path_.field(name);
  if (!yaml_[name]) {
    return TreeData(YAML::Node(YAML::NodeType::Undefined), tree_path);
  }
  const auto data = yaml_[name];
  yaml_.remove(name);
  return TreeData(data, tree_path);
}
*/

}  // namespace autoware::diagnostic_graph_aggregator
