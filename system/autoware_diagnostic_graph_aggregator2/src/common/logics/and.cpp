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

#include "logics/and.hpp"

#include "config/parser.hpp"
#include "config/yaml.hpp"

namespace autoware::diagnostic_graph_aggregator
{

AndLogic::AndLogic(const LogicConfig & config)
{
  ConfigYaml yaml = config.yaml();
  for (ConfigYaml node : yaml.required("list").list()) {
    ports_.push_back(config.parse(node));
  }
}

OrLogic::OrLogic(const LogicConfig & config)
{
  ConfigYaml yaml = config.yaml();
  for (ConfigYaml node : yaml.required("list").list()) {
    ports_.push_back(config.parse(node));
  }
}

DiagLogic::DiagLogic(const LogicConfig & config)
{
  const auto diag_node = config.yaml().required("node").text();
  const auto diag_name = config.yaml().required("name").text();
  const auto sep = diag_node.empty() ? "" : ": ";
  port_ = config.parse(diag_node + sep + diag_name);
}

struct DummyLogic : public Logic
{
  explicit DummyLogic(const LogicConfig &) {}
};

RegisterLogic<DiagLogic> registration4("diag");
RegisterLogic<AndLogic> registration("and");
RegisterLogic<OrLogic> registration3("or");
RegisterLogic<DummyLogic> registration2("ok");
RegisterLogic<AndLogic> registration5("short-circuit-and");

}  // namespace autoware::diagnostic_graph_aggregator
