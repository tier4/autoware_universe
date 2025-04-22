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

#include "config/parser.hpp"

#include <string>

namespace autoware::diagnostic_graph_aggregator
{

void load_file(FileConfig & config)
{
  config.required("files");
}

void load_file(const std::string & path, Logger & logger)
{
  logger.info("Loading file: " + path);

  FileConfig config;
  config.path = "./__input__";
  config.yaml["files"][0]["path"] = path;

  logger.info(YAML::Dump(config.yaml));
}

}  // namespace autoware::diagnostic_graph_aggregator
