// Copyright 2026 TIER IV, Inc.

#include "autoware/static_freespace_planner/route_index_loader.hpp"

#include <algorithm>
#include <filesystem>
#include <string>
#include <vector>

namespace autoware::static_freespace_planner
{
RouteIndexLoader::RouteIndexLoader(const std::string & map_path) : map_path_(map_path)
{
}

std::vector<RouteIndexLoader::RouteDefinition> RouteIndexLoader::loadRouteDefinitions()
{
  std::vector<RouteDefinition> route_definitions;
  const std::string static_path_dir = getStaticPathDir();

  // get directory csv files
  for (const auto & entry : std::filesystem::directory_iterator(static_path_dir)) {
    // check for csv extension
    auto ext = entry.path().extension().string();
    if (ext.empty() || ext[0] != '.') {
      continue;
    }

    // convert extension to lower case for case-insensitive comparison
    std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char c) {
      return static_cast<unsigned char>(std::tolower(c));
    });

    // check for csv files
    if (entry.is_regular_file() && ext == ".csv") {
      RouteDefinition route_def;
      route_def.name = extractRouteName(entry.path().filename().string());
      route_def.csv_path = entry.path().string();
      route_definitions.push_back(route_def);
    }
  }

  return route_definitions;
}
std::string RouteIndexLoader::getStaticPathDir() const
{
  return map_path_ + "/static_path/";
}
std::string RouteIndexLoader::extractRouteName(const std::string & filename) const
{
  const auto last_dot_pos = filename.find_last_of('.');
  if (last_dot_pos == std::string::npos) {
    // return the whole filename if no extension found
    return filename;
  }

  // return the filename without extension
  return filename.substr(0, last_dot_pos);
}

}  // namespace autoware::static_freespace_planner
