// Copyright 2026 TIER IV, Inc.

#ifndef AUTOWARE__STATIC_FREESPACE_PLANNER__ROUTE_INDEX_LOADER_HPP_
#define AUTOWARE__STATIC_FREESPACE_PLANNER__ROUTE_INDEX_LOADER_HPP_

#include <string>
#include <vector>

class TestRouteIndexLoader;

namespace autoware::static_freespace_planner
{
class RouteIndexLoader
{
public:
  struct RouteDefinition
  {
    std::string name;      // Extract from file name (excluding extension)
    std::string csv_path;  // CSV file absolute path
  };

  explicit RouteIndexLoader(const std::string & map_path);
  std::vector<RouteDefinition> loadRouteDefinitions();

private:
  std::string map_path_;
  std::string getStaticPathDir() const;  // return {map_path}/static_path/
  std::string extractRouteName(const std::string & filename) const;  // exclude extension

  friend class ::TestRouteIndexLoader;
};
}  // namespace autoware::static_freespace_planner
#endif  // AUTOWARE__STATIC_FREESPACE_PLANNER__ROUTE_INDEX_LOADER_HPP_
