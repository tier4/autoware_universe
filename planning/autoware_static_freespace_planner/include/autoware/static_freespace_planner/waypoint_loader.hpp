// Copyright 2026 TIER IV, Inc.

#ifndef AUTOWARE__STATIC_FREESPACE_PLANNER__WAYPOINT_LOADER_HPP_
#define AUTOWARE__STATIC_FREESPACE_PLANNER__WAYPOINT_LOADER_HPP_

#include <string>
#include <vector>

class TestWaypointLoader;

namespace autoware::static_freespace_planner
{
class WaypointLoader
{
public:
  struct Waypoint
  {
    int seq;                // Sequence number (for switching direction)
    double x, y, z;         // Position
    double qx, qy, qz, qw;  // Orientation (quaternion)
    double mps;             // m/s (positive: forward, negative: backward)
  };
  WaypointLoader();

  std::vector<Waypoint> loadWaypoints(const std::string & csv_path);
  Waypoint loadFirstWaypoint(const std::string & csv_path);
  Waypoint loadLastWaypoint(const std::string & csv_path);

  // Load waypoints filtered by seq
  std::vector<Waypoint> loadWaypointsBySeq(const std::string & csv_path, int seq);

  // Get list of available seq numbers
  std::vector<int> getAvailableSeqs(const std::string & csv_path);

private:
  Waypoint parseCSVLine(const std::string & line);

  friend class ::TestWaypointLoader;
};
}  // namespace autoware::static_freespace_planner
#endif  // AUTOWARE__STATIC_FREESPACE_PLANNER__WAYPOINT_LOADER_HPP_
